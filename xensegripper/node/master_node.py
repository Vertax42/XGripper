#!/usr/bin/env python3

from importlib.metadata import PackageNotFoundError, version
import time
import logging
from pathlib import Path
import subprocess
import traceback
from threading import Event
from typing import Sequence

import xensesdk.ezros as ezros
from xensesdk.ezros.ip_tool import is_lan_connected, get_mac_address, get_net_infos
from xensesdk.xenseInterface.scanXenseDevice import scan_xense_devices
from xensegripper._version import __version__ as bundled_xensegripper_version
from xensegripper.node.node_utils import init_logger, launch_subprocess


class MasterNode:
    
    def __init__(self):
        # 注册资源清理函数
        ezros.init(log_level = "INFO", shutdown_callback=self.stop)
        
        self.mac_addr = get_mac_address("eth0")  # 获取当前设备的 MAC 地址
        self.node = ezros.Node(f"master_{self.mac_addr}")
        self.logger = self.node.logger
        init_logger(self.logger.name)
        self.core_service = self.node.create_service(f"master_{self.mac_addr}", True)  # 手动启动
        self.setup_service()

        self.subprocesses = {}  # 存储子进程的字典，key为设备序列号或名称，value为子进程对象
        self.reboot_event: Event = Event()  # 用于重启服务的事件

    def setup_service(self):
        self.core_service.register_callback("system_info", self.system_info)
        self.core_service.register_callback("scan_sensor_sn", self.scan_sensor_sn)
        self.core_service.register_callback("list_camera", self.list_camera)
        self.core_service.register_callback("launch_gripper", self.launch_gripper)
        self.core_service.register_callback("launch_sensor", self.launch_sensor)
        self.core_service.register_callback("launch_camera", self.launch_camera)
        self.core_service.register_callback("check_sensor_online", self.check_sensor_online, private=True)
        self.core_service.register_callback("check_gripper_online", self.check_gripper_online, private=True)
        self.core_service.register_callback("kill_sensor", self.kill_sensor)
        self.core_service.register_callback("kill_gripper", self.kill_gripper)
        self.core_service.register_callback("kill_camera", self.kill_camera)
        self.core_service.register_callback("reboot", self.reboot)

    def run(self):
        """
        Start the master service.
        """
        self.start()
        cnt = 0
        while True:
            try:
                # lan_connected = is_lan_connected()
                if self.reboot_event.is_set():
                    self.logger.warning("No LAN connection detected. Master service will stop.")
                    self.stop()
                    break
                time.sleep(1)
                cnt += 1
                
                if cnt >= 5:  # loop every 5 seconds
                    cnt = 0
                    # 重新扫描设备列表，启动新检测到的设备
                    sensors, _, ego_cam = scan_xense_devices()
                    for serial_number in sensors.keys():
                        if serial_number not in self.subprocesses:
                            self.launch_sensor(serial_number)

                    for cam_id in ego_cam.values():
                        if f"Cam-{self.mac_addr.lower()}-{cam_id}" not in self.subprocesses:
                            self.launch_camera(cam_id)
                
            except Exception as e:
                self.logger.error(f"Error in MasterService: {e}")
                self.logger.error(traceback.format_exc())
                self.stop()
                break

    def reboot(self):
        self.reboot_event.set()  # 设置重启事件
        return True

    def stop(self):
        """
        Stop the all services.
        """
        # 终止所有子进程
        for process_name in list(self.subprocesses.keys()):
            self._kill_process(process_name)
        self.reboot_event.set()
        self.node.shutdown()

    def start(self):
        """
        Start devices.
        """
        self.launch_gripper()
        sensors, _, ego_cam = scan_xense_devices()
        self.logger.info(f"Scaned xense sensors: {list(sensors.keys())}")

        for serial_number in sensors.keys():
            self.launch_sensor(serial_number)

        for cam_id in ego_cam.values():
            self.launch_camera(cam_id)
    
    def system_info(self):
        def get_pkg_version(*pkg_names: str) -> str | None:
            for pkg_name in pkg_names:
                try:
                    return version(pkg_name)
                except PackageNotFoundError:
                    continue
            return None

        xensesdk_ver = get_pkg_version("xensesdk")
        xensegripper_ver = (
            get_pkg_version("xensegripper", "xgripper")
            or bundled_xensegripper_version
        )
        sensors, _, cams = scan_xense_devices()

        ret_dict = {
            "xensegripper_ver": xensegripper_ver,
            "xensesdk_ver": xensesdk_ver,
            "sensor_dict": sensors,
            "cams": [i for i in cams.values()]
        }
        return ret_dict

    def scan_sensor_sn(self):
        sensors, _, _ = scan_xense_devices()
        return sensors

    def list_camera(self):
        _, _, cams = scan_xense_devices()
        return cams

    def check_sensor_online(self, serial_number: str):
        return self.node.service_is_ready(serial_number)
    
    def check_gripper_online(self):
        return self.node.service_is_ready(f"gripper_{self.mac_addr}")
        
    def launch_sensor(self, serial_number: str):
        self.logger.info(f"Launch sensor {serial_number}...")
        cmd = ["xensegripper", "--serial_number", str(serial_number)]
        if not self.check_sensor_online(serial_number):  # 避免重复启动
            process = launch_subprocess(cmd, read_stdout=True, logger=self.logger, log_prefix=f"[SENSOR] ")
            self.subprocesses[serial_number] = process
        return True

    def launch_gripper(self):
        self.logger.info(f"Launch gripper...")
        cmd = ["xensegripper", "--gripper"]
        if not self.check_gripper_online(): # 避免重复启动
            process = launch_subprocess(cmd, read_stdout=True, logger=self.logger, log_prefix=f"[GRIPPER] ")
            self.subprocesses[f"gripper_{self.mac_addr}"] = process
        return True

    def launch_camera(self, cam_id, frame_size: Sequence[int]=(640, 480), frame_rate=50):
        self.logger.info(f"Launch camera {cam_id}...")
        serial_number = f"Cam-{self.mac_addr.lower()}-{cam_id}"
        cmd = [
            "xensegripper", "--camera", serial_number, 
            "--frame_size", str(frame_size[0]), str(frame_size[1]), 
            "--frame_rate", str(frame_rate)
        ]
        if ezros.get_topic_type(serial_number, timeout=0.5) is None:  # 避免重复启动
            process = launch_subprocess(cmd, read_stdout=True, logger=self.logger, log_prefix=f"[CAMERA] ")
            self.subprocesses[serial_number] = process
        return True
    
    def _kill_process(self, process_name: str):
        if process_name in self.subprocesses:
            process = self.subprocesses[process_name]
            if process.poll() is None:
                self.logger.info(f"Terminating {process_name} subprocess...")
                process.terminate()
                try:
                    process.wait(timeout=2)  # 等待进程终止
                except subprocess.TimeoutExpired:
                    self.logger.warning(f"{process_name} subprocess did not terminate in time, killing it...")
                    process.kill()
            del self.subprocesses[process_name]
            return f"{process_name} subprocess terminated."
        else:
            self.logger.warning(f"No {process_name} subprocess to kill.")
            return f"No {process_name} subprocess found."
        
    def kill_gripper(self):
        return self._kill_process(f"gripper_{self.mac_addr}")
    
    def kill_sensor(self, serial_number: str):
        return self._kill_process(serial_number)
    
    def kill_camera(self, cam_id: str):
        serial_number = f"Cam-{self.mac_addr.lower()}-{cam_id}"
        return self._kill_process(serial_number)


if __name__ == '__main__':
    
    master_service = MasterNode()
    master_service.run()
