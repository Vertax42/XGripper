#!/usr/bin/env python3

import os
import logging
from pathlib import Path
import time
from threading import Event, Thread, Lock

from xensesdk.xenseInterface.XenseSensor import Sensor
import xensesdk.ezros as ezros


class XenseSensorNode:

    def __init__(
        self,
        serial_number: str,
    ):
        ezros.init()
        self.node = ezros.Node(serial_number)
        self._logger = self.node.logger

        self._cache_types = set()  # 当前需要发布的数据类型
        self._serial_number = serial_number
        
        self.running_event: Event = Event()
        self.suspend_event: Event = Event()
        self.lock = Lock()
        self.rectify_size=(96,160)
        
        # retry creating sensor when disconnected
        self.max_retries = 10
        
        # 创建 Sensor 实例
        self._logger.info("Begin to create sensor...")
        self.sensor = None
                   
        # init publisher
        self.publisher = self.node.create_publisher(ezros.BytesMessage, self._serial_number, 1)
        self.timer = self.node.create_timer(15, self.on_timeout)

        # pub monitor
        ezros.get_global_monitor(0, check_interval=30, timeout_minutes=10)
        ezros.monitor_publisher(self._serial_number, cleanup_callback=self.suspend_sensor)

        # Define services
        self.service = self.node.create_service(self._serial_number, False)
        self.service.register_callback("get_img", self.get_img)
        self.service.register_callback("terminate_node", self.terminate_node)
        self.service.register_callback("get_time", self.get_time, private=True)  # for debugging
        self.service.register_callback("config", self.get_config, private=True)
        self.service.register_callback("reset_fetch_types", self.reset_fetch_types)
        self.service.register_callback("calibrate", self.reset_reference)
        self.service.register_callback("check_is_alive", self.check_is_alive)
        self.service.register_callback("export_runtime", self.export_runtime, private=True)
        self.service.register_callback("reset_sensor", self.reset_sensor)

        self.service.start(threaded=False)  # block
    
    def reset_sensor(self, width, height):
        self.sensor._config_manager.rectify_config.width = width
        self.sensor._config_manager.rectify_config.height = height
        rectify_params = self.sensor._config_manager.calc_rectify_params()
        self.sensor._real_camera.rectify.set(**rectify_params)
    
    def create_sensor(self):
        """
        创建 Sensor 实例
        """
        for attempt in range(self.max_retries):
            try:
                self.sensor = Sensor.create(
                    self._serial_number,
                    config_path=Path.home()/"xense_workspace/config",
                    rectify_size=self.rectify_size
                )
                self._logger.info("Sensor created successfully.")
                self.running_event.set()
                break
            except Exception as e:
                if attempt+1 < self.max_retries:
                    self._logger.warning(f"Attempt {attempt+1}/{self.max_retries} to create sensor {self._serial_number} failed, Error: {str(e)}")
                    time.sleep(2)
                else:
                    self._logger.critical(f"Failed to create sensor after {self.max_retries} attempts. Exiting...")
                    self.running_event.clear()
                    self.terminate_node()
                    exit(1)
        
    def reset_fetch_types(self, types):
        """
        Reset the types of data to be fetched from the sensor.
        :param types: List of data types to fetch.
        """
        self._cache_types |= set(types)
        return True

    def reset_reference(self):
        with self.lock:
            self.sensor.resetReferenceImage()
        return 0

    def on_timeout(self):
        if not self.running_event.is_set():
            self.create_sensor()
        
        if not self._cache_types:
            # self.publisher.publish({})
            return
        
        # Check if the sensor is still alive
        if not self.sensor.getNetworkStatus():
            self.publisher.publish(ezros.BytesMessage(data="ERROR"))
            self.running_event.clear()
            
        with self.lock:
            data = self.sensor._dag_runner.selectSensorInfoDict(*self._cache_types)
        self.publisher.publish(ezros.BytesMessage(data={k: v for k, v in data.items() if k in self._cache_types}))

    def get_img(self):
        with self.lock:
            return self.sensor.selectSensorInfo(Sensor.OutputType.Rectify)
            
    def get_time(self):
        import time
        return time.time()
    
    def get_config(self):
        if self.sensor is None:
            self._logger.error("Sensor is not initialized.")
            return None
        if self.suspend_event.is_set():
            self.resume_sensor()
        return self.sensor._config_manager

    def terminate_node(self):
        try:
            self.publisher.publish(ezros.BytesMessage(data="ERROR"))
            self.sensor.release()
        except Exception as e:
            pass
        self._logger.info("Sensor released.")
        self.node.shutdown()
        return 0

    def check_is_alive(self):
        return self.sensor.getNetworkStatus()
    
    def export_runtime(self):
        return self.sensor.exportRuntimeConfig(binary=True)

    def suspend_sensor(self):
        if not self.suspend_event.is_set():  # 避免重复设置
            self._cache_types.clear()  # 清空需要发布的数据类型
            if self.sensor and self.sensor._real_camera:
                self.sensor._real_camera.destroyFetchThread()
            self.suspend_event.set()

    def resume_sensor(self):
        if self.suspend_event.is_set():  # 避免重复恢复
            ezros.monitor_publisher(self._serial_number, cleanup_callback=self.suspend_sensor)
            self._cache_types.clear()  # 清空需要发布的数据类型
            if self.sensor and self.sensor._real_camera:
                if self.sensor._real_camera._fetch_img_thread is None or not self.sensor._real_camera._fetch_img_thread.is_alive():
                    self.sensor._real_camera._fetch_img_thread = Thread(target=self.sensor._real_camera._on_fetch_img, daemon=True)
                    self.sensor._real_camera._streaming = True
                    self.sensor._real_camera._fetch_img_thread.start()
            self.suspend_event.clear()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Xense Data Publisher")
    parser.add_argument('--serial_number', type=str, default='OP000064', help='Serial number of the sensor')

    args = parser.parse_args()

    # Create node
    XenseSensorNode(serial_number=args.serial_number)


if __name__ == '__main__':
    main()
