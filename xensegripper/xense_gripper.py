from enum import IntEnum
import struct
from typing import Union
import time
from contextlib import contextmanager
from functools import wraps
from abc import ABC, abstractmethod
from threading import Thread
from queue import Queue, Empty

import xensesdk.ezros as ezros
from xensesdk.ezros import Node
from xensesdk.ezgl.functions import CircularBuffer

from .serial_device import SerialDevice
from .utils import deprecated
from .controller import SafeCtlImpl


class ControlMode(IntEnum):
    POSITION = 0
    SPEED = 2
    SAFE = 1

class Command(IntEnum):
    POSITION_CONTROL    = 0XA4
    GRIPPER_STATUS      = 0X9C
    CALIBRATE           = 0X21
    ACTIVE_RESPONSE     = 0XB6
    WRITE_WS2812B       = 0XA5      # 设置LED灯颜色
    SPEED_CONTROL       = 0XC3      # 速度闭环

class ResponseType(IntEnum):
    GRIPPER_STATUS = 0x9c
    BUTTON_EVENT = 0xB8

class ButtonEvent(IntEnum):
    RELEASE = 0x00
    PRESS = 0x01
    CLICK = 0x02
    DOUBLE_CLICK = 0x03
    LONG_PRESS = 0x04
    NONE = 0x08


class XenseGripper(ABC):

    ControlMode = ControlMode

    @abstractmethod
    def set_position(self, position: float, vmax: float , fmax: float):
        pass
    
    @abstractmethod
    def set_position_sync(self, position: float, vmax: float, fmax: float, 
                           tolerance: float = 0.01, timeout: float = 5.0, 
                           poll_interval: float = 0.05):
        pass

    @abstractmethod
    def get_gripper_status(self):
        pass
    
    @classmethod
    def create(cls, mac_addr=None, **kwargs) -> Union["XenseTCPGripper", "XenseSerialGripper"]:
        """
        创建一个 XenseGripper 实例，自动选择通信方式（串口或 TCP/IP）

        根据传入参数自动决定使用串口实现（`XenseSerialGripper`）或
        网络实现（`XenseTCPGripper`）创建一个夹爪实例。

        Args:
            mac_addr (str, optional): 如果提供 IP 地址，则使用 TCP 连接到远程夹爪。
                                        否则使用本地串口连接。
            **kwargs: 额外参数（如 `port`），仅在串口连接时使用。

        Returns:
            XenseGripper: 实现 `Gripper` 接口的夹爪实例，具体为串口或 TCP 实现。
        """
        if mac_addr is not None:
            return XenseTCPGripper(mac_addr)
        else:
            return XenseSerialGripper(kwargs["port"])
    
    @abstractmethod
    def release(self):
        """
        释放资源
        """
        pass


class XenseSerialGripper(XenseGripper):
    """
    Direct communication with gripper
    """
    def __init__(self, port, device_id = 1):
        self._port = port
        self._device_id = device_id
        self._serial_master = SerialDevice(port)
        self._active_response_enabled = False
        self._active_response_command = None
        self._gripper_status = CircularBuffer(1)
        self._button_status = Queue(maxsize=5)
        self._running = True
        self._worker = Thread(target=self._run_loop, daemon=True)
        self._worker.start()
    
    def _bytes_to_button_status(self, res_bytes):
        if len(res_bytes) == 13:
            return res_bytes[5]
        else:
            return None

    def _run_loop(self):
        while self._running:
            data = self._serial_master.receive()
            if data is not None:
                if data[3] == ResponseType.GRIPPER_STATUS:
                    status = self._bytes_to_gripper_status(data)
                    if status is not None:
                        self._gripper_status.put(status)

                elif data[3] == ResponseType.BUTTON_EVENT:
                    status = self._bytes_to_button_status(data)
                    if status is not None:
                        self._button_status.put(ButtonEvent(status))
                
    def __del__(self):
        self.set_speed(0)
        self.release()
    
    def release(self):
        self._serial_master.close()
    
    def set_position(self, position, vmax=80.0, fmax=27.0):
        """
        Set the target position of the Gripper.

        Args:
            position (float): Target position of the gripper in millimeters (mm). 
                              Must be in the range (0, 85). 
                              0 mm means fully open, 85 mm means fully closed.
            vmax (float, optional): Maximum speed of motion in mm/s. 
                                    Must be in the range (0, 350). 
                                    Default is 80 mm/s.
            fmax (float, optional): Maximum output force in Newtons (N). 
                                    Must be in the range (0, 60). 
                                    Default is 27 N.

        Raises:
            ValueError: If any of the input arguments are outside their allowed physical limits.

        """
        if not 0 <= vmax <= 350:
            raise ValueError(f"vmax {vmax} out of range, please input value in 0~350(mm/s)")
        if not 0 <= fmax <= 60:
            raise ValueError(f"fmax {fmax} out of range, please input value in 0~60(N)")
        if not 0 <= position <= 85:
            raise ValueError(f"vmax {position} out of range, please input value in 0~85(mm)")
        position = 85 - position # reverse user control
        vmax /= 0.1      # (0.1mm/s)/LSB, 设置区间0-200mm/s
        fmax /= 0.01     # 0.01N/LSB, 设置区间0-60N
        position /= 0.01 # 0.01mm/LSB，即1000代表10mm, 设置区间0-85mm, 0 代表闭合， 85 代表张开
        command = Command.POSITION_CONTROL.to_bytes(1, 'little')
        force_max_bytes  = int(fmax).to_bytes(2, 'little')
        v_max_bytes = int(vmax).to_bytes(2, 'little')
        position_bytes  = int(position).to_bytes(2, 'little')
        data_bytes = command + (0).to_bytes(1, 'little') + force_max_bytes + v_max_bytes + position_bytes
        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        self._serial_master.send(data_packed)

    def set_speed(self, vmax, fmax=27):
        """
        速度闭环控制
        fmax: 最大推力 (N)，范围 [0, 60]
        vmax: 目标速度 (mm/s)，范围 [0, 440]
        """
        if not -440 <= vmax <= 440:  # 预留负速度（如果固件支持）
            raise ValueError(f"vmax {vmax} out of range, please input value in -440~440 (mm/s)")
        if not 0 <= fmax <= 60:
            raise ValueError(f"fmax {fmax} out of range, please input value in 0~60 (N)")

        # 单位换算
        fmax /= 0.01   # 0.01N / LSB
        vmax /= 0.1    # 0.1mm/s / LSB

        command = Command.SPEED_CONTROL.to_bytes(1, 'little')
        force_max_bytes = int(fmax).to_bytes(2, 'little', signed=False)
        v_max_bytes     = int(vmax).to_bytes(2, 'little', signed=True)

        # ➕ 补齐 2 字节 padding，使总长度 = 8
        data_bytes = command + (0).to_bytes(1, 'little') + force_max_bytes + v_max_bytes + (0).to_bytes(2, 'little')

        assert len(data_bytes) == 8

        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        self._serial_master.send(data_packed)
    
    def set_position_sync(self, position, vmax, fmax, tolerance = 0.01, timeout = 5, poll_interval = 0.05):
        """
        Move the gripper to a target position and block until the target is reached or timeout occurs.

        Parameters:
            position (float): Target position of the gripper in millimeters (mm).
                            Must be in the range (0, 85). 
                            0 mm means fully open, 85 mm means fully closed.
            vmax (float): Maximum speed of motion in mm/s.
                        Must be in the range (0, 350).
                        Default is 80 mm/s.
            fmax (float): Maximum output force in Newtons (N).
                        Must be in the range (0, 60).
                        Default is 27 N.
            tolerance (float, optional): Allowed position error in mm to consider motion complete.
                                        Default is 0.01 mm.
            timeout (float, optional): Maximum time in seconds to wait for the target to be reached.
                                        Default is 5.0 seconds.
            poll_interval (float, optional): Time interval in seconds between position checks.
                                            Default is 0.05 seconds.
        """
        self.set_position(position, vmax, fmax)

        start_time = time.time()

        while True:
            current_pos = self.get_gripper_status()["position"]  # Must be implemented in the driver layer

            # Check if position is within the tolerance
            if abs(current_pos - position) <= tolerance:
                break

            # Timeout check
            if (time.time() - start_time) > timeout:
                raise TimeoutError(
                    f"Gripper did not reach target position {position} mm "
                    f"within {timeout} seconds. Last position: {current_pos:.3f} mm"
                )

            time.sleep(poll_interval)
        return 

    def set_led_color(self, R, G, B):
        """
        设置 WS2812B 灯颜色
        参数 G, R, B：绿色、红色、蓝色亮度值，范围 0~255
        """
        if not (0 <= G <= 255 and 0 <= R <= 255 and 0 <= B <= 255):
            raise ValueError("RGB 颜色值必须在 0~255 范围内")
        
        command = Command.WRITE_WS2812B.to_bytes(1, 'little')
        g_byte = G.to_bytes(1, 'little')
        r_byte = R.to_bytes(1, 'little')
        b_byte = B.to_bytes(1, 'little')
        reserved = (0).to_bytes(1, 'little') * 4  # 补足到 8 字节
        
        data_bytes = command + g_byte + r_byte + b_byte + reserved
        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        self._serial_master.send(data_packed)

    def _bytes_to_gripper_status(self, res_bytes):
        if len(res_bytes) == 13:
            # Byte 4: motor temperature (int8_t), in °C (1°C/LSB)
            temperature = res_bytes[4]

            # Bytes 5-6: gripper output force (int16_t), scaled by 0.01 N/LSB
            force = struct.unpack('<h', res_bytes[5:7])[0] * 0.01

            # Bytes 7-8: gripper speed (int16_t), scaled by 0.1 mm/s/LSB
            vel = struct.unpack('<h', res_bytes[7:9])[0] * 0.1

            # Bytes 9-10: gripper position (int16_t), scaled by 0.01 mm/LSB
            position = struct.unpack('<h', res_bytes[9:11])[0] * 0.01
            # print(f"position: {position} mm, velocity: {vel} mm/s, force: {force} N, temperature: {temperature} °C")
            gripper_status = {
                'position': 85-position,
                'velocity': vel,
                'force': force,
                'temperature': temperature
            }
            return gripper_status
        else:
            return None

    def get_gripper_status(self):
        """Retrieve the gripper status, including motor temperature, output force, speed, and position.

        Returns:
            dict: gripper status including: position, velocity, force and temperature 
        """
        # send command 
        command = Command.GRIPPER_STATUS.to_bytes(1, 'little')
        data_bytes = command + (0).to_bytes(1, 'little')*7
        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        # self._serial_master.clear_rx_buf()  # NOTE: Clear receive buffer before sending command
        self._serial_master.send(data_packed)
        
        return self._gripper_status.get()
    
    def get_button_status(self):
        """每 5ms 在 gripper server node 调用一次"""
        try:
            ret = self._button_status.get(block=False)
        except Empty:
            return ButtonEvent.NONE
        return ret

    # @deprecated("Currently useless, may be removed in future")
    def calibrate(self):
        ##NOTE: Just reset gripper please
        command = Command.CALIBRATE.to_bytes(1, 'little')
        data_bytes = command + (0).to_bytes(1, 'little')*7
        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        self._serial_master.send(data_packed)
        return 0
    
    @deprecated("Buggy, will remove in future")
    def set_active_response(self, enable=True, period=10):
        ##BUG: Do not use, cause reading delay 
        self._active_response_enabled = enable
        self._active_response_command = Command.GRIPPER_STATUS

        command = Command.ACTIVE_RESPONSE.to_bytes(1, 'little')
        response_type = Command.GRIPPER_STATUS.to_bytes(1, 'little')
        enable = int(enable).to_bytes(1, 'little')
        period = int(period).to_bytes(2, 'little') # ms
        data_bytes = command + response_type + enable + period + (0).to_bytes(1, 'little')*3
        data_packed = self._serial_master.build_packet(self._device_id, data_bytes)
        self._serial_master.send(data_packed)


class GripperClientNode(Node):
    
    def __init__(
        self,
        mac_addr
    ):
        assert isinstance(mac_addr, str), "Device MAC must be a string"
        super().__init__(name=f"gripper_{mac_addr}*")
        self._mac_addr = mac_addr
        self.buffer = CircularBuffer(2)

        # 夹爪自启动
        if not self.service_is_ready(f"gripper_{mac_addr}"):
            self.logger.info(f"Gripper not started, starting now...")
            ret = self.call_service(f"master_{mac_addr}", "launch_gripper")
            if ret is None:
                raise Exception(f"Failed to launch gripper_{mac_addr}")

        self.client = self.create_client(f"gripper_{mac_addr}", timeout=8, interval=0.1, quiet=True)
        if self.client is None:
            self.logger.error(f"Failed to create client for gripper {mac_addr}")
            
        self.subscriber = self.create_subscriber(ezros.BytesMessage, f"gripper_{mac_addr}", self.on_fetch_data, 1)
        self.publisher = self.create_publisher(ezros.BytesMessage, f"gripper_control_{mac_addr}", 1)

        self.button_queue = Queue()
        self.button_callbacks = {}
        self.button_thread = None

    def register_button_callback(self, event_type: ButtonEvent, callback):
        """
        注册按钮事件回调函数

        Args:
            event_type (ButtonEvent): 事件类型，支持 "CLICK", "DOUBLE_CLICK", "LONG_PRESS", "PRESS", "RELEASE"
            callback (function): 回调函数，接受一个参数，表示按钮状态
        """
        self.button_callbacks[event_type] = callback
        if self.button_thread is None:
            self.button_thread = Thread(target=self._button_event_loop, daemon=True)
            self.button_thread.start()

    def _button_event_loop(self):
        while True:
            key = self.button_queue.get(block=True)
            if key in self.button_callbacks:
                self.button_callbacks[key]()
            if key == "QUIT":
                break
            time.sleep(0.01)
    
    def shutdown(self):
        self.button_queue.put("QUIT")
        super().shutdown()
        
    def on_fetch_data(self, msg):
        data = msg.get_data()
        
        if isinstance(data, dict) and "button" in data:
            if data["button"] != ButtonEvent.NONE:
                self.button_queue.put(data["button"])

        return self.buffer.put(data)
    
    def set_gripper_position(self, position, vmax, fmax):
        self.client.call_async("set_position", position, vmax, fmax)

    def set_gripper_speed(self, velocity, fmax):
        self.client.call_async("set_speed", velocity, fmax)

    def set_led_color(self, r, g, b):
        self.client.set_led_color(r, g, b)
    
    def calibrate(self):
        self.client.call_async("calibrate")


class DirectCtl:
    def __init__(self, gripper_client: GripperClientNode):
        self.gripper_client = gripper_client

    def set_position(self, position, vmax=80.0, fmax=27.0):
        """
        Set the target position of the Gripper.
        """
        self.gripper_client.set_gripper_position(position, vmax, fmax)
    
    def set_speed(self, velocity, fmax=27):
        """
        速度闭环控制
        fmax: 最大推力 (N)，范围 [0, 60]
        vmax: 目标速度 (mm/s)，范围 [0, 440]
        """
        self.gripper_client.set_gripper_speed(velocity, fmax)


class SafeCtl:
    def __init__(self, gripper_client: GripperClientNode, serial_number=None):
        from xensesdk import Sensor
        import sys
        if serial_number is None:
            # find sensor
            MASTER_SERVICE = f"master_{gripper_client._mac_addr}"
            # find all sensors
            ret = gripper_client.call_service(MASTER_SERVICE, "scan_sensor_sn")
            if ret is None:
                print(f"Failed to scan sensors")
                sys.exit(1)
            else:
                print(f"Found sensors: {ret}, using the first one.")
            serial_number = list(ret.keys())[1]
        else:
            print(f"Using provided serial number: {serial_number}")

        # create a sensor
        self.gripper_client = gripper_client
        sensor = Sensor.create(serial_number, mac_addr=gripper_client._mac_addr)
        self._controller: SafeCtlImpl = SafeCtlImpl(sensor, self.gripper_client)
        self._controller.start()

    def set_position(self, position, vmax=80.0, fmax=27.0):
        """
        Set the target position of the Gripper.
        """
        self._controller.set_target_position(position)
    
    def set_speed(self, velocity, fmax=27):
        raise NotImplementedError("SafeCtl does not support speed control, use DirectCtl instead.")

    def set_control_param(self, F_target, kp, ki, kd):
        if F_target is not None:
            self._controller.F_target = F_target
        if kp is not None:
            self._controller.pid_ctl.Kp = kp
        if ki is not None:
            self._controller.pid_ctl.Ki = ki
        if kd is not None:
            self._controller.pid_ctl.Kd = kd


class XenseTCPGripper(XenseGripper):
    """
    Direct communication with gripper
    """
    def require_mode(expected_mode: ControlMode):
        def deco(fn):
            @wraps(fn)
            def wrapper(self, *args, **kwargs):
                mode = self._ctl_mode
                if mode != expected_mode:
                    raise RuntimeError(f"{fn.__name__} 需要 {expected_mode.name} 模式，当前为 {mode.name}")
                return fn(self, *args, **kwargs)
            return wrapper
        return deco

    def __init__(self, mac_addr):
        self._mac_addr = mac_addr 
        self.gripper_client = GripperClientNode(mac_addr)
        self._ctl = DirectCtl(self.gripper_client)
        self._ctl_mode = ControlMode.POSITION
        time.sleep(0.1)

    def register_button_callback(self, event_type: str, callback):
        assert event_type in ["CLICK", "DOUBLE_CLICK", "LONG_PRESS", "PRESS", "RELEASE"], \
            "Invalid event type, valid types are: CLICK, DOUBLE_CLICK, LONG_PRESS, PRESS, RELEASE"
        self.gripper_client.register_button_callback(ButtonEvent[event_type], callback)
 
    def release(self):
        self.gripper_client.shutdown()

    @contextmanager
    def mode(self, mode: Union[ControlMode, str], serial_number=None):
        """
        上下文管理器：进入时切换到 SPEED 模式，退出时自动停机并恢复原模式
        """
        mode = ControlMode[mode] if isinstance(mode, str) else mode
        prev_mode = self._ctl_mode
        self.enable_mode(mode, serial_number)
        try:
            yield self
        finally:
            self.disable_mode()
            self.enable_mode(prev_mode)

    def enable_mode(self, mode: ControlMode, serial_number=None):
        if self._ctl_mode == mode:
            return
        self._ctl_mode = mode
        if mode == ControlMode.POSITION:
            self._ctl = DirectCtl(self.gripper_client)
        elif mode == ControlMode.SAFE:
            self._ctl = SafeCtl(self.gripper_client, serial_number)
    
    def disable_mode(self):
        if self._ctl_mode == ControlMode.SPEED:
            try:
                self.set_speed(0.0, fmax=27)
                print("Stopped speed control")
            except Exception as e:
                print(f"Error stopping speed control: {e}")
        elif self._ctl_mode == ControlMode.SAFE:
            try:
                self._ctl._controller.stop()
                print("Stopped safe control")
            except Exception as e:
                print(f"Error stopping safe control: {e}")
        
    def set_position(self, position, vmax=80.0, fmax=27.0):
        """
        Set the target position of the Gripper.

        Args:
            position (float): Target position of the gripper in millimeters (mm). 
                              Must be in the range (0, 85). 
                              85 mm means fully open, 0 mm means fully closed.
            vmax (float, optional): Maximum speed of motion in mm/s. 
                                    Must be in the range (0, 350). 
                                    Default is 80 mm/s.
            fmax (float, optional): Maximum output force in Newtons (N). 
                                    Must be in the range (0, 60). 
                                    Default is 27 N.

        Raises:
            ValueError: If any of the input arguments are outside their allowed physical limits.

        """
        self._ctl.set_position(position, vmax, fmax)
    
    @require_mode(ControlMode.SPEED)
    def set_speed(self, velocity, fmax=27):
        """
        速度闭环控制
        fmax: 最大推力 (N)，范围 [0, 60]
        vmax: 目标速度 (mm/s)，范围 [0, 440]
        """
        self._ctl.set_speed(velocity, fmax)

    @require_mode(ControlMode.SAFE)
    def set_control_param(self, stiffness=None, kp=None, ki=None, kd=None):
        self._ctl.set_control_param(stiffness, kp, ki, kd)

    @require_mode(ControlMode.SAFE)
    def get_control_param(self):
        return (self._ctl._controller.F_target, 
                self._ctl._controller.pid_ctl.Kp, 
                self._ctl._controller.pid_ctl.Ki, 
                self._ctl._controller.pid_ctl.Kd)

    def set_position_sync(self, position, vmax, fmax, tolerance = 0.01, timeout = 5, poll_interval = 0.05):
        """
        Move the gripper to a target position and block until the target is reached or timeout occurs.

        Parameters:
            position (float): Target position of the gripper in millimeters (mm). 
                              Must be in the range (0, 85). 
                              85 mm means fully open, 0 mm means fully closed.
            vmax (float, optional): Maximum speed of motion in mm/s. 
                                    Must be in the range (0, 350). 
                                    Default is 80 mm/s.
            fmax (float, optional): Maximum output force in Newtons (N). 
                                    Must be in the range (0, 60). 
                                    Default is 27 N.
            tolerance (float, optional): Allowed position error in mm to consider motion complete.
                                        Default is 0.01 mm.
            timeout (float, optional): Maximum time in seconds to wait for the target to be reached.
                                        Default is 5.0 seconds.
            poll_interval (float, optional): Time interval in seconds between position checks.
                                            Default is 0.05 seconds.
        """
        

        start_time = time.time()

        while True:
            self._ctl.set_position(position, vmax, fmax)
            # Must be implemented in the driver layer
            data_rec = self.gripper_client.buffer.get()
            if data_rec is None:
                print("pass")
                continue

            current_pos = data_rec["position"]

            # Check if position is within the tolerance
            if abs(current_pos - position) <= tolerance:
                break

            # Timeout check
            if (time.time() - start_time) > timeout:
                raise TimeoutError(
                    f"Gripper did not reach target position {position} mm "
                    f"within {timeout} seconds. Last position: {current_pos:.3f} mm"
                )

            time.sleep(poll_interval)
        return True

    def get_gripper_status(self):
        full_data = self.gripper_client.buffer.get()
        if full_data is not None:
            return {k: v for k, v in full_data.items() if k != 'button'}
        else:
            return None
    
    def get_button_status(self):
        full_data = self.gripper_client.buffer.get()
        if full_data is not None:
            return full_data["button"]
        else:
            return None

    def open_gripper(self):
        self._ctl.set_position(85, vmax=40, fmax=27)

    def close_gripper(self):
        self._ctl.set_position(0, vmax=40, fmax=27)
    
    def set_led_color(self, r, g, b):
        """
        设置 WS2812B 灯颜色
        参数 G, R, B：绿色、红色、蓝色亮度值，范围 0~255
        """
        self.gripper_client.set_led_color(r, g, b)
    
    def calibrate(self):
        self.gripper_client.calibrate()