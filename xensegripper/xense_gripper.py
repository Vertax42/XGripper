from enum import IntEnum
import struct
from typing import Union
import time
from functools import wraps
from abc import ABC, abstractmethod
from threading import Thread
from queue import Queue, Empty

from collections import deque

from .serial_device import SerialDevice
from .utils import deprecated


class ControlMode(IntEnum):
    POSITION = 0
    SPEED = 2


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
    def create(cls, port: str, **kwargs) -> "XenseSerialGripper":
        """
        创建一个 XenseSerialGripper 实例（串口通信）

        Args:
            port (str): 串口设备路径，例如 /dev/ttyUSB0 或 COM3
            **kwargs: 额外参数，如 device_id（默认为 1）

        Returns:
            XenseSerialGripper: 串口夹爪实例
        """
        return XenseSerialGripper(port, **kwargs)

    @abstractmethod
    def release(self):
        """
        释放资源
        """
        pass


class XenseSerialGripper(XenseGripper):
    """
    Direct serial communication with gripper
    """
    def __init__(self, port, device_id = 1):
        self._port = port
        self._device_id = device_id
        self._serial_master = SerialDevice(port)
        self._active_response_enabled = False
        self._active_response_command = None
        self._gripper_status = deque([None], maxlen=1)
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
                        self._gripper_status.append(status)

                elif data[3] == ResponseType.BUTTON_EVENT:
                    status = self._bytes_to_button_status(data)
                    if status is not None:
                        self._button_status.put(ButtonEvent(status))

    def __del__(self):
        self.set_speed(0)
        self.release()

    def release(self):
        self._running = False
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
            status = self.get_gripper_status()

            # Timeout check
            if (time.time() - start_time) > timeout:
                last = f"{status['position']:.3f} mm" if status else "N/A"
                raise TimeoutError(
                    f"Gripper did not reach target position {position} mm "
                    f"within {timeout} seconds. Last position: {last}"
                )

            if status is None:
                time.sleep(poll_interval)
                continue

            current_pos = status["position"]
            if abs(current_pos - position) <= tolerance:
                break

            time.sleep(poll_interval)
        return

    def set_led_color(self, R, G, B):
        """
        设置 WS2812B 灯颜色
        参数 G, R, B: 绿色、红色、蓝色亮度值,范围[0, 255]
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
        self._serial_master.send(data_packed)

        return self._gripper_status[-1]

    def get_button_status(self):
        """获取按钮状态"""
        try:
            ret = self._button_status.get(block=False)
        except Empty:
            return ButtonEvent.NONE
        return ret

    def register_button_callback(self, event_type: str, callback):
        """
        注册按钮事件回调函数（后台线程方式）

        Args:
            event_type (str): 事件类型，支持 "CLICK", "DOUBLE_CLICK", "LONG_PRESS", "PRESS", "RELEASE"
            callback (function): 回调函数，无参数
        """
        if not hasattr(self, '_button_callbacks'):
            self._button_callbacks = {}
            self._button_cb_thread = Thread(target=self._button_callback_loop, daemon=True)
            self._button_cb_thread.start()
        self._button_callbacks[ButtonEvent[event_type]] = callback

    def _button_callback_loop(self):
        while self._running:
            try:
                event = self._button_status.get(timeout=0.1)
                if event in self._button_callbacks:
                    self._button_callbacks[event]()
            except Empty:
                continue

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
