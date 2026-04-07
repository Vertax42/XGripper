from enum import IntEnum
import struct
from typing import Union
import time
from contextlib import contextmanager
from functools import wraps
from abc import ABC, abstractmethod
from threading import Thread
from queue import Queue, Empty

# ezros is only required for XenseTCPGripper (network/FlareGrip path).
# XenseSerialGripper works without it.
try:
    import xensesdk.ezros as ezros
    from xensesdk.ezros import Node
    _EZROS_AVAILABLE = True
except ImportError:
    _EZROS_AVAILABLE = False
    ezros = None
    Node = object  # fallback base so GripperClientNode class body is parseable

from collections import deque

from .serial_device import SerialDevice
from .utils import deprecated
from .controller import SafeCtlImpl


class CircularBuffer:
    """简单的循环缓冲区实现"""

    def __init__(self, size):
        self.size = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.count = 0

    def put(self, item):
        self.buffer[self.head] = item
        self.head = (self.head + 1) % self.size
        if self.count < self.size:
            self.count += 1
        else:
            self.tail = (self.tail + 1) % self.size

    def get(self):
        if self.count == 0:
            return None
        item = self.buffer[self.tail]
        self.tail = (self.tail + 1) % self.size
        self.count -= 1
        return item

    def clear(self):
        self.buffer = [None] * self.size
        self.head = 0
        self.tail = 0
        self.count = 0

    def is_empty(self):
        return self.count == 0

    def is_full(self):
        return self.count == self.size


class ControlMode(IntEnum):
    POSITION = 0
    SPEED = 2
    SAFE = 1

class Command(IntEnum):
    POSITION_CONTROL    = 0XA4
    GRIPPER_STATUS      = 0X9C
    CALIBRATE           = 0X20      # 标定
    CALIBRATE_ENCODER   = 0X21      # 编码器标定
    ACTIVE_RESPONSE     = 0XB6
    WRITE_WS2812B       = 0XA5      # 设置LED灯颜色
    SPEED_CONTROL       = 0XC3      # 速度闭环
    STOP_MOTOR          = 0X81      # 停止电机
    READ_BOARD_SN       = 0XD0      # 读取板载MCU序列号

class ResponseType(IntEnum):
    GRIPPER_STATUS   = 0x9c
    BUTTON_EVENT     = 0xB8
    CALIBRATE_STATUS = 0x20

class ButtonEvent(IntEnum):
    RELEASE      = 0x00
    PRESS        = 0x01
    CLICK        = 0x02
    DOUBLE_CLICK = 0x03
    LONG_PRESS   = 0x04
    NONE         = 0x08


class XenseGripper(ABC):

    ControlMode = ControlMode

    @abstractmethod
    def set_position(self, position: float, vmax: float, fmax: float):
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
        创建一个 XenseGripper 实例，自动选择通信方式。

        Args:
            mac_addr (str, optional): FlareGrip 设备 MAC 地址，使用 ezros 网络通信。
                                      若为 None，则使用串口直连（需提供 ``port`` kwarg）。
            **kwargs: 串口参数，如 ``port``、``baudrate``、``timeout``（仅串口模式使用）。

        Returns:
            XenseTCPGripper  — 当 mac_addr 不为 None 时（需安装 xensesdk）
            XenseSerialGripper — 当提供 port 时（仅需 pyserial）
        """
        if mac_addr is not None:
            return XenseTCPGripper(mac_addr)
        else:
            return XenseSerialGripper(kwargs["port"],
                                      baudrate=kwargs.get("baudrate", 115200),
                                      timeout=kwargs.get("timeout", 1.0))

    @abstractmethod
    def release(self):
        """释放资源"""
        pass


class XenseSerialGripper(XenseGripper):
    """纯串口直连夹爪驱动，不依赖 ezros / xensesdk。"""

    def __init__(self, port, device_id=1, baudrate=115200, timeout=1.0):
        self._port = port
        self._device_id = device_id
        self._serial_master = SerialDevice(port, baudrate=baudrate, timeout=timeout)
        self._active_response_enabled = False
        self._active_response_command = None
        self._gripper_status = CircularBuffer(1)
        self._button_status = Queue(maxsize=5)
        self._calibrate_status = CircularBuffer(1)
        self._running = True
        self._receive_thread_error: str | None = None
        self._worker = Thread(target=self._run_loop, daemon=True)
        self._worker.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
        return False

    def __del__(self):
        # 仅标记停止，避免串口已关闭时产生异常
        self._running = False

    def release(self):
        self._running = False
        if self._serial_master:
            # Close the port first so any blocking ser.read() in _run_loop raises
            # an exception immediately, making the thread exit without waiting for
            # the full serial read timeout.
            self._serial_master.close()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=1.0)

    # ── receive loop ──────────────────────────────────────────────────────────

    def _run_loop(self):
        while self._running:
            try:
                data = self._serial_master.receive()
            except Exception as e:
                # Serial port was closed (e.g. during release()) — exit cleanly
                if self._running:
                    self._receive_thread_error = f"{type(e).__name__}: {e}"
                break
            if data is not None:
                if data[3] == ResponseType.GRIPPER_STATUS:
                    status = self._bytes_to_gripper_status(data)
                    if status is not None:
                        self._gripper_status.put(status)
                elif data[3] == ResponseType.BUTTON_EVENT:
                    status = self._bytes_to_button_status(data)
                    if status is not None:
                        self._button_status.put(ButtonEvent(status))
                elif data[3] == ResponseType.CALIBRATE_STATUS:
                    status = self._bytes_to_calibrate_status(data)
                    if status is not None:
                        self._calibrate_status.put(status)

    def is_receive_thread_alive(self) -> bool:
        """Return True while the background serial receive thread is alive."""
        return self._worker is not None and self._worker.is_alive()

    def get_receive_thread_failure_reason(self) -> str | None:
        """Return a failure reason only when the receive thread is unexpectedly unhealthy."""
        if not self._running:
            return None
        if self._receive_thread_error is not None:
            return self._receive_thread_error
        if not self.is_receive_thread_alive():
            return "receive thread exited unexpectedly"
        return None

    def _bytes_to_gripper_status(self, res_bytes):
        if len(res_bytes) == 13:
            temperature = res_bytes[4]
            force    = struct.unpack('<h', res_bytes[5:7])[0] * 0.01
            vel      = struct.unpack('<h', res_bytes[7:9])[0] * 0.1
            raw_pos  = struct.unpack('<h', res_bytes[9:11])[0] * 0.01
            position = max(0.0, min(85.0, raw_pos))
            return {
                'position':    85.0 - position,
                'velocity':    vel,
                'force':       force,
                'temperature': temperature,
            }
        return None

    def _bytes_to_button_status(self, res_bytes):
        if len(res_bytes) == 13:
            return res_bytes[5]
        return None

    def _bytes_to_calibrate_status(self, res_bytes):
        if len(res_bytes) == 13:
            status = 'done' if res_bytes[4] == 0x20 else 'run'
            return {'status': status}
        return None

    # ── motion control ────────────────────────────────────────────────────────

    def set_position(self, position, vmax=80.0, fmax=27.0):
        """
        设置夹爪目标位置。

        Args:
            position: 目标位置 (mm)，范围 [0, 85]。0 = 完全闭合，85 = 完全张开。
            vmax:     最大速度 (mm/s)，范围 [0, 350]，默认 80。
            fmax:     最大力 (N)，范围 [0, 60]，默认 27。
        """
        if not 0 <= vmax <= 350:
            raise ValueError(f"vmax {vmax} out of range [0, 350] mm/s")
        if not 0 <= fmax <= 60:
            raise ValueError(f"fmax {fmax} out of range [0, 60] N")
        if not 0 <= position <= 85:
            raise ValueError(f"position {position} out of range [0, 85] mm")
        # The MCU register uses the opposite direction of the public API:
        # raw 0 = fully open, raw 85 = fully closed.
        raw_pos = 85 - position
        vmax    /= 0.1
        fmax    /= 0.01
        raw_pos /= 0.01
        data_bytes = (Command.POSITION_CONTROL.to_bytes(1, 'little')
                      + (0).to_bytes(1, 'little')
                      + int(fmax).to_bytes(2, 'little')
                      + int(vmax).to_bytes(2, 'little')
                      + int(raw_pos).to_bytes(2, 'little'))
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))

    def open_gripper(self, vmax=80.0, fmax=27.0):
        """完全张开（用户坐标 85 mm）。"""
        self.set_position(85.0, vmax=vmax, fmax=fmax)

    def close_gripper(self, vmax=80.0, fmax=27.0):
        """完全闭合（用户坐标 0 mm）。"""
        self.set_position(0.0, vmax=vmax, fmax=fmax)

    def set_speed(self, vmax, fmax=27):
        """
        速度闭环控制。

        Args:
            vmax: 目标速度 (mm/s)，范围 [-440, 440]。
            fmax: 最大推力 (N)，范围 [0, 60]，默认 27。
        """
        if not -440 <= vmax <= 440:
            raise ValueError(f"vmax {vmax} out of range [-440, 440] mm/s")
        if not 0 <= fmax <= 60:
            raise ValueError(f"fmax {fmax} out of range [0, 60] N")
        fmax /= 0.01
        vmax /= 0.1
        data_bytes = (Command.SPEED_CONTROL.to_bytes(1, 'little')
                      + (0).to_bytes(1, 'little')
                      + int(fmax).to_bytes(2, 'little', signed=False)
                      + int(vmax).to_bytes(2, 'little', signed=True)
                      + (0).to_bytes(2, 'little'))
        assert len(data_bytes) == 8
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))

    def stop(self):
        """立即停止电机。"""
        data_bytes = Command.STOP_MOTOR.to_bytes(1, 'little') + (0).to_bytes(1, 'little') * 7
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))
        return 0

    def set_position_sync(self, position, vmax, fmax, tolerance=0.01, timeout=5.0, poll_interval=0.05):
        """
        发送位置指令并阻塞直到到达目标位置或超时。

        Args:
            position:      目标位置 (mm)，[0, 85]。
            vmax:          最大速度 (mm/s)。
            fmax:          最大力 (N)。
            tolerance:     位置容差 (mm)，默认 0.01。
            timeout:       超时时间 (s)，默认 5.0。
            poll_interval: 轮询间隔 (s)，默认 0.05。
        """
        # raw_position = 85 - position
        self.set_position(position, vmax, fmax)
        start_time = time.time()
        while True:
            status = self.get_gripper_status(timeout=poll_interval)
            if status is not None and abs(status["position"] - position) <= tolerance:
                break
            if (time.time() - start_time) > timeout:
                current = status["position"] if status else float("nan")
                raise TimeoutError(
                    f"Gripper did not reach {position} mm within {timeout} s. "
                    f"Last position: {current:.3f} mm"
                )

    # ── LED ──────────────────────────────────────────────────────────────────

    def set_led_color(self, R, G, B):
        """设置 WS2812B 灯颜色（R/G/B 各 0~255）。"""
        if not (0 <= R <= 255 and 0 <= G <= 255 and 0 <= B <= 255):
            raise ValueError("RGB values must be in 0~255")
        data_bytes = (Command.WRITE_WS2812B.to_bytes(1, 'little')
                      + G.to_bytes(1, 'little')
                      + R.to_bytes(1, 'little')
                      + B.to_bytes(1, 'little')
                      + (0).to_bytes(1, 'little') * 4)
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))

    # ── status queries ────────────────────────────────────────────────────────

    def get_gripper_status(self, timeout=None):
        """
        查询夹爪状态（位置/速度/力/温度）。

        Args:
            timeout: 若设置，等待最多 timeout 秒收到回包；否则立即返回缓存值。

        Returns:
            dict | None: {'position', 'velocity', 'force', 'temperature'} 或 None。
        """
        self._gripper_status.clear()  # flush stale cached data before fresh request
        data_bytes = Command.GRIPPER_STATUS.to_bytes(1, 'little') + (0).to_bytes(1, 'little') * 7
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))
        if timeout is None:
            return self._gripper_status.get()
        deadline = time.time() + timeout
        poll = min(0.05, timeout / 10) if timeout > 0 else 0.05
        while time.time() < deadline:
            status = self._gripper_status.get()
            if status is not None:
                return status
            time.sleep(poll)
        return self._gripper_status.get()

    def get_button_status(self):
        try:
            return self._button_status.get(block=False)
        except Empty:
            return ButtonEvent.NONE

    def get_calibrate_status(self):
        return self._calibrate_status.get()

    # ── calibration ───────────────────────────────────────────────────────────

    def calibrate(self):
        """触发夹爪标定。"""
        data_bytes = Command.CALIBRATE.to_bytes(1, 'little') + (0).to_bytes(1, 'little') * 7
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))
        return 0

    def calibrate_encoder(self):
        """触发编码器标定。"""
        data_bytes = Command.CALIBRATE_ENCODER.to_bytes(1, 'little') + (0).to_bytes(1, 'little') * 7
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))
        return 0

    @deprecated("Buggy, will remove in future")
    def set_active_response(self, enable=True, period=10):
        # BUG: Do not use, causes reading delay
        self._active_response_enabled = enable
        self._active_response_command = Command.GRIPPER_STATUS
        data_bytes = (Command.ACTIVE_RESPONSE.to_bytes(1, 'little')
                      + Command.GRIPPER_STATUS.to_bytes(1, 'little')
                      + int(enable).to_bytes(1, 'little')
                      + int(period).to_bytes(2, 'little')
                      + (0).to_bytes(1, 'little') * 3)
        self._serial_master.send(self._serial_master.build_packet(self._device_id, data_bytes))


class GripperClientNode(Node):

    def __init__(self, mac_addr):
        if not _EZROS_AVAILABLE:
            raise ImportError(
                "xensesdk (ezros) is required for XenseTCPGripper. "
                "For serial-only usage call XenseGripper.create(port='/dev/ttyUSB0') instead."
            )
        assert isinstance(mac_addr, str), "Device MAC must be a string"
        super().__init__(name=f"gripper_{mac_addr}*")
        self._mac_addr = mac_addr
        self.buffer = deque([None], maxlen=2)

        # 夹爪自启动
        if not self.service_is_ready(f"gripper_{mac_addr}"):
            self.logger.info("Gripper not started, starting now...")
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
        注册按钮事件回调。

        Args:
            event_type: ButtonEvent 枚举值。
            callback:   无参回调函数。
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
        self.buffer.append(data)

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
        self.gripper_client.set_gripper_position(position, vmax, fmax)

    def set_speed(self, velocity, fmax=27):
        self.gripper_client.set_gripper_speed(velocity, fmax)


class SafeCtl:
    def __init__(self, gripper_client: GripperClientNode, serial_number=None):
        from xensesdk import Sensor
        import sys
        if serial_number is None:
            MASTER_SERVICE = f"master_{gripper_client._mac_addr}"
            ret = gripper_client.call_service(MASTER_SERVICE, "scan_sensor_sn")
            if ret is None:
                print("Failed to scan sensors")
                sys.exit(1)
            else:
                print(f"Found sensors: {ret}, using the first one.")
            serial_number = list(ret.keys())[1]
        else:
            print(f"Using provided serial number: {serial_number}")

        self.gripper_client = gripper_client
        sensor = Sensor.create(serial_number, mac_addr=gripper_client._mac_addr)
        self._controller: SafeCtlImpl = SafeCtlImpl(sensor, self.gripper_client)
        self._controller.start()

    def set_position(self, position, vmax=80.0, fmax=27.0):
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
    """通过 ezros 网络协议控制 FlareGrip 夹爪（需安装 xensesdk）。"""

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
            "Valid types: CLICK, DOUBLE_CLICK, LONG_PRESS, PRESS, RELEASE"
        self.gripper_client.register_button_callback(ButtonEvent[event_type], callback)

    def release(self):
        self.gripper_client.shutdown()

    @contextmanager
    def mode(self, mode: Union[ControlMode, str], serial_number=None):
        """上下文管理器：切换控制模式，退出时自动恢复。"""
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
        设置夹爪目标位置（通过 ezros）。

        Args:
            position: 目标位置 (mm)，[0, 85]。85 = 完全张开，0 = 完全闭合。
            vmax:     最大速度 (mm/s)，[0, 350]。
            fmax:     最大力 (N)，[0, 60]。
        """
        self._ctl.set_position(position, vmax, fmax)

    @require_mode(ControlMode.SPEED)
    def set_speed(self, velocity, fmax=27):
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

    def set_position_sync(self, position, vmax, fmax, tolerance=0.01, timeout=5.0, poll_interval=0.05):
        start_time = time.time()
        while True:
            self._ctl.set_position(position, vmax, fmax)
            data_rec = self.gripper_client.buffer[-1]
            if data_rec is None:
                continue
            current_pos = data_rec["position"]
            if abs(current_pos - position) <= tolerance:
                break
            if (time.time() - start_time) > timeout:
                raise TimeoutError(
                    f"Gripper did not reach {position} mm within {timeout} s. "
                    f"Last position: {current_pos:.3f} mm"
                )
            time.sleep(poll_interval)
        return True

    def get_gripper_status(self):
        full_data = self.gripper_client.buffer[-1]
        if full_data is not None:
            return {k: v for k, v in full_data.items() if k != 'button'}
        return None

    def get_button_status(self):
        full_data = self.gripper_client.buffer[-1]
        if full_data is not None:
            return full_data["button"]
        return None

    def open_gripper(self, vmax=40.0, fmax=27.0):
        self._ctl.set_position(85, vmax=vmax, fmax=fmax)

    def close_gripper(self, vmax=40.0, fmax=27.0):
        self._ctl.set_position(0, vmax=vmax, fmax=fmax)

    def set_led_color(self, r, g, b):
        self.gripper_client.set_led_color(r, g, b)

    def calibrate(self):
        self.gripper_client.calibrate()


def read_board_sn(port: str, baudrate: int = 115200, device_id: int = 1, retries: int = 3) -> str | None:
    """Read the MCU serial number from a gripper over serial port.

    Sends command 0xD0 (TYPE=0x03) and reads the 6-byte ASCII SN from the response.

    Args:
        port:      Serial port path, e.g. "/dev/ttyUSB0"
        baudrate:  Baud rate (default 115200)
        device_id: Target device ID (default 1)
        retries:   Number of attempts before giving up (default 3)

    Returns:
        SN string (e.g. "XG0042") or None if no response.
    """
    import time
    dev = SerialDevice(port, baudrate=baudrate, timeout=0.5)
    try:
        time.sleep(0.05)
        packet = dev.build_packet(device_id, bytes([Command.READ_BOARD_SN, 0x03, 0, 0, 0, 0, 0, 0]))
        for attempt in range(retries):
            dev.clear_rx_buf()
            dev.send(packet)
            dev.ser.flush()
            time.sleep(0.08)
            resp = dev.receive(13)
            if resp and len(resp) == 13 and resp[3] == Command.READ_BOARD_SN and resp[4] == 0x03:
                return resp[5:11].decode("ascii", errors="replace").rstrip("\x00") or None
            if attempt < retries - 1:
                time.sleep(0.1)
    finally:
        dev.close()
    return None
