import time
from threading import Event

import cv2
import numpy as np

import xensesdk.ezros as ezros
from collections import deque
from xensesdk.utils.math_utils import Filter


class CameraNode(ezros.Node):
    
    def __init__(self, cam_id, mac_addr: str, frame_size=(640, 480), frame_rate=50, log_level="WARNING"):
        super().__init__(name=f"Cam-{mac_addr.lower()}-{cam_id}x", log_level=log_level)

        if ezros.get_topic_type(self.name[:-1], timeout=0.5) is None:
            self.logger.warning(f"{self.name[:-1]} not started, starting now...")
            ret = self.call_service(f"master_{mac_addr}", "launch_camera", cam_id, frame_size, frame_rate)
            if ret is None:
                raise Exception(f"Failed to launch camera {self.name[:-1]}")

        self.client = None
        self.subscriber = None
        self.frame_size = frame_size
        self.frame_rate = frame_rate
        self.new_data_event = Event()
        self.buffer = deque([None], maxlen=2)
        self._fps = Filter(30, alpha=0.2)
        self.last_frame_time = time.time()
        self._connect()

    def _connect(self):
        self.client = self.create_client(self.name[:-1], timeout=10, interval=0.1, quiet=False)
        if self.client is None:
            raise Exception(f"Failed to connect to camera {self.name[:-1]}")
        self.client.reset(self.frame_size, self.frame_rate)
        self.subscriber = self.create_subscriber(
            ezros.PureBytesMessage,
            self.name[:-1],
            self._on_receive,
            qos_depth=1
        )
        time.sleep(0.5)  # 等待数据稳定

    def _on_receive(self, msg):
        packet_bytes = msg.get_data()
        if len(packet_bytes) < 200:
            return
        
        try:
            np_bytes = np.frombuffer(packet_bytes, np.uint8)
            img_array = cv2.imdecode(np_bytes, cv2.IMREAD_COLOR)
            self.buffer.append((img_array, time.time()))
            self.new_data_event.set()
            
            # real fps
            self._fps.update( 1 / (time.time() - self.last_frame_time + 1e-4) )
            self.last_frame_time = time.time()
            
        except Exception as e:
            self.new_data_event.clear()
            self.logger.warning(f"Error decoding image: {e}")

    def read(self, timeout=1):
        """读取最新一帧图像，阻塞等待直到超时, 返回 (timestamp, frame)"""
        if self.new_data_event.wait(timeout):
            frame, time_stamp = self.buffer[-1]
            self.new_data_event.clear()
            return time_stamp, frame
        else:
            return None, None

    @property
    def fps(self):
        """返回当前接收帧率"""
        return self._fps.data