#!/usr/bin/env python3

import os
import time
from threading import Event
import av
from typing import Sequence


import xensesdk.ezros as ezros


class CameraServer:

    def __init__(
        self,
        serial_number: str,  # Cam-Mac-id
        frame_size: Sequence[int] = (640, 480),
        frame_rate: int = 50,
        exposure_time = 10
    ):
        """
        创建相机节点, like: Cam-000000000000-121.
        客户端连接时需首先调用 reset(frame_size, frame_rate) 来唤醒可能休眠的相机, frame_size 和 frame_rate 可选.

        Parameters:
        - serial_number : str, required, unique identifier for the camera node, format: "Cam-Mac-id", e.g., Cam-001a2b3c4d5e-121
        - frame_size : tuple, optional, default: (640, 480), resolution of the camera
        - frame_rate : int, optional, default: 50, frame rate for the camera, minimum 50 to ensure smooth streaming
        """
        ezros.init()
        self.node = ezros.Node(serial_number)
        self._logger = self.node.logger

        self.serial_number = serial_number
        self.cam_id = serial_number.split("-")[-1]
        self.size = None
        self.rate = None
        self.container = None
        self._video_stream = None
        self.exposure_time = exposure_time

        # flags
        self.running_event: Event = Event()
        self.reopen_event: Event = Event()
    
        # init publisher and monitor
        self.publisher = self.node.create_publisher(ezros.PureBytesMessage, self.serial_number, 1)
        ezros.get_global_monitor(0, check_interval=30, timeout_minutes=10)
        
        # 创建 Camera 实例
        self.reset(frame_size, frame_rate)

        # Define services
        self.service = self.node.create_service(self.serial_number, False)
        self.service.register_callback("terminate_node", self.terminate_node)
        self.service.register_callback("reset", self.reset, private=True)
        self.service.start(threaded=True)
        
    def run(self):
        try:
            while ezros.ok():
                if not self.running_event.wait(timeout=1):  # 等待唤醒
                    continue
                
                if self.reopen_event.is_set():
                    self._reopen()
                
                if self.container is None:
                    continue
                
                for packet in self.container.demux(self._video_stream):
                    self.publisher.publish(ezros.PureBytesMessage(bytes(packet)))
                    self.rate.sleep()

                    if not self.running_event.is_set() or self.reopen_event.is_set():
                        break
                
                time.sleep(0.001)  # 避免CPU占用过高
        except Exception as e:
            self._logger.error(f"Error occurred: {e}")

    def reset(self, frame_size=None, frame_rate=None) -> tuple:
        self.resume_camera()  # 若休眠, 唤醒主线程

        if frame_size is not None and self.size != frame_size and len(frame_size) == 2:
            self.size = tuple(frame_size)
            self.reopen_event.set()
        if frame_rate is not None:
            self.rate = ezros.Rate(max(frame_rate, 50))  # 最小50fps, 否则 container 缓存读取不及时, 导致延迟

        if self.container is None:
            self.reopen_event.set()

        return self.size
    
    def _reopen(self):
        if self.container is not None:
            self.container.close()

        self._logger.info(f"Opening camera {self.cam_id} with size {self.size}...")

        if os.name == "nt":
            # file = f"video={self.cam_id}"
            file = f"video=USB Camera"
            format = "dshow"
        else:
            file = f"/dev/video{self.cam_id}"
            format = "v4l2"

        self.container = av.open(
            file=file,
            format=format,
            buffer_size=1,
            options={
                "input_format": "mjpeg",
                "video_size": f"{self.size[0]}x{self.size[1]}",
                "framerate": "30",
                "autofocus": "0",
                "auto_exposure": "1",  # 1表示手动模式
                "exposure_time_absolute": f"{self.exposure_time}"  # 固定曝光时间
            }
        )
        self._video_stream = self.container.streams.video[0]
        self.reopen_event.clear()
        self._logger.info(f"Camera {self.cam_id} create success.")

    def terminate_node(self):
        try:
            self.publisher.publish(ezros.PureBytesMessage(data=b"ERROR"))
            self.container.close()
            self.container = None
            self._video_stream = None
        except Exception as e:
            self._logger.error(f"Error occurred in terminate_node: {e}")
        self._logger.info("Camera released.")
        self.node.shutdown()
        return 0
    
    def suspend_camera(self):
        self.running_event.clear()

    def resume_camera(self):
        if not self.running_event.is_set():  # 避免重复恢复
            ezros.monitor_publisher(self.serial_number, cleanup_callback=self.suspend_camera)
            self.running_event.set()

if __name__ == '__main__':

    def main():
        import argparse
        parser = argparse.ArgumentParser(description="Camera Data Publisher")
        parser.add_argument('--serial_number', type=str, default='Cam-000000000000-121', help='Serial number of the Camera')
        parser.add_argument('--frame_size', type=int, nargs=2, default=(640, 480), help='Size of the Camera feed (width height)')
        parser.add_argument('--frame_rate', type=int, default=50, help='Frame rate of the Camera feed, min 50')

        args = parser.parse_args()

        # Create node
        CameraServer(serial_number=args.serial_number, frame_size=args.frame_size, frame_rate=args.frame_rate).run()

    main()
