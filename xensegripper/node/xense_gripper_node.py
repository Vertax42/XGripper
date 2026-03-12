import os
import logging

import xensesdk.ezros as ezros
from xensesdk.ezros.ip_tool import get_mac_address
from xensegripper import XenseGripper

class XenseGripperNode:

    def __init__(
            self,
            port = "/dev/ttyS8",  # 固定串口
    ):
        self.retry_times = 0
        self._mac_addr = get_mac_address("eth0")
        ezros.init()
        self.node = ezros.Node(f"gripper_{self._mac_addr}")
        self._logger = self.node.logger

        self._logger.info("start creating serial gripper.")
        self._gripper = XenseGripper.create(port=port)  # 固定串口

        self._logger.info("start creating data publisher.")
        self._publisher = self.node.create_publisher(ezros.BytesMessage, f"gripper_{self._mac_addr}", 1)
        self._timer = self.node.create_timer(5, self.on_timeout)

        self._logger.info("start creating server.")
        self._service = self.node.create_service(f"gripper_{self._mac_addr}", start=False)
        self._service.register_callback("set_position", self.set_position) 
        self._service.register_callback("set_speed", self.set_speed) 
        self._service.register_callback("set_led_color", self.set_led_color)
        self._service.register_callback("calibrate", self._gripper.calibrate)
        # self._subsriber = self.node.create_subscriber(ezros.BytesMessage, f"gripper_control_{self._mac_addr}", self.control_gripper, 1)

        self._logger.info("finish create gripper node.")
        self._service.start(threaded=False)  # block

    def on_timeout(self):
        for _ in range(5):
            dict_data = self._gripper.get_gripper_status()
            if dict_data is None:
                continue
            else:
                break
        
        if dict_data is None:
            if self.retry_times % 100 == 0:
                self._logger.error("gripper status is None after retries...")
            self.retry_times += 1
            if self.retry_times >= 500:
                self._logger.error("gripper disconnected, stopping node.")
                self.node.shutdown()
            return
        else:
            self.retry_times = 0

        button_data = self._gripper.get_button_status()
        dict_data['button'] = button_data
        
        self._publisher.publish(ezros.BytesMessage(data=dict_data))
    
    def set_position(self, position, vmax, fmax):
        self._gripper.set_position(position, vmax, fmax)
        return 0

    def set_speed(self, v, fmax):
        self._gripper.set_speed(v, fmax)
        return 0

    def control_gripper(self, msg):
        data = msg.get_data()
        if len(data) == 2:
            # 速度控制
            self._gripper.set_speed(data[0], data[1])
        elif len(data) == 3:
            # 位置控制
            self._gripper.set_position(data[0], data[1], data[2])
        return 0
    
    def set_led_color(self, r, g, b):
        self._gripper.set_led_color(r, g, b)
        return 0

def main():
    gripper = XenseGripperNode()
    gripper._logger.info("gripper process end.")

if __name__ == '__main__':
    main()