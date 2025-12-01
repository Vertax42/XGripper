import logging
from typing import Union

from xensesdk import call_service, Sensor
from xensegripper import XenseGripper, XenseCamera

# from xenseflare.core.vive import ViveTracker
from xgripper.utils import Logger


class FlareGrip:

    def __init__(
        self,
        mac_addr: str,
        vive_sn=None,
        cam_size=(640, 480),
        log_level=logging.INFO,
        no_gripper=False,
        no_sensor=False,
        no_vive=False,
        no_cam=False,
    ):
        self.mac_addr = mac_addr
        self.logger = Logger(f"Flare-{self.mac_addr[:6]}", log_level)

        self.fg_vive: ViveTracker = None
        self.fg_cam: XenseCamera = None
        self.fg_gripper: XenseGripper = None
        self.fg_sensors: dict[str, Sensor] = {}

        self.logger.info("Initializing FlareGrip...")

        # find all sensors
        if not no_sensor:
            sensor_sns = call_service(f"master_{self.mac_addr}", "scan_sensor_sn")
            if sensor_sns is None:
                self.logger.warning("Failed to find sensors")
            else:
                for sn in sensor_sns.keys():
                    self.fg_sensors[sn] = Sensor.create(sn, mac_addr=self.mac_addr)

        # init camera
        if not no_cam:
            camera_id = call_service(f"master_{self.mac_addr}", "list_camera")
            if camera_id is None:
                self.logger.warning("Failed to find camera")
            else:
                self.fg_cam = XenseCamera(
                    next(iter(camera_id.values())),
                    mac_addr=self.mac_addr,
                    frame_size=cam_size,
                )

        # init gripper
        if not no_gripper:
            self.fg_gripper = XenseGripper.create(self.mac_addr)

        # init vive tracker
        if not no_vive:
            self._vive_sn = vive_sn  # NOTE: support specific vive tracker
            self.fg_vive = ViveTracker()
            if len(self.fg_vive.trackers) == 0:
                self.logger.warning("No Vive Tracker found")
                self.fg_vive = None
            else:
                if self._vive_sn is None or self._vive_sn not in self.fg_vive.trackers:
                    self.logger.warning(
                        f"Vive Tracker {self._vive_sn} not found, using {list(self.fg_vive.trackers.keys())[0]} instead"
                    )
                    self._vive_sn = next(iter(self.fg_vive.trackers.keys()))

    def sensor(self, id: Union[int, str]) -> Sensor | None:
        if isinstance(id, int):
            if id > len(self.fg_sensors) - 1:
                self.logger.error(f"Sensor id {id} out of range")
                return None
            id = list(self.fg_sensors.keys())[id]

        if id not in self.fg_sensors:
            self.logger.error(
                f"Sensor {id} not found, available sensors: {list(self.fg_sensors.keys())}"
            )
            return None

        return self.fg_sensors[id]

    # region: gripper methods
    def register_button_callback(self, event_type: str, callback):
        if self.fg_gripper is not None:
            self.fg_gripper.register_button_callback(event_type, callback)
        else:
            self.logger.warning(
                "No gripper initialized, cannot register button callback"
            )

    def calibrate_gripper(self):
        if self.fg_gripper is not None:
            self.fg_gripper.calibrate()
        else:
            self.logger.warning("No gripper initialized, cannot calibrate")

    # endregion: gripper methods

    def recv_data(self, ee_pose=True, gripper=True, wrist_img=True):
        data = {
            "wrist_img": None,
            "gripper_position": None,
            "gripper_velocity": None,
            "gripper_force": None,
            "ee_pose": None,
        }

        # get camera data
        if wrist_img:
            if self.fg_cam is not None:
                data["wrist_img"] = self.fg_cam.read()[1]
            # else:
            #     self.logger.warning("No camera initialized")

        # get gripper data
        if gripper and self.fg_gripper is not None:
            gripper_status = self.fg_gripper.get_gripper_status()
            data["gripper_position"] = gripper_status["position"]
            data["gripper_velocity"] = gripper_status["velocity"]
            data["gripper_force"] = gripper_status["force"]

        if ee_pose:
            if self.fg_vive is not None:
                tmp_data = self.fg_vive.recv_data()
                data["ee_pose"] = tmp_data.get(self._vive_sn, None)
            # else:
            #     self.logger.warning("No vive tracker initialized")

        return data

    def close(self):
        for sensor in self.fg_sensors.values():
            sensor.release()
        if self.fg_cam is not None:
            self.fg_cam.release()
        if self.fg_vive is not None:
            self.fg_vive.close()
        self.logger.info("FlareGrip closed.")


if __name__ == "__main__":
    FlareGrip("6ebbc5f53240")
