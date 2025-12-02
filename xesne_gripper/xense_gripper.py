#!/usr/bin/env python

# Copyright 2025 The Xense Robotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import logging
from typing import Union
import time

from xensesdk import call_service, Sensor
from xensegripper import XenseGripper, XenseCamera

from vive_tracker import ViveTracker


class FlareGrip:

    def __init__(
        self,
        mac_addr: str,
        cam_size=(640, 480),
        log_level=logging.INFO,
        no_gripper=False,
        no_sensor=False,
        no_vive=False,
        no_cam=False,
    ):
        self.mac_addr = mac_addr
        self.logger = logging.getLogger(f"Flare-{self.mac_addr[:6]}")
        self.logger.setLevel(log_level)

        # camera
        self._fg_cam: XenseCamera = None
        # gripper
        self._fg_gripper: XenseGripper = None
        # sensors
        self._fg_sensors: dict[str, Sensor] = {}
        # vive tracker
        self._fg_vive: ViveTracker | None = None
        self._vive_tracker_config = None
        self._vive_tracker_lh = None
        self._vive_tracker_args = None

        self.logger.info("Initializing FlareGrip...")

        # find all sensors
        if not no_sensor:
            sensor_sns = call_service(f"master_{self.mac_addr}", "scan_sensor_sn")
            if sensor_sns is None:
                self.logger.warning("Failed to find sensors")
            else:
                for sn in sensor_sns.keys():
                    self._fg_sensors[sn] = Sensor.create(sn, mac_addr=self.mac_addr)

        # init camera
        if not no_cam:
            camera_id = call_service(f"master_{self.mac_addr}", "list_camera")
            if camera_id is None:
                self.logger.warning("Failed to find camera")
            else:
                self._fg_cam = XenseCamera(
                    next(iter(camera_id.values())),
                    mac_addr=self.mac_addr,
                    frame_size=cam_size,
                )

        # init gripper
        if not no_gripper:
            self._fg_gripper = XenseGripper.create(self.mac_addr)

        # init vive tracker
        if not no_vive:
            self._fg_vive = ViveTracker(
                config_path=self._vive_tracker_config,
                lh_config=self._vive_tracker_lh,
                args=self._vive_tracker_args,
            )
            self.logger.info("Waiting for Vive Tracker to be detected...")
            if not self._fg_vive.connect():
                self.logger.warning("Failed to connect to Vive Tracker")
                self._fg_vive = None
            else:
                # Give more time for device detection

                time.sleep(1.0)
                # Filter out lighthouses (LH0, LH1, etc.) and only keep trackers
                all_devices = self._fg_vive.get_devices()
                trackers = [d for d in all_devices if not d.startswith("LH")]
                self.logger.info(f"Detected devices: {all_devices}, Trackers: {trackers}")

                if len(trackers) == 0:
                    self.logger.warning(
                        f"No Vive Tracker found yet (detected devices: {all_devices}), "
                        "will try to detect during runtime"
                    )

    def sensor(self, id: Union[int, str]) -> Sensor | None:
        if isinstance(id, int):
            if id > len(self._fg_sensors) - 1:
                self.logger.error(f"Sensor id {id} out of range")
                return None
            id = list(self._fg_sensors.keys())[id]

        if id not in self._fg_sensors:
            self.logger.error(
                f"Sensor {id} not found, available sensors: {list(self._fg_sensors.keys())}"
            )
            return None

        return self._fg_sensors[id]

    # region: gripper methods
    def register_button_callback(self, event_type: str, callback):
        if self._fg_gripper is not None:
            self._fg_gripper.register_button_callback(event_type, callback)
        else:
            self.logger.warning(
                "No gripper initialized, cannot register button callback"
            )

    def calibrate_gripper(self):
        if self._fg_gripper is not None:
            self._fg_gripper.calibrate()
        else:
            self.logger.warning("No gripper initialized, cannot calibrate")

    # endregion: gripper methods

    def set_vive_tracker_config(self, config_path=None, lh_config=None, args=None):
        """
        set Vive Tracker configuration

        Args:
            config_path (str, optional): configuration file path
            lh_config (str, optional): lighthouse configuration
            args (list, optional): other pysurvive parameters
        """
        self._vive_tracker_config = config_path
        self._vive_tracker_lh = lh_config
        self._vive_tracker_args = args

    def get_vive_tracker(self):
        """
        get Vive Tracker object

        Returns:
            ViveTracker: Vive Tracker object
        """
        if self._fg_vive is None:
            try:
                self._fg_vive = ViveTracker(
                    config_path=self._vive_tracker_config,
                    lh_config=self._vive_tracker_lh,
                    args=self._vive_tracker_args,
                )
                self._fg_vive.connect()
            except Exception as e:
                self.logger.error(f"failed to initialize Vive Tracker: {e}")
                return None

        return self._fg_vive

    def get_pose(self, device_name=None):
        """
        get pose data of Vive Tracker

        Args:
            device_name (str, optional): device name, if None, return pose data for all devices

        Returns:
            PoseData or dict: if device_name is specified, return the PoseData object for the device;
            otherwise return a dictionary containing pose data for all devices {device_name: PoseData}
        """
        tracker = self.get_vive_tracker()
        if tracker:
            return tracker.get_pose(device_name)
        else:
            self.logger.warning("No vive tracker initialized, cannot get pose data")
            return None if device_name else {}

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
            if self._fg_cam is not None:
                data["wrist_img"] = self._fg_cam.read()[1]
            # else:
            #     self.logger.warning("No camera initialized")

        # get gripper data
        if gripper and self._fg_gripper is not None:
            gripper_status = self._fg_gripper.get_gripper_status()
            data["gripper_position"] = gripper_status["position"]
            data["gripper_velocity"] = gripper_status["velocity"]
            data["gripper_force"] = gripper_status["force"]

        if ee_pose:
            if self._fg_vive is not None:
                data["ee_pose"] = self._fg_vive.get_pose()
            else:
                self.logger.warning("No vive tracker initialized")

        return data

    def close(self):
        for sensor in self._fg_sensors.values():
            try:
                sensor.release()
            except Exception as e:
                self.logger.error(f"failed to release sensor: {e}")
        if self._fg_cam is not None:
            try:
                # Try different methods to release camera
                if hasattr(self._fg_cam, 'release'):
                    self._fg_cam.release()
                elif hasattr(self._fg_cam, 'stop'):
                    self._fg_cam.stop()
                elif hasattr(self._fg_cam, 'close'):
                    self._fg_cam.close()
            except Exception as e:
                self.logger.error(f"failed to release camera: {e}")
        if self._fg_vive is not None:
            try:
                self._fg_vive.disconnect()
            except Exception as e:
                self.logger.error(f"failed to disconnect vive tracker: {e}")
        self.logger.info("XenseGripper closed.")


if __name__ == "__main__":
    FlareGrip("6ebbc5f53240")
