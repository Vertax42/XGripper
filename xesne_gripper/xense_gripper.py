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

from typing import Union
import time

import spdlog

from xensesdk import call_service, Sensor
from xensegripper import XenseGripper, XenseCamera

from vive_tracker import ViveTracker

# Module-level logger for standalone functions
_module_logger = None


def _get_module_logger():
    """Get or create module-level logger."""
    global _module_logger
    if _module_logger is None:
        _module_logger = spdlog.ConsoleLogger("xense_gripper", multithreaded=True, stdout=True, colored=True)
        _module_logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v")
        _module_logger.set_level(spdlog.LogLevel.INFO)
    return _module_logger


def scan_sensors(mac_addr: str, verbose: bool = True) -> dict | None:
    """
    Scan for available Xense sensors.
    
    Args:
        mac_addr: MAC address of the FlareGrip device
        verbose: If True, print sensor information
        
    Returns:
        dict: Dictionary of sensor serial numbers and their info, or None if failed
    """
    logger = _get_module_logger()
    
    if verbose:
        logger.info(f"Scanning for sensors on device {mac_addr}...")
    
    try:
        sensor_sns = call_service(f"master_{mac_addr}", "scan_sensor_sn")
        
        if sensor_sns is None:
            if verbose:
                logger.warn("No sensors found or service unavailable")
            return None
        
        if verbose:
            logger.info(f"Found {len(sensor_sns)} sensor(s):")
            for sn, info in sensor_sns.items():
                logger.info(f"  - {sn}: {info}")
        
        return sensor_sns
        
    except Exception as e:
        if verbose:
            logger.error(f"Error scanning sensors: {e}")
        return None


def get_system_info(mac_addr: str, verbose: bool = True) -> dict | None:
    """
    Get system information including all available nodes and services.
    Similar to `ezros -a` command.
    
    Args:
        mac_addr: MAC address of the FlareGrip device
        verbose: If True, print detailed system information
        
    Returns:
        dict: System information containing cameras, sensors, gripper status
              or None if master node is not reachable
    """
    logger = _get_module_logger()
    
    if verbose:
        logger.info("=" * 60)
        logger.info("Scanning XGripper System...")
        logger.info("=" * 60)
    
    system_info = {
        "mac_addr": mac_addr,
        "master_node": f"master_{mac_addr}",
        "cameras": {},
        "sensors": {},
        "gripper": None,
        "connected": False,
    }
    
    try:
        # Get system info from master node
        info = call_service(f"master_{mac_addr}", "system_info")
        if info is not None:
            system_info["connected"] = True
            system_info["raw_info"] = info
            if verbose:
                logger.info(f"Master Node: master_{mac_addr} ✓")
        else:
            if verbose:
                logger.error(f"Master Node: master_{mac_addr} ✗ (not reachable)")
            return system_info
    except Exception as e:
        if verbose:
            logger.error(f"Failed to connect to master node: {e}")
        return system_info
    
    # Get camera info
    try:
        cameras = call_service(f"master_{mac_addr}", "list_camera")
        if cameras:
            system_info["cameras"] = cameras
            if verbose:
                logger.info(f"Cameras ({len(cameras)}):")
                for cam_name, cam_info in cameras.items():
                    logger.info(f"  - {cam_name}: {cam_info}")
        else:
            if verbose:
                logger.info("Cameras: None detected")
    except Exception as e:
        if verbose:
            logger.warn(f"Failed to list cameras: {e}")
    
    # Get sensor info
    try:
        sensors = call_service(f"master_{mac_addr}", "scan_sensor_sn")
        if sensors:
            system_info["sensors"] = sensors
            if verbose:
                logger.info(f"Sensors ({len(sensors)}):")
                for sn, info in sensors.items():
                    logger.info(f"  - SN: {sn}, Info: {info}")
        else:
            if verbose:
                logger.info("Sensors: None detected")
    except Exception as e:
        if verbose:
            logger.warn(f"Failed to scan sensors: {e}")
    
    # Check gripper availability
    try:
        # Try to check if gripper node exists
        gripper_node = f"gripper_{mac_addr}"
        system_info["gripper"] = gripper_node
        if verbose:
            logger.info(f"Gripper Node: {gripper_node}")
    except Exception as e:
        if verbose:
            logger.warn(f"Failed to check gripper: {e}")
    
    if verbose:
        logger.info("=" * 60)
        logger.info("System scan complete")
        logger.info("=" * 60)
    
    return system_info


def print_system_info(mac_addr: str):
    """
    Print system information in a formatted way.
    Convenient wrapper around get_system_info().
    
    Args:
        mac_addr: MAC address of the FlareGrip device
    """
    print()
    print("=" * 70)
    print("                    XGripper System Information")
    print("=" * 70)
    
    info = get_system_info(mac_addr, verbose=False)
    
    if not info or not info.get("connected"):
        print(f"  ❌ Cannot connect to master node: master_{mac_addr}")
        print("     Please check:")
        print("       - Device is powered on")
        print("       - Network connection is working")
        print("       - MAC address is correct")
        print("=" * 70)
        return info
    
    print(f"  MAC Address: {mac_addr}")
    print(f"  Master Node: master_{mac_addr} ✅")
    print()
    
    # Cameras
    cameras = info.get("cameras", {})
    print(f"  📷 Cameras ({len(cameras)}):")
    if cameras:
        for cam_name, cam_info in cameras.items():
            print(f"      - {cam_name}: {cam_info}")
    else:
        print("      (none)")
    print()
    
    # Sensors
    sensors = info.get("sensors", {})
    print(f"  📡 Sensors ({len(sensors)}):")
    if sensors:
        for sn, sensor_info in sensors.items():
            print(f"      - {sn}: {sensor_info}")
    else:
        print("      (none)")
    print()
    
    # Gripper
    gripper = info.get("gripper")
    print(f"  🤖 Gripper:")
    if gripper:
        print(f"      - {gripper}")
    else:
        print("      (not available)")
    
    print("=" * 70)
    print()
    
    return info


class FlareGrip:

    def __init__(
        self,
        mac_addr: str,
        cam_size=(640, 480),
        rectify_size=(400, 700),
        log_level: str = "INFO",
        no_gripper=False,
        no_sensor=False,
        no_vive=False,
        no_cam=False,
    ):
        self.mac_addr = mac_addr
        self.rectify_size = rectify_size

        # Create spdlog logger for this instance
        logger_name = f"Flare-{self.mac_addr[:6]}"
        self.logger = spdlog.ConsoleLogger(logger_name, multithreaded=True, stdout=True, colored=True)
        self.logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v")

        # Set log level
        level_map = {
            "DEBUG": spdlog.LogLevel.DEBUG,
            "INFO": spdlog.LogLevel.INFO,
            "WARN": spdlog.LogLevel.WARN,
            "WARNING": spdlog.LogLevel.WARN,
            "ERROR": spdlog.LogLevel.ERR,
            "CRITICAL": spdlog.LogLevel.CRITICAL,
        }
        if isinstance(log_level, str):
            self.logger.set_level(level_map.get(log_level.upper(), spdlog.LogLevel.INFO))
        else:
            self.logger.set_level(spdlog.LogLevel.INFO)

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

        # Scan and print available sensors first
        self.logger.info("=" * 50)
        self.logger.info("Scanning available sensors...")
        available_sensors = scan_sensors(self.mac_addr, verbose=False)
        if available_sensors:
            self.logger.info(f"Available sensors ({len(available_sensors)}):")
            for sn, info in available_sensors.items():
                self.logger.info(f"  - SN: {sn}, Info: {info}")
        else:
            self.logger.warn("No sensors detected on this device")
        self.logger.info("=" * 50)

        # Initialize sensors
        if not no_sensor:
            if available_sensors:
                self.logger.info(f"Initializing {len(available_sensors)} sensor(s)...")
                for sn in available_sensors.keys():
                    self.logger.info(f"  Initializing sensor: {sn}")
                    try:
                        self._fg_sensors[sn] = Sensor.create(
                            sn, mac_addr=self.mac_addr, rectify_size=self.rectify_size
                        )
                        self.logger.info(f"  Sensor {sn} initialized successfully")
                    except Exception as e:
                        self.logger.error(f"  Failed to initialize sensor {sn}: {e}")
            else:
                self.logger.warn("No sensors to initialize")

        # init camera
        if not no_cam:
            camera_id = call_service(f"master_{self.mac_addr}", "list_camera")
            if camera_id is None:
                self.logger.warn("Failed to find camera")
            else:
                self._fg_cam = XenseCamera(
                    next(iter(camera_id.values())),
                    mac_addr=self.mac_addr,
                    frame_size=cam_size,
                )

        # init gripper
        if not no_gripper:
            self._fg_gripper = XenseGripper.create(self.mac_addr)
            if self._fg_gripper is not None:
                self.logger.info("Gripper initialized successfully")
            else:
                self.logger.warn("Failed to initialize gripper (XenseGripper.create returned None)")

        # init vive tracker
        if not no_vive:
            self._fg_vive = ViveTracker(
                config_path=self._vive_tracker_config,
                lh_config=self._vive_tracker_lh,
                args=self._vive_tracker_args,
            )
            self.logger.info("Waiting for Vive Tracker to be detected...")
            if not self._fg_vive.connect():
                self.logger.warn("Failed to connect to Vive Tracker")
                self._fg_vive = None
            else:
                # Use wait_for_devices() method which monitors devices_info
                # populated by _pose_collector thread via NextUpdated()
                self.logger.info("Waiting for Vive devices (this may take a few seconds)...")
                devices = self._fg_vive.wait_for_devices(timeout=10.0, required_trackers=1)
                
                lighthouses = devices["lighthouses"]
                trackers = devices["trackers"]
                
                self.logger.info(f"Detected: lighthouses={lighthouses}, trackers={trackers}")

                if len(trackers) == 0:
                    self.logger.warn(
                        f"No Vive Tracker found (lighthouses: {lighthouses}). "
                        "Check if tracker is paired with dongle and in lighthouse view."
                    )
                else:
                    self.logger.info(f"Found {len(trackers)} tracker(s): {trackers}")
                    # Log reference coordinate system information
                    self._fg_vive.log_reference_frame_info()

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
            self.logger.warn(
                "No gripper initialized, cannot register button callback"
            )

    def calibrate_gripper(self):
        if self._fg_gripper is not None:
            self._fg_gripper.calibrate()
        else:
            self.logger.warn("No gripper initialized, cannot calibrate")

    # endregion: gripper methods

    def set_vive_tracker_config(self, config_path=None, lh_config=None, args=None):
        """
        Set Vive Tracker configuration.

        Args:
            config_path (str, optional): Configuration file path
            lh_config (str, optional): Lighthouse configuration
            args (list, optional): Other pysurvive parameters
        """
        self._vive_tracker_config = config_path
        self._vive_tracker_lh = lh_config
        self._vive_tracker_args = args

    def get_vive_tracker(self):
        """
        Get Vive Tracker object.

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
                self.logger.error(f"Failed to initialize Vive Tracker: {e}")
                return None

        return self._fg_vive

    def get_pose(self, device_name=None):
        """
        Get pose data of Vive Tracker.

        Args:
            device_name (str, optional): Device name, if None, return pose data for all devices

        Returns:
            PoseData or dict: If device_name is specified, return the PoseData object for the device;
            otherwise return a dictionary containing pose data for all devices {device_name: PoseData}
        """
        tracker = self.get_vive_tracker()
        if tracker:
            return tracker.get_pose(device_name)
        else:
            self.logger.warn("No vive tracker initialized, cannot get pose data")
            return None if device_name else {}

    def recv_data(self, ee_pose=True, gripper=True, wrist_img=True, sensor=True):
        """
        Receive all data from FlareGrip components.
        
        Args:
            ee_pose: Whether to get end-effector pose (Vive tracker) data
            gripper: Whether to get gripper status data
            wrist_img: Whether to get wrist camera image
            sensor: Whether to get sensor rectify data
            
        Returns:
            dict: Dictionary containing all requested data:
                - wrist_img: Camera image (np.ndarray or None)
                - gripper_position: Gripper position (float or None)
                - gripper_velocity: Gripper velocity (float or None)
                - gripper_force: Gripper force (float or None)
                - ee_pose: End-effector pose (dict or None)
                - sensor_rectify: Sensor rectify images (dict[str, np.ndarray] or None)
        """
        data = {
            "wrist_img": None,
            "gripper_position": None,
            "gripper_velocity": None,
            "gripper_force": None,
            "ee_pose": None,
            "sensor_rectify": None,
        }

        # get camera data
        if wrist_img:
            if self._fg_cam is not None:
                data["wrist_img"] = self._fg_cam.read()[1]

        # get gripper data
        if gripper and self._fg_gripper is not None:
            try:
                gripper_status = self._fg_gripper.get_gripper_status()
                if gripper_status is not None:
                    data["gripper_position"] = gripper_status.get("position")
                    data["gripper_velocity"] = gripper_status.get("velocity")
                    data["gripper_force"] = gripper_status.get("force")
                else:
                    self.logger.debug("Gripper status returned None")
            except Exception as e:
                self.logger.warn(f"Failed to get gripper status: {e}")

        # get end-effector pose data
        if ee_pose:
            data["ee_pose"] = self.get_pose()

        # get sensor rectify data
        if sensor and self._fg_sensors:
            sensor_rectify = {}
            for sn, sensor_obj in self._fg_sensors.items():
                try:
                    rectify = sensor_obj.selectSensorInfo(Sensor.OutputType.Rectify)
                    if rectify is not None:
                        sensor_rectify[sn] = rectify
                except Exception as e:
                    self.logger.debug(f"Failed to read sensor {sn} rectify data: {e}")
            
            if sensor_rectify:
                data["sensor_rectify"] = sensor_rectify

        return data

    def close(self):
        for sensor in self._fg_sensors.values():
            try:
                sensor.release()
            except Exception as e:
                self.logger.error(f"Failed to release sensor: {e}")
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
                self.logger.error(f"Failed to release camera: {e}")
        if self._fg_vive is not None:
            try:
                self._fg_vive.disconnect()
            except Exception as e:
                self.logger.error(f"Failed to disconnect vive tracker: {e}")
        self.logger.info("XenseGripper closed.")


if __name__ == "__main__":
    FlareGrip("6ebbc5f53240")
