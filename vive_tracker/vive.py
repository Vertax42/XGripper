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

import sys
import time
import os  # noqa: F401
import signal  # noqa: F401
import math
import threading
import queue
import logging
import numpy as np
from .pose_utils import xyzQuaternion2matrix, xyzrpy2Mat, matrixToXYZQuaternion

# configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("xgripper.vive_tracker")

# import pysurvive library
try:
    import pysurvive
except ImportError:
    logger.error("pysurvive library not found, please ensure it is correctly installed")
    raise ImportError("pysurvive library not found, please ensure it is correctly installed")


class PoseData:
    """pose data structure, for storing and formatting pose information"""

    def __init__(self, device_name, timestamp, position, rotation):
        self.device_name = device_name
        self.timestamp = timestamp
        self.position = position  # [x, y, z]
        self.rotation = rotation  # [x, y, z, w] quaternion

    def __str__(self):
        """format output pose information"""
        return f"{self.device_name}: T: {self.timestamp:.6f} P: {self.position[0]:9.6f}, {self.position[1]:9.6f}, {self.position[2]:9.6f} R: {self.rotation[0]:9.6f}, {self.rotation[1]:9.6f}, {self.rotation[2]:9.6f}, {self.rotation[3]:9.6f}"  # noqa: E501


class ViveTracker:
    """
    Vive Tracker device class, provide access to Vive Tracker device pose data

    Args:
        config_path (str, optional): configuration file path
        lh_config (str, optional): lighthouse configuration
        args (list, optional): other pysurvive parameters
    """

    def __init__(self, config_path=None, lh_config=None, args=None):
        self.config_path = config_path
        self.lh_config = lh_config
        self.args = args if args else []

        # initialize state variables
        self.running = False
        self.context = None
        self.pose_queue = queue.Queue(maxsize=100)  # queue for storing latest pose
        self.devices_info = {}  # dictionary for storing device information
        self.data_lock = threading.Lock()
        self.latest_poses = {}  # dictionary for storing latest pose of each device

        # thread objects
        self.collector_thread = None
        self.processor_thread = None
        self.device_monitor_thread = None

    def connect(self):
        """
        initialize and connect to Vive Tracker device

        Returns:
            bool: whether the connection is successful
        """
        if self.running:
            logger.warning("Vive Tracker is already connected")
            return True

        try:
            logger.info("initializing pysurvive...")

            # build pysurvive parameters
            survive_args = sys.argv[:1]  # keep program name

            # add configuration file parameter
            if self.config_path:
                survive_args.extend(["--config", self.config_path])

            # add lighthouse configuration parameter
            if self.lh_config:
                survive_args.extend(["--lh", self.lh_config])

            # add other parameters
            survive_args.extend(self.args)

            # initialize pysurvive context
            self.context = pysurvive.SimpleContext(survive_args)
            if not self.context:
                logger.error("error: cannot initialize pysurvive context")
                return False

            logger.info("pysurvive context initialized successfully")

            # mark as running state
            self.running = True

            # create and start pose collection thread
            self.collector_thread = threading.Thread(target=self._pose_collector)
            self.collector_thread.daemon = True
            self.collector_thread.start()

            # create and start pose processing thread
            self.processor_thread = threading.Thread(target=self._pose_processor)
            self.processor_thread.daemon = True
            self.processor_thread.start()

            # create and start device monitoring thread
            self.device_monitor_thread = threading.Thread(target=self._device_monitor)
            self.device_monitor_thread.daemon = True
            self.device_monitor_thread.start()

            logger.info("Vive Tracker pose tracking started")

            # wait for initial data
            time.sleep(0.5)
            return True

        except Exception as e:
            logger.error(f"error: cannot connect to Vive Tracker: {e}")
            self.running = False
            return False

    def disconnect(self):
        """
        disconnect from Vive Tracker device
        """
        if not self.running:
            return

        logger.info("stopping Vive Tracker pose tracking...")
        self.running = False

        # wait for threads to finish
        if self.collector_thread:
            self.collector_thread.join(timeout=2.0)

        if self.processor_thread:
            self.processor_thread.join(timeout=2.0)

        if self.device_monitor_thread:
            self.device_monitor_thread.join(timeout=2.0)

        # clean up resources
        self.context = None
        self.pose_queue = queue.Queue(maxsize=100)

        # print statistics information
        logger.info("statistics information:")    
        for device_name, info in self.devices_info.items():
            logger.info(f"  - {device_name}: updates {info['updates']}")

        logger.info("Vive Tracker disconnected")

    def _device_monitor(self):
        """
        device monitoring thread function
        periodically check for new devices and update device list
        """
        logger.info("device monitoring thread started")

        # initialize device list
        self._update_device_list()

        # periodically check for new devices
        while self.running and self.context.Running():
            # update device list
            self._update_device_list()

            # check every second
            time.sleep(1.0)

    def _update_device_list(self):
        """
        update device list
        """
        try:
            # get all devices
            devices = list(self.context.Objects())

            # update device information dictionary
            with self.data_lock:
                for device in devices:
                    device_name = str(device.Name(), "utf-8")
                    if device_name not in self.devices_info:
                        logger.info(f"new device detected: {device_name}")
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0,
                        }
        except Exception as e:
            logger.error(f"error: cannot update device list: {e}")

    def _pose_collector(self):
        """
        pose collection thread function
        continuously get latest pose data from pysurvive and put it into queue
        """
        logger.info("pose collection thread started")

        # get and print all available devices
        devices = list(self.context.Objects())
        if not devices:
            logger.warning("warning: no devices detected")
        else:
            logger.info(f"detected {len(devices)} devices:")
            for device in devices:
                device_name = str(device.Name(), "utf-8")
                logger.info(f"  - {device_name}")
                self.devices_info[device_name] = {"updates": 0, "last_update": 0}

        # continuously get latest pose
        while self.running and self.context.Running():
            updated = self.context.NextUpdated()
            if updated:
                # get device name
                device_name = str(updated.Name(), "utf-8")

                # if it is a new device, add to device information dictionary
                with self.data_lock:
                    if device_name not in self.devices_info:
                        logger.info(f"new device detected: {device_name}")
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0,
                        }

                # get pose data
                pose_obj = updated.Pose()
                pose_data = pose_obj[0]  # pose data
                timestamp = pose_obj[1]  # timestamp

                # convert pose data to matrix
                # note: the order of quaternions in pysurvive is [w,x,y,z], in the following operations it is converted to [x,y,z,w]
                origin_mat = xyzQuaternion2matrix(
                    pose_data.Pos[0],
                    pose_data.Pos[1],
                    pose_data.Pos[2],
                    pose_data.Rot[1],
                    pose_data.Rot[2],
                    pose_data.Rot[3],
                    pose_data.Rot[0],
                )

                # initial rotation correction: rotate around X axis -20 degrees (roll)
                initial_rotation = xyzrpy2Mat(0, 0, 0, -(20.0 / 180.0 * math.pi), 0, 0)
                # alignment rotation: rotate around X axis -90 degrees, around Y axis -90 degrees
                alignment_rotation = xyzrpy2Mat(
                    0, 0, 0, -90 / 180 * math.pi, -90 / 180 * math.pi, 0
                )
                # merge rotation matrices
                rotate_matrix = np.dot(initial_rotation, alignment_rotation)
                # apply translation transform - transform the collected pose data to the gripper center
                transform_matrix = xyzrpy2Mat(0.172, 0, -0.076, 0, 0, 0)
                # calculate final transformation matrix
                result_mat = np.matmul(
                    np.matmul(origin_mat, rotate_matrix), transform_matrix
                )
                # extract position and quaternion from result matrix
                x, y, z, qx, qy, qz, qw = matrixToXYZQuaternion(result_mat)

                # extract position and rotation information
                position = [x, y, z]
                rotation = [qx, qy, qz, qw]

                # create pose data object
                pose = PoseData(device_name, timestamp, position, rotation)

                # update device information
                with self.data_lock:
                    if device_name in self.devices_info:
                        self.devices_info[device_name]["updates"] += 1
                        self.devices_info[device_name]["last_update"] = time.time()

                # put pose data into queue, if the queue is full, discard old data
                try:
                    self.pose_queue.put_nowait(pose)
                except queue.Full:
                    try:
                        self.pose_queue.get_nowait()  # discard oldest data
                        self.pose_queue.put_nowait(pose)
                    except Exception as e:
                        logger.error(f"error: cannot put pose data into queue: {e}")

    def _pose_processor(self):
        """
        pose processing thread function
        get and process pose data from queue and update latest pose dictionary
        """
        logger.info("pose processing thread started")

        while self.running:
            try:
                # try to get pose data from queue, set timeout to periodically check running state
                pose = self.pose_queue.get(timeout=0.1)

                # update latest pose dictionary
                with self.data_lock:
                    self.latest_poses[pose.device_name] = pose

                # here you can add custom pose processing logic
                # for example: send to other applications, record to file, perform analysis, etc.

            except queue.Empty:
                # queue is empty, continue waiting
                continue
            except Exception as e:
                logger.error(f"error: cannot process pose data: {e}")

    def get_pose(self, device_name=None):
        """
        get latest pose data for specified device

        Args:
            device_name (str, optional): device name, if None, return pose data for all devices

        Returns:
            PoseData or dict: if device_name is specified, return the PoseData object for the device;
                           otherwise return a dictionary containing pose data for all devices {device_name: PoseData}
        """
        if not self.running:
            logger.warning("Vive Tracker is not connected, returning empty pose data")
            return None if device_name else {}

        # force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            if device_name:
                return self.latest_poses.get(device_name)
            else:
                return self.latest_poses.copy()

    def get_devices(self):
        """
        get list of all detected devices

        Returns:
            list: list of device names
        """
        # force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            return list(self.devices_info.keys())

    def get_device_info(self, device_name=None):
        """
        get device information

        Args:
            device_name (str, optional): device name, if None, return information for all devices

        Returns:
            dict: device information dictionary
        """
        # force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            if device_name:
                return self.devices_info.get(device_name)
            else:
                return self.devices_info.copy()

    def __del__(self):
        """
        destructor function, ensure resources are released correctly
        """
        self.disconnect()
