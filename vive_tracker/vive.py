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
import numpy as np
from .pose_utils import xyzQuaternion2matrix, xyzrpy2Mat, matrixToXYZQuaternion

from utils.spdlogger import get_logger

# Get logger for this module
logger = get_logger("xgripper.vive_tracker")

# Import pysurvive library
try:
    import pysurvive
except ImportError:
    logger.error("pysurvive library not found, please ensure it is correctly installed")
    raise ImportError("pysurvive library not found, please ensure it is correctly installed")


class PoseData:
    """Pose data structure, for storing and formatting pose information."""

    def __init__(self, device_name, timestamp, position, rotation):
        self.device_name = device_name
        self.timestamp = timestamp
        self.position = position  # [x, y, z]
        self.rotation = rotation  # [x, y, z, w] quaternion

    def __str__(self):
        """Format output pose information."""
        return f"{self.device_name}: T: {self.timestamp:.6f} P: {self.position[0]:9.6f}, {self.position[1]:9.6f}, {self.position[2]:9.6f} R: {self.rotation[0]:9.6f}, {self.rotation[1]:9.6f}, {self.rotation[2]:9.6f}, {self.rotation[3]:9.6f}"  # noqa: E501


class ViveTracker:
    """
    Vive Tracker device class, provide access to Vive Tracker device pose data.

    Args:
        config_path (str, optional): Configuration file path
        lh_config (str, optional): Lighthouse configuration
        args (list, optional): Other pysurvive parameters
    """

    def __init__(self, config_path=None, lh_config=None, args=None):
        self.config_path = config_path
        self.lh_config = lh_config
        self.args = args if args else []

        # Initialize state variables
        self.running = False
        self.context = None
        self.pose_queue = queue.Queue(maxsize=100)  # Queue for storing latest pose
        self.devices_info = {}  # Dictionary for storing device information
        self.data_lock = threading.Lock()
        self.latest_poses = {}  # Dictionary for storing latest pose of each device

        # Thread objects
        self.collector_thread = None
        self.processor_thread = None
        self.device_monitor_thread = None

    def connect(self):
        """
        Initialize and connect to Vive Tracker device.

        Returns:
            bool: Whether the connection is successful
        """
        if self.running:
            logger.warn("Vive Tracker is already connected")
            return True

        try:
            logger.info("Initializing pysurvive...")

            # Build pysurvive parameters
            survive_args = sys.argv[:1]  # Keep program name

            # Add configuration file parameter
            if self.config_path:
                survive_args.extend(["--config", self.config_path])

            # Add lighthouse configuration parameter
            if self.lh_config:
                survive_args.extend(["--lh", self.lh_config])

            # Add other parameters
            survive_args.extend(self.args)

            # Initialize pysurvive context
            self.context = pysurvive.SimpleContext(survive_args)
            if not self.context:
                logger.error("Cannot initialize pysurvive context")
                return False

            logger.info("Pysurvive context initialized successfully")

            # Mark as running state
            self.running = True

            # Create and start pose collection thread
            self.collector_thread = threading.Thread(target=self._pose_collector)
            self.collector_thread.daemon = True
            self.collector_thread.start()

            # Create and start pose processing thread
            self.processor_thread = threading.Thread(target=self._pose_processor)
            self.processor_thread.daemon = True
            self.processor_thread.start()

            # Create and start device monitoring thread
            self.device_monitor_thread = threading.Thread(target=self._device_monitor)
            self.device_monitor_thread.daemon = True
            self.device_monitor_thread.start()

            logger.info("Vive Tracker pose tracking started")

            # Wait a short time for pose collector to start receiving data
            time.sleep(0.5)
            
            return True

        except Exception as e:
            logger.error(f"Cannot connect to Vive Tracker: {e}")
            self.running = False
            return False

    def disconnect(self):
        """Disconnect from Vive Tracker device."""
        if not self.running:
            return

        logger.info("Stopping Vive Tracker pose tracking...")
        self.running = False

        # Wait for threads to finish
        if self.collector_thread:
            self.collector_thread.join(timeout=2.0)

        if self.processor_thread:
            self.processor_thread.join(timeout=2.0)

        if self.device_monitor_thread:
            self.device_monitor_thread.join(timeout=2.0)

        # Clean up resources
        self.context = None
        self.pose_queue = queue.Queue(maxsize=100)

        # Print statistics information
        logger.info("Statistics information:")
        for device_name, info in self.devices_info.items():
            logger.info(f"  - {device_name}: updates {info['updates']}")

        logger.info("Vive Tracker disconnected")

    def _device_monitor(self):
        """
        Device monitoring thread function.
        Periodically check for new devices and update device list.
        """
        logger.info("Device monitoring thread started")

        # Initialize device list
        self._update_device_list()

        # Periodically check for new devices
        while self.running and self.context.Running():
            # Update device list
            self._update_device_list()

            # Check every second
            time.sleep(1.0)

    def _update_device_list(self):
        """
        Update device list.
        Note: Device detection is primarily done by _pose_collector via NextUpdated().
        This method is kept for compatibility and can check Objects() for any missed devices.
        """
        try:
            # Objects() can return LH devices, add them if not already in devices_info
            devices = list(self.context.Objects())
            with self.data_lock:
                for device in devices:
                    device_name = str(device.Name(), "utf-8")
                    if device_name not in self.devices_info:
                        logger.info(f"New device detected (Objects): {device_name}")
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0,
                        }
        except Exception as e:
            logger.error(f"Cannot update device list: {e}")
    
    def wait_for_devices(self, timeout: float = 10.0, required_trackers: int = 1) -> dict:
        """
        Wait for devices to be detected.
        This method monitors devices_info which is populated by the _pose_collector thread.
        
        Args:
            timeout: Maximum time to wait in seconds
            required_trackers: Number of trackers required (0 = don't wait for trackers)
            
        Returns:
            dict: Dictionary with 'lighthouses' and 'trackers' lists
        """
        start_time = time.time()
        
        logger.info(f"Waiting for devices (timeout={timeout}s, required_trackers={required_trackers})...")
        
        while time.time() - start_time < timeout:
            # Get current devices from devices_info (populated by _pose_collector thread)
            with self.data_lock:
                all_devices = list(self.devices_info.keys())
            
            lighthouses = [n for n in all_devices if n.startswith("LH")]
            trackers = [n for n in all_devices if n.startswith("WM") or n.startswith("T2")]
            
            # Check if we have enough devices
            if len(trackers) >= required_trackers and len(lighthouses) >= 1:
                logger.info(f"Required devices found: {len(lighthouses)} lighthouses, {len(trackers)} trackers")
                break
                
            time.sleep(0.1)  # Check every 100ms
        
        # Get final device list
        with self.data_lock:
            all_devices = list(self.devices_info.keys())
        
        lighthouses = [n for n in all_devices if n.startswith("LH")]
        trackers = [n for n in all_devices if n.startswith("WM") or n.startswith("T2")]
        others = [n for n in all_devices if n not in lighthouses and n not in trackers]
        
        result = {
            "lighthouses": lighthouses,
            "trackers": trackers,
            "others": others,
            "all": all_devices,
        }
        
        logger.info(f"Detection complete: {len(lighthouses)} lighthouses, {len(trackers)} trackers")
        return result
    
    def is_tracker_device(self, device_name: str) -> bool:
        """Check if a device name is a tracker (not a lighthouse)."""
        return device_name.startswith("WM") or device_name.startswith("T2") or device_name.startswith("HMD")
    
    def get_tracker_devices(self) -> list:
        """Get list of tracker device names only (excluding lighthouses)."""
        with self.data_lock:
            return [name for name in self.devices_info.keys() if self.is_tracker_device(name)]
    
    def get_lighthouse_devices(self) -> list:
        """Get list of lighthouse device names only."""
        with self.data_lock:
            return [name for name in self.devices_info.keys() if name.startswith("LH")]
    
    def get_reference_frame_info(self) -> dict:
        """
        Get information about the reference coordinate frame.
        
        In libsurvive, LH0 is used as the reference lighthouse (origin of coordinate system).
        All poses are calculated relative to LH0.
        
        Returns:
            dict: Reference frame information including:
                - reference_lighthouse: The reference lighthouse name (typically "LH0")
                - lighthouses: List of all detected lighthouses
                - trackers: List of all detected trackers
                - coordinate_system: Description of the coordinate system
        """
        lighthouses = self.get_lighthouse_devices()
        trackers = self.get_tracker_devices()
        
        # In libsurvive, LH0 is always used as reference if available
        reference_lh = "LH0" if "LH0" in lighthouses else (lighthouses[0] if lighthouses else None)
        
        return {
            "reference_lighthouse": reference_lh,
            "lighthouses": lighthouses,
            "trackers": trackers,
            "coordinate_system": {
                "origin": f"{reference_lh} position" if reference_lh else "Unknown",
                "handedness": "Right-handed",
                "note": "Axis orientation depends on LH0 physical mounting angle"
            }
        }
    
    def log_reference_frame_info(self):
        """Log the reference coordinate frame information."""
        info = self.get_reference_frame_info()
        
        logger.info("=" * 50)
        logger.info("Reference Coordinate System Information")
        logger.info("=" * 50)
        logger.info(f"  Reference Lighthouse: {info['reference_lighthouse']} (origin)")
        logger.info(f"  All Lighthouses: {info['lighthouses']}")
        logger.info(f"  All Trackers: {info['trackers']}")
        logger.info(f"  Coordinate System: Right-handed")
        logger.info(f"  Note: All poses are relative to {info['reference_lighthouse']}")
        logger.info(f"  Note: Axis orientation depends on {info['reference_lighthouse']} physical mounting")
        logger.info("=" * 50)

    def _pose_collector(self):
        """
        Pose collection thread function.
        Continuously get latest pose data from pysurvive and put it into queue.
        """
        logger.info("Pose collection thread started")

        # Get and print all available devices
        devices = list(self.context.Objects())
        if not devices:
            logger.warn("No devices detected")
        else:
            logger.info(f"Detected {len(devices)} devices:")
            for device in devices:
                device_name = str(device.Name(), "utf-8")
                logger.info(f"  - {device_name}")
                self.devices_info[device_name] = {"updates": 0, "last_update": 0}

        # Continuously get latest pose
        while self.running and self.context.Running():
            updated = self.context.NextUpdated()
            if updated:
                # Get device name
                device_name = str(updated.Name(), "utf-8")

                # If it is a new device, add to device information dictionary
                with self.data_lock:
                    if device_name not in self.devices_info:
                        logger.info(f"New device detected: {device_name}")
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0,
                        }

                # Get pose data
                pose_obj = updated.Pose()
                pose_data = pose_obj[0]  # Pose data
                timestamp = pose_obj[1]  # Timestamp

                # Convert pose data to matrix
                # Note: the order of quaternions in pysurvive is [w,x,y,z],
                # in the following operations it is converted to [x,y,z,w]
                origin_mat = xyzQuaternion2matrix(
                    pose_data.Pos[0],
                    pose_data.Pos[1],
                    pose_data.Pos[2],
                    pose_data.Rot[1],
                    pose_data.Rot[2],
                    pose_data.Rot[3],
                    pose_data.Rot[0],
                )

                # Initial rotation correction: rotate around X axis -20 degrees (roll)
                initial_rotation = xyzrpy2Mat(0, 0, 0, -(20.0 / 180.0 * math.pi), 0, 0)
                # Alignment rotation: rotate around X axis -90 degrees, around Y axis -90 degrees
                alignment_rotation = xyzrpy2Mat(
                    0, 0, 0, -90 / 180 * math.pi, -90 / 180 * math.pi, 0
                )
                # Merge rotation matrices
                rotate_matrix = np.dot(initial_rotation, alignment_rotation)
                # Apply translation transform - transform the collected pose data to the gripper center
                transform_matrix = xyzrpy2Mat(0.172, 0, -0.076, 0, 0, 0)
                # Calculate final transformation matrix
                result_mat = np.matmul(
                    np.matmul(origin_mat, rotate_matrix), transform_matrix
                )
                # Extract position and quaternion from result matrix
                x, y, z, qx, qy, qz, qw = matrixToXYZQuaternion(result_mat)

                # Extract position and rotation information
                position = [x, y, z]
                rotation = [qx, qy, qz, qw]

                # Create pose data object
                pose = PoseData(device_name, timestamp, position, rotation)

                # Update device information
                with self.data_lock:
                    if device_name in self.devices_info:
                        self.devices_info[device_name]["updates"] += 1
                        self.devices_info[device_name]["last_update"] = time.time()

                # Put pose data into queue, if the queue is full, discard old data
                try:
                    self.pose_queue.put_nowait(pose)
                except queue.Full:
                    try:
                        self.pose_queue.get_nowait()  # Discard oldest data
                        self.pose_queue.put_nowait(pose)
                    except Exception as e:
                        logger.error(f"Cannot put pose data into queue: {e}")

    def _pose_processor(self):
        """
        Pose processing thread function.
        Get and process pose data from queue and update latest pose dictionary.
        """
        logger.info("Pose processing thread started")

        while self.running:
            try:
                # Try to get pose data from queue, set timeout to periodically check running state
                pose = self.pose_queue.get(timeout=0.1)

                # Update latest pose dictionary
                with self.data_lock:
                    self.latest_poses[pose.device_name] = pose

                # Here you can add custom pose processing logic
                # For example: send to other applications, record to file, perform analysis, etc.

            except queue.Empty:
                # Queue is empty, continue waiting
                continue
            except Exception as e:
                logger.error(f"Cannot process pose data: {e}")

    def get_pose(self, device_name=None):
        """
        Get latest pose data for specified device.

        Args:
            device_name (str, optional): Device name, if None, return pose data for all devices

        Returns:
            PoseData or dict: If device_name is specified, return the PoseData object for the device;
                           otherwise return a dictionary containing pose data for all devices {device_name: PoseData}
        """
        if not self.running:
            logger.warn("Vive Tracker is not connected, returning empty pose data")
            return None if device_name else {}

        # Force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            if device_name:
                return self.latest_poses.get(device_name)
            else:
                return self.latest_poses.copy()

    def get_devices(self):
        """
        Get list of all detected devices.

        Returns:
            list: List of device names
        """
        # Force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            return list(self.devices_info.keys())

    def get_device_info(self, device_name=None):
        """
        Get device information.

        Args:
            device_name (str, optional): Device name, if None, return information for all devices

        Returns:
            dict: Device information dictionary
        """
        # Force update device list to ensure latest devices are detected
        self._update_device_list()

        with self.data_lock:
            if device_name:
                return self.devices_info.get(device_name)
            else:
                return self.devices_info.copy()

    def __del__(self):
        """Destructor function, ensure resources are released correctly."""
        self.disconnect()
