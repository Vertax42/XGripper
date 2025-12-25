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
import threading
import queue
import numpy as np
from .pose_utils import xyzQuaternion2matrix, matrixToXYZQuaternion, quaternion_multiply

from utils.spdlogger import get_logger

# Get logger for this module
logger = get_logger("xgripper.vive_tracker")

# Import pysurvive library
try:
    import pysurvive
except ImportError:
    logger.error("pysurvive library not found, please ensure it is correctly installed")
    raise ImportError("pysurvive library not found, please ensure it is correctly installed")


# ============================================================================
# Vive Tracker to End-Effector Calibration Transform
# ============================================================================
# Calibration data: [x, y, z, qw, qx, qy, qz]
# From: vive2ee = Matrix4x4.fromVector7d(0.000, 0.021, 0.160, 0.676, -0.207, 0.207, 0.676)
VIVE2EE_POS = np.array([0.000, 0.0, 0.160])
VIVE2EE_QUAT_WXYZ = np.array([0.676, -0.207, -0.207, -0.676])  # [qw, qx, qy, qz]
# VIVE2EE_QUAT_WXYZ = np.array([0.676,−0.207,0.207,0.676])  # [qw, qx, qy, qz]
# Pre-compute the vive2ee transformation matrix
# Note: xyzQuaternion2matrix expects (x, y, z, qx, qy, qz, qw)
VIVE2EE_MATRIX = xyzQuaternion2matrix(
    VIVE2EE_POS[0], VIVE2EE_POS[1], VIVE2EE_POS[2],
    VIVE2EE_QUAT_WXYZ[1], VIVE2EE_QUAT_WXYZ[2], VIVE2EE_QUAT_WXYZ[3], VIVE2EE_QUAT_WXYZ[0]
)
# ============================================================================
# Robot End-Effector Initial Pose (for coordinate system alignment)
# ============================================================================
# Format: [x, y, z, qw, qx, qy, qz]
# From robot TCP at startup: [ 0.693307 -0.114902  0.14589   0.004567  0.003238  0.999984  0.001246]
EE_INIT_POS = np.array([0.693307, -0.114902, 0.14589])
EE_INIT_QUAT_WXYZ = np.array([0.004567, 0.003238, 0.999984, 0.001246])  # [qw, qx, qy, qz]

# Pre-compute the ee_init transformation matrix
EE_INIT_MATRIX = xyzQuaternion2matrix(
    EE_INIT_POS[0], EE_INIT_POS[1], EE_INIT_POS[2],
    EE_INIT_QUAT_WXYZ[1], EE_INIT_QUAT_WXYZ[2], EE_INIT_QUAT_WXYZ[3], EE_INIT_QUAT_WXYZ[0]
)


class PoseData:
    """Pose data structure for storing and formatting pose information."""

    def __init__(self, device_name: str, timestamp: float, position: list, rotation: list):
        """
        Initialize PoseData.
        
        Args:
            device_name: Name of the device (e.g., "WM0", "LH0")
            timestamp: Timestamp of the pose
            position: Position [x, y, z] in meters
            rotation: Quaternion [qw, qx, qy, qz] in wxyz order
        """
        self.device_name = device_name
        self.timestamp = timestamp
        self.position = position  # [x, y, z]
        self.rotation = rotation  # [qw, qx, qy, qz]

    def __str__(self):
        """Format output pose information."""
        p = self.position
        r = self.rotation
        return (f"{self.device_name}: T={self.timestamp:.6f} "
                f"Pos=[{p[0]:+8.4f}, {p[1]:+8.4f}, {p[2]:+8.4f}] "
                f"Rot=[{r[0]:+6.3f}, {r[1]:+6.3f}, {r[2]:+6.3f}, {r[3]:+6.3f}]")
    
    def to_dict(self) -> dict:
        """Convert to dictionary format."""
        return {
            "device_name": self.device_name,
            "timestamp": self.timestamp,
            "position": self.position,
            "rotation": self.rotation,
        }


class ViveTracker:
    """
    Vive Tracker device class, provide access to Vive Tracker device pose data.

    The output pose is transformed from Vive Tracker body frame to End-Effector frame.
    
    Coordinate System:
    - Output position: [x, y, z] in meters
    - Output rotation: [qw, qx, qy, qz] quaternion (wxyz order)
    
    Args:
        config_path (str, optional): Configuration file path
        lh_config (str, optional): Lighthouse configuration
        args (list, optional): Other pysurvive parameters
        apply_vive2ee (bool): Whether to apply vive-to-ee transformation (default: True)
    """

    def __init__(self, config_path=None, lh_config=None, args=None, apply_vive2ee=True):
        self.config_path = config_path
        self.lh_config = lh_config
        self.args = args if args else []
        self.apply_vive2ee = apply_vive2ee

        # Initialize state variables
        self.running = False
        self.context = None
        self.pose_queue = queue.Queue(maxsize=100)
        self.devices_info = {}
        self.data_lock = threading.Lock()
        self.latest_poses = {}

        # Thread objects
        self.collector_thread = None
        self.processor_thread = None
        self.device_monitor_thread = None
        
        # Coordinate system alignment
        # Store the first frame vive pose for alignment with robot coordinate system
        self.vive_init_pose = None  # Will store (position, rotation_wxyz)
        self.vive_init_matrix = None  # 4x4 matrix of first frame (with vive2ee applied)
        self.vive_init_inv_matrix = None  # Inverse of vive_init_matrix
        self.coordinate_initialized = False

    def connect(self) -> bool:
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

            if self.config_path:
                survive_args.extend(["--config", self.config_path])

            if self.lh_config:
                survive_args.extend(["--lh", self.lh_config])

            survive_args.extend(self.args)

            # Initialize pysurvive context
            self.context = pysurvive.SimpleContext(survive_args)
            if not self.context:
                logger.error("Cannot initialize pysurvive context")
                return False

            logger.info("Pysurvive context initialized successfully")

            # Mark as running state
            self.running = True

            # Create and start threads
            self.collector_thread = threading.Thread(target=self._pose_collector, name="PoseCollector")
            self.collector_thread.daemon = True
            self.collector_thread.start()

            self.processor_thread = threading.Thread(target=self._pose_processor, name="PoseProcessor")
            self.processor_thread.daemon = True
            self.processor_thread.start()

            self.device_monitor_thread = threading.Thread(target=self._device_monitor, name="DeviceMonitor")
            self.device_monitor_thread.daemon = True
            self.device_monitor_thread.start()

            logger.info("Vive Tracker pose tracking started")
            logger.info(f"Vive2EE transform: {'Enabled' if self.apply_vive2ee else 'Disabled'}")

            # Wait for pose collector to start receiving data
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

        # Print statistics
        logger.info("Statistics:")
        for device_name, info in self.devices_info.items():
            logger.info(f"  - {device_name}: {info['updates']} updates")

        logger.info("Vive Tracker disconnected")

    def _device_monitor(self):
        """Device monitoring thread function."""
        logger.info("Device monitoring thread started")
        self._update_device_list()

        while self.running and self.context.Running():
            self._update_device_list()
            time.sleep(1.0)

    def _update_device_list(self):
        """Update device list from pysurvive context."""
        try:
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

    def _transform_pose(self, pos: list, rot_wxyz: list) -> tuple:
        """
        Transform pose from Vive Tracker body frame to Robot End-Effector frame.
        
        Formula: action_pose = ee_init_pose @ inv(vive_init_pose) @ current_pose
        
        Where:
        - current_pose = vive_raw @ vive2ee (with vive2ee transformation)
        - vive_init_pose = first frame's current_pose
        - ee_init_pose = known robot TCP at initialization
        
        Args:
            pos: Position [x, y, z] from pysurvive
            rot_wxyz: Quaternion [qw, qx, qy, qz] from pysurvive
            
        Returns:
            tuple: (position [x, y, z], rotation [qw, qx, qy, qz]) in robot frame
        """
        # Convert pysurvive pose to 4x4 matrix
        # pysurvive quaternion is [qw, qx, qy, qz]
        # xyzQuaternion2matrix expects (x, y, z, qx, qy, qz, qw)
        vive_matrix = xyzQuaternion2matrix(
            pos[0], pos[1], pos[2],
            rot_wxyz[1], rot_wxyz[2], rot_wxyz[3], rot_wxyz[0]
        )

        # Apply vive2ee transformation: current_pose = vive @ vive2ee
        if self.apply_vive2ee:
            current_pose_matrix = vive_matrix @ VIVE2EE_MATRIX
        else:
            current_pose_matrix = vive_matrix

        # Initialize coordinate system on first frame
        if not self.coordinate_initialized:
            # Store vive_init_pose (first frame)
            self.vive_init_matrix = current_pose_matrix.copy()
            self.vive_init_pose = (pos.copy(), rot_wxyz.copy())
            
            # Compute: vive_init_inv = inv(vive_init_pose)
            self.vive_init_inv_matrix = np.linalg.inv(self.vive_init_matrix)
            
            self.coordinate_initialized = True
            logger.info(f"Coordinate system initialized!")
            logger.info(f"  Vive init pose: pos={pos}, rot={rot_wxyz}")
            logger.info(f"  EE init pose: pos={EE_INIT_POS.tolist()}, rot={EE_INIT_QUAT_WXYZ.tolist()}")

        # Compute action_pose = ee_init_pose @ inv(vive_init_pose) @ current_pose
        action_matrix = EE_INIT_MATRIX @ self.vive_init_inv_matrix @ current_pose_matrix

        # Extract position and quaternion from matrix
        # matrixToXYZQuaternion returns (x, y, z, qx, qy, qz, qw)
        x, y, z, qx, qy, qz, qw = matrixToXYZQuaternion(action_matrix)

        position = [x, y, z]
        rotation = [qw, qx, qy, qz]

        return position, rotation

    def _pose_collector(self):
        """Pose collection thread function."""
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
                device_name = str(updated.Name(), "utf-8")

                # Add new device if not seen before
                with self.data_lock:
                    if device_name not in self.devices_info:
                        logger.info(f"New device detected: {device_name}")
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0,
                        }

                # Get pose data from pysurvive
                pose_obj = updated.Pose()
                pose_data = pose_obj[0]
                timestamp = pose_obj[1]

                # Extract raw pose from pysurvive
                # pysurvive quaternion order is [qw, qx, qy, qz]
                raw_pos = [pose_data.Pos[0], pose_data.Pos[1], pose_data.Pos[2]]
                raw_rot = [pose_data.Rot[0], pose_data.Rot[1], pose_data.Rot[2], pose_data.Rot[3]]

                # Apply transformation (only for tracker devices, not lighthouses)
                if self.is_tracker_device(device_name):
                    position, rotation = self._transform_pose(raw_pos, raw_rot)
                else:
                    # For lighthouses, use raw pose (convert to wxyz format)
                    position = raw_pos
                    rotation = raw_rot

                # Create pose data object
                pose = PoseData(device_name, timestamp, position, rotation)

                # Update device information
                with self.data_lock:
                    if device_name in self.devices_info:
                        self.devices_info[device_name]["updates"] += 1
                        self.devices_info[device_name]["last_update"] = time.time()

                # Put pose data into queue
                try:
                    self.pose_queue.put_nowait(pose)
                except queue.Full:
                    try:
                        self.pose_queue.get_nowait()
                        self.pose_queue.put_nowait(pose)
                    except Exception as e:
                        logger.error(f"Cannot put pose data into queue: {e}")

    def _pose_processor(self):
        """Pose processing thread function."""
        logger.info("Pose processing thread started")

        while self.running:
            try:
                pose = self.pose_queue.get(timeout=0.1)
                with self.data_lock:
                    self.latest_poses[pose.device_name] = pose
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Cannot process pose data: {e}")

    def wait_for_devices(self, timeout: float = 10.0, required_trackers: int = 1) -> dict:
        """
        Wait for devices to be detected.
        
        Args:
            timeout: Maximum time to wait in seconds
            required_trackers: Number of trackers required (0 = don't wait for trackers)
            
        Returns:
            dict: Dictionary with 'lighthouses' and 'trackers' lists
        """
        start_time = time.time()
        logger.info(f"Waiting for devices (timeout={timeout}s, required_trackers={required_trackers})...")
        
        while time.time() - start_time < timeout:
            with self.data_lock:
                all_devices = list(self.devices_info.keys())
            
            lighthouses = [n for n in all_devices if n.startswith("LH")]
            trackers = [n for n in all_devices if self.is_tracker_device(n)]
            
            if len(trackers) >= required_trackers and len(lighthouses) >= 1:
                logger.info(f"Required devices found: {len(lighthouses)} lighthouses, {len(trackers)} trackers")
                break
                
            time.sleep(0.1)
        
        with self.data_lock:
            all_devices = list(self.devices_info.keys())
        
        lighthouses = [n for n in all_devices if n.startswith("LH")]
        trackers = [n for n in all_devices if self.is_tracker_device(n)]
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

    def get_pose(self, device_name: str = None):
        """
        Get latest pose data for specified device.
        
        The pose is transformed to robot coordinate system using:
        action_pose = ee_init_pose @ inv(vive_init_pose) @ current_pose

        Args:
            device_name: Device name, if None, return pose data for all devices

        Returns:
            PoseData or dict: If device_name is specified, return the PoseData object;
                           otherwise return a dictionary {device_name: PoseData}
        """
        if not self.running:
            logger.warn("Vive Tracker is not connected, returning empty pose data")
            return None if device_name else {}

        self._update_device_list()

        with self.data_lock:
            if device_name:
                return self.latest_poses.get(device_name)
            else:
                return self.latest_poses.copy()

    def get_devices(self) -> list:
        """Get list of all detected devices."""
        self._update_device_list()
        with self.data_lock:
            return list(self.devices_info.keys())

    def get_device_info(self, device_name: str = None) -> dict:
        """Get device information."""
        self._update_device_list()
        with self.data_lock:
            if device_name:
                return self.devices_info.get(device_name)
            else:
                return self.devices_info.copy()

    def reset_coordinate_system(self):
        """
        Reset the coordinate system alignment.
        The next pose will be used as the new init pose for alignment.
        """
        self.vive_init_pose = None
        self.vive_init_matrix = None
        self.vive_init_inv_matrix = None
        self.coordinate_initialized = False
        logger.info("Coordinate system reset. Will re-initialize on next pose.")

    def get_reference_frame_info(self) -> dict:
        """
        Get information about the reference coordinate frame.
        
        Returns:
            dict: Reference frame information
        """
        lighthouses = self.get_lighthouse_devices()
        trackers = self.get_tracker_devices()
        reference_lh = "LH0" if "LH0" in lighthouses else (lighthouses[0] if lighthouses else None)
        
        return {
            "reference_lighthouse": reference_lh,
            "lighthouses": lighthouses,
            "trackers": trackers,
            "vive2ee_enabled": self.apply_vive2ee,
            "coordinate_initialized": self.coordinate_initialized,
            "vive_init_pose": self.vive_init_pose,
            "ee_init_pose": (EE_INIT_POS.tolist(), EE_INIT_QUAT_WXYZ.tolist()),
            "coordinate_system": {
                "origin": "Robot base frame (aligned with EE_INIT_POSE)",
                "handedness": "Right-handed",
                "rotation_format": "[qw, qx, qy, qz]",
            }
        }

    def log_reference_frame_info(self):
        """Log the reference coordinate frame information."""
        info = self.get_reference_frame_info()
        
        logger.info("=" * 60)
        logger.info("Reference Coordinate System Information")
        logger.info("=" * 60)
        logger.info(f"  Lighthouses: {info['lighthouses']}")
        logger.info(f"  Trackers: {info['trackers']}")
        logger.info(f"  Vive2EE Transform: {'Enabled' if info['vive2ee_enabled'] else 'Disabled'}")
        logger.info(f"  Coordinate Initialized: {info['coordinate_initialized']}")
        if info['coordinate_initialized'] and info['vive_init_pose']:
            pos, rot = info['vive_init_pose']
            logger.info(f"  Vive Init Pose: pos={pos}, rot={rot}")
        ee_pos, ee_rot = info['ee_init_pose']
        logger.info(f"  EE Init Pose: pos={ee_pos}, rot={ee_rot}")
        logger.info(f"  Output Frame: Robot base frame")
        logger.info(f"  Rotation Format: [qw, qx, qy, qz]")
        logger.info("=" * 60)

    def __del__(self):
        """Destructor function, ensure resources are released correctly."""
        self.disconnect()
