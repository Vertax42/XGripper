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

"""
Visualization module for XGripper using Rerun.

Provides real-time visualization of:
- Camera images (wrist camera)
- Gripper state (position, velocity)
- Vive Tracker poses and trajectories
"""

import os
from typing import Any
from collections import defaultdict

import numpy as np

from .spdlogger import get_logger

logger = get_logger("xgripper.visualization")

try:
    import rerun as rr
    import rerun.blueprint as rrb  # noqa: F401

    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False


# Entity path prefixes
CAMERA_PREFIX = "camera"
GRIPPER_PREFIX = "gripper"
TRACKER_PREFIX = "vive_tracker"


def check_rerun_available() -> bool:
    """Check if Rerun is available."""
    if not RERUN_AVAILABLE:
        logger.warn("Rerun is not installed. Install it with: pip install rerun-sdk")
        return False
    return True


def init_rerun(
    session_name: str = "xgripper_visualization",
    spawn: bool = True,
    memory_limit: str | None = None,
    save_path: str | None = None,
) -> bool:
    """
    Initialize the Rerun SDK for visualizing XGripper data.

    Supports two workflows:
    - Synchronous: spawn=True starts the Native Viewer for real-time visualization
    - Asynchronous: save_path saves data to .rrd file for later viewing

    Both can be used together (real-time viewing + recording to file).

    Args:
        session_name: Name of the Rerun session/recording.
        spawn: Whether to spawn the Rerun viewer automatically (synchronous workflow).
        memory_limit: Memory limit for Rerun (e.g., "10%", "2GB").
            If None, uses XGRIPPER_RERUN_MEMORY_LIMIT env var or "10%".
        save_path: Path to save the recording (.rrd file). If provided, enables
            asynchronous workflow. Can be used with or without spawn.

    Returns:
        bool: True if initialization was successful, False otherwise.

    Examples:
        # Real-time visualization only
        init_rerun(spawn=True)

        # Save to file only (no viewer, lowest resource usage)
        init_rerun(spawn=False, save_path="/tmp/recording.rrd")

        # Both: real-time + save to file
        init_rerun(spawn=True, save_path="/tmp/recording.rrd")
    """
    if not check_rerun_available():
        return False

    try:
        # Configure flush settings for better streaming performance
        batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
        os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size

        # Suppress warnings and logging to avoid TextLog clutter
        import warnings
        import logging
        
        # Suppress all warnings
        warnings.filterwarnings("ignore")
        
        # Suppress rerun internal logging  
        logging.getLogger("rerun").setLevel(logging.CRITICAL)
        
        # Save original showwarning before Rerun can override it
        original_showwarning = warnings.showwarning
        
        # Initialize Rerun
        rr.init(session_name)
        
        # Restore original showwarning (removes Rerun's warning capture)
        warnings.showwarning = original_showwarning

        # Asynchronous workflow: save to file
        if save_path:
            rr.save(save_path)
            logger.info(f"Recording will be saved to: {save_path}")

        # Synchronous workflow: spawn viewer
        if spawn:
            if memory_limit is None:
                memory_limit = os.getenv("XGRIPPER_RERUN_MEMORY_LIMIT", "10%")
            rr.spawn(memory_limit=memory_limit)

        logger.info(f"Rerun initialized with session: {session_name}")
        return True

    except Exception as e:
        logger.error(f"Failed to initialize Rerun: {e}")
        return False


def log_camera_image(
    image: np.ndarray,
    camera_name: str = "wrist",
    entity_path: str | None = None,
    color_format: str = "RGB",
) -> None:
    """
    Log a camera image to Rerun.

    Args:
        image: Image array in HWC format (Height, Width, Channels).
            Supports RGB (3 channels), RGBA (4 channels), or grayscale (1 channel).
        camera_name: Name of the camera (e.g., "wrist", "side").
        entity_path: Custom entity path. If None, uses "{CAMERA_PREFIX}/{camera_name}".
        color_format: Color format of input image. "RGB" (default) or "BGR".
            If "BGR", will be converted to RGB before logging.
    """
    if not check_rerun_available():
        return

    if image is None:
        return

    path = entity_path or f"{CAMERA_PREFIX}/{camera_name}"

    # Handle CHW -> HWC conversion if needed
    if (
        image.ndim == 3
        and image.shape[0] in (1, 3, 4)
        and image.shape[-1] not in (1, 3, 4)
    ):
        image = np.transpose(image, (1, 2, 0))

    # Convert BGR to RGB if needed
    if color_format.upper() == "BGR" and image.ndim == 3 and image.shape[2] == 3:
        image = image[:, :, ::-1]

    rr.log(path, rr.Image(image))


def log_gripper_state(
    position: float | None = None,
    velocity: float | None = None,
    entity_path: str = GRIPPER_PREFIX,
) -> None:
    """
    Log gripper state (position, velocity) to Rerun.

    Args:
        position: Gripper position (0-85 range, 85 is fully open).
        velocity: Gripper velocity.
        entity_path: Base entity path for gripper data.
    """
    if not check_rerun_available():
        return

    if position is not None:
        rr.log(f"{entity_path}/position", rr.Scalars(float(position)))

    if velocity is not None:
        rr.log(f"{entity_path}/velocity", rr.Scalars(float(velocity)))


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [qx, qy, qz, qw] to 3x3 rotation matrix."""
    qx, qy, qz, qw = q
    
    # Rotation matrix from quaternion
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])


def log_coordinate_axes(
    entity_path: str,
    position: np.ndarray,
    rotation: np.ndarray,
    axis_length: float = 0.1,
) -> None:
    """
    Log coordinate axes (XYZ) at a given pose.
    
    Args:
        entity_path: Base entity path
        position: Position [x, y, z]
        rotation: Quaternion [qw, qx, qy, qz] (w, x, y, z order)
        axis_length: Length of each axis arrow
    """
    if not check_rerun_available():
        return
    
    # Convert from [qw, qx, qy, qz] to [qx, qy, qz, qw] for quaternion_to_rotation_matrix
    rot_xyzw = np.array([rotation[1], rotation[2], rotation[3], rotation[0]])
    
    # Convert quaternion to rotation matrix
    R = quaternion_to_rotation_matrix(rot_xyzw)
    
    # Axis directions in world frame
    x_axis = R @ np.array([axis_length, 0, 0])
    y_axis = R @ np.array([0, axis_length, 0])
    z_axis = R @ np.array([0, 0, axis_length])
    
    # Log arrows for each axis
    origins = np.array([position, position, position])
    vectors = np.array([x_axis, y_axis, z_axis])
    colors = np.array([
        [255, 0, 0],    # X - Red
        [0, 255, 0],    # Y - Green
        [0, 0, 255],    # Z - Blue
    ])
    
    rr.log(
        f"{entity_path}/axes",
        rr.Arrows3D(
            origins=origins,
            vectors=vectors,
            colors=colors,
            radii=0.003,
        ),
    )


def log_vive_pose(
    device_name: str,
    position: list[float] | np.ndarray,
    rotation: list[float] | np.ndarray,
    timestamp: float | None = None,
    log_trajectory: bool = True,
    entity_path: str | None = None,
) -> None:
    """
    Log Vive Tracker pose to Rerun.

    Logs:
    - 3D transform (position + rotation as quaternion)
    - Red point at tracker position
    - Coordinate axes (XYZ arrows)
    - Label with device name
    - Individual position/rotation components as scalars

    Args:
        device_name: Name of the Vive device (e.g., "T20", "HMD").
        position: Position [x, y, z] in meters.
        rotation: Quaternion [qw, qx, qy, qz] (w, x, y, z order).
        timestamp: Optional timestamp for the pose.
        log_trajectory: Whether to log trajectory points.
        entity_path: Custom base entity path. If None, uses "{TRACKER_PREFIX}/{device_name}".
    """
    if not check_rerun_available():
        return

    if position is None or rotation is None:
        return

    base_path = entity_path or f"{TRACKER_PREFIX}/{device_name}"
    pos = np.array(position)
    rot = np.array(rotation)  # [qw, qx, qy, qz]
    
    # Convert from [qw, qx, qy, qz] to [qx, qy, qz, qw] for Rerun
    rot_xyzw = np.array([rot[1], rot[2], rot[3], rot[0]])

    # Log 3D transform using Transform3D
    rr.log(
        f"{base_path}/pose",
        rr.Transform3D(
            translation=pos,
            rotation=rr.Quaternion(xyzw=rot_xyzw),
        ),
    )

    # Log position as RED 3D point for tracker (no label to avoid blocking trajectory)
    rr.log(
        f"{base_path}/point",
        rr.Points3D(
            [pos],
            radii=[0.015],
            colors=[[255, 50, 50]],  # Red
        ),
    )
    
    # Log coordinate axes (expects [qw, qx, qy, qz])
    log_coordinate_axes(base_path, pos, rot, axis_length=0.08)

    # Log individual position components as scalars for plotting
    rr.log(f"{base_path}/position/x", rr.Scalars(float(pos[0])))
    rr.log(f"{base_path}/position/y", rr.Scalars(float(pos[1])))
    rr.log(f"{base_path}/position/z", rr.Scalars(float(pos[2])))

    # Log quaternion components (in wxyz order as received)
    rr.log(f"{base_path}/rotation/qw", rr.Scalars(float(rot[0])))
    rr.log(f"{base_path}/rotation/qx", rr.Scalars(float(rot[1])))
    rr.log(f"{base_path}/rotation/qy", rr.Scalars(float(rot[2])))
    rr.log(f"{base_path}/rotation/qz", rr.Scalars(float(rot[3])))


class TrajectoryVisualizer:
    """
    Helper class to visualize trajectories over time.

    Maintains a history of positions for each device and renders
    them as line strips in 3D space.
    """

    def __init__(self, max_points: int = 500, line_radius: float = 0.005):
        """
        Initialize trajectory visualizer.

        Args:
            max_points: Maximum number of points to keep in trajectory history.
            line_radius: Radius of the trajectory line.
        """
        self.max_points = max_points
        self.line_radius = line_radius
        self.trajectories: dict[str, list[np.ndarray]] = defaultdict(list)

    def add_point(
        self,
        device_name: str,
        position: list[float] | np.ndarray,
        entity_path: str | None = None,
    ) -> None:
        """
        Add a point to the trajectory and log the updated trajectory.

        Args:
            device_name: Name of the device.
            position: Position [x, y, z] to add.
            entity_path: Custom entity path for the trajectory.
        """
        if not check_rerun_available():
            return

        pos = np.array(position)
        self.trajectories[device_name].append(pos)

        # Keep only the last max_points
        if len(self.trajectories[device_name]) > self.max_points:
            self.trajectories[device_name] = self.trajectories[device_name][
                -self.max_points :
            ]

        # Log trajectory as line strip
        path = entity_path or f"{TRACKER_PREFIX}/{device_name}/trajectory"
        points = np.array(self.trajectories[device_name])

        if len(points) >= 2:
            rr.log(path, rr.LineStrips3D([points], radii=[self.line_radius]))

    def clear(self, device_name: str | None = None) -> None:
        """
        Clear trajectory history.

        Args:
            device_name: Device to clear. If None, clears all.
        """
        if device_name:
            self.trajectories[device_name] = []
        else:
            self.trajectories.clear()


class XGripperVisualizer:
    """
    High-level visualizer for XGripper that integrates all visualization features.

    Example usage:
        viz = XGripperVisualizer()
        viz.init()

        while running:
            data = flare.recv_data()
            viz.log_all(data)
    """

    def __init__(
        self,
        session_name: str | None = None,
        max_trajectory_points: int = 500,
    ):
        """
        Initialize XGripper visualizer.

        Args:
            session_name: Name for the Rerun session. If None, generates a unique
                name based on timestamp to avoid loading cached blueprints.
            max_trajectory_points: Maximum points to keep in trajectory.
        """
        if session_name is None:
            session_name = "xgripper"
        self.session_name = session_name
        self.trajectory_viz = TrajectoryVisualizer(max_points=max_trajectory_points)
        self.initialized = False

    def init(
        self,
        spawn: bool = True,
        memory_limit: str | None = None,
        save_path: str | None = None,
    ) -> bool:
        """
        Initialize Rerun visualization.

        Args:
            spawn: Whether to spawn the Rerun viewer (synchronous workflow).
            memory_limit: Memory limit for Rerun.
            save_path: Path to save the recording (.rrd file). If provided,
                enables asynchronous workflow.

        Returns:
            bool: True if initialization was successful.

        Examples:
            # Real-time visualization
            viz.init(spawn=True)

            # Save only (no viewer)
            viz.init(spawn=False, save_path="/tmp/session.rrd")

            # Both
            viz.init(spawn=True, save_path="/tmp/session.rrd")
        """
        self.initialized = init_rerun(
            session_name=self.session_name,
            spawn=spawn,
            memory_limit=memory_limit,
            save_path=save_path,
        )
        return self.initialized

    def log_all(self, data: dict[str, Any]) -> None:
        """
        Log all data from FlareGrip.recv_data() output.

        Expected data format:
        {
            "wrist_img": np.ndarray or None,
            "gripper_position": float or None,
            "gripper_velocity": float or None,
            "ee_pose": dict[str, PoseData] or None,
            "sensor_rectify": dict[str, np.ndarray] or None,
        }

        Args:
            data: Data dictionary from FlareGrip.recv_data().
        """
        if not self.initialized:
            logger.warn("Visualizer not initialized. Call init() first.")
            return

        # Log camera image (BGR format from camera, convert to RGB)
        if data.get("wrist_img") is not None:
            log_camera_image(
                data["wrist_img"],
                camera_name="wrist",
                color_format="BGR",
            )

        # Log gripper state
        log_gripper_state(
            position=data.get("gripper_position"),
            velocity=data.get("gripper_velocity"),
        )

        # Log Vive tracker poses
        ee_pose = data.get("ee_pose")
        if ee_pose and isinstance(ee_pose, dict):
            for device_name, pose_data in ee_pose.items():
                if pose_data is None:
                    continue

                # Handle PoseData objects
                if hasattr(pose_data, "position") and hasattr(pose_data, "rotation"):
                    position = pose_data.position
                    rotation = pose_data.rotation
                else:
                    # Handle dict format
                    position = pose_data.get("position")
                    rotation = pose_data.get("rotation")

                if position is not None and rotation is not None:
                    # Log lighthouse as static reference points (no trajectory)
                    if device_name.startswith("LH"):
                        self._log_lighthouse(device_name, position, rotation)
                    else:
                        # Log tracker with full pose and trajectory
                        log_vive_pose(
                            device_name=device_name,
                            position=position,
                            rotation=rotation,
                        )
                        # Also add to trajectory
                        self.trajectory_viz.add_point(device_name, position)
        
        # Log sensor rectify data
        sensor_rectify = data.get("sensor_rectify")
        if sensor_rectify and isinstance(sensor_rectify, dict):
            for sn, rectify_img in sensor_rectify.items():
                if rectify_img is not None:
                    log_camera_image(
                        image=rectify_img,
                        camera_name=sn,
                        entity_path=f"sensors/{sn}/rectify",
                        color_format="BGR",  # Sensor outputs BGR format
                    )
    
    def _log_lighthouse(
        self,
        device_name: str,
        position: list[float] | np.ndarray,
        rotation: list[float] | np.ndarray,
    ) -> None:
        """Log lighthouse as a static reference marker with label and coordinate axes.
        
        Args:
            device_name: Name of the lighthouse device
            position: Position [x, y, z]
            rotation: Quaternion [qw, qx, qy, qz] (w, x, y, z order)
        """
        if not check_rerun_available():
            return
        
        pos = np.array(position)
        rot = np.array(rotation)  # [qw, qx, qy, qz]
        # Convert from [qw, qx, qy, qz] to [qx, qy, qz, qw] for Rerun
        rot_xyzw = np.array([rot[1], rot[2], rot[3], rot[0]])
        base_path = f"{TRACKER_PREFIX}/{device_name}"
        
        # Use green/blue colors for LH0/LH1
        if device_name == "LH0":
            color = [0, 255, 100]  # Green
        elif device_name == "LH1":
            color = [100, 180, 255]  # Blue
        else:
            color = [255, 200, 100]  # Orange
        
        # Log 3D transform
        rr.log(
            f"{base_path}/pose",
            rr.Transform3D(
                translation=pos,
                rotation=rr.Quaternion(xyzw=rot_xyzw),
            ),
        )
        
        # Log as LARGE 3D point with label
        rr.log(
            f"{base_path}/point",
            rr.Points3D(
                [pos],
                radii=[0.05],  # Larger radius for lighthouses
                colors=[color],
                labels=[device_name],  # Add label
            ),
        )
        
        # Log coordinate axes for lighthouse pose
        log_coordinate_axes(base_path, pos, rot, axis_length=0.15)

    def log_camera(self, image: np.ndarray, camera_name: str = "wrist") -> None:
        """Log a camera image."""
        if self.initialized:
            log_camera_image(image, camera_name=camera_name)

    def log_gripper(
        self,
        position: float | None = None,
        velocity: float | None = None,
    ) -> None:
        """Log gripper state."""
        if self.initialized:
            log_gripper_state(position=position, velocity=velocity)

    def log_tracker_pose(
        self,
        device_name: str,
        position: list[float] | np.ndarray,
        rotation: list[float] | np.ndarray,
        add_to_trajectory: bool = True,
    ) -> None:
        """
        Log a Vive tracker pose.

        Args:
            device_name: Name of the tracker device.
            position: Position [x, y, z].
            rotation: Quaternion [qx, qy, qz, qw].
            add_to_trajectory: Whether to add point to trajectory history.
        """
        if not self.initialized:
            logger.warn("Visualizer not initialized. Call init() first.")
            return

        log_vive_pose(device_name=device_name, position=position, rotation=rotation)

        if add_to_trajectory:
            self.trajectory_viz.add_point(device_name, position)

    def clear_trajectories(self, device_name: str | None = None) -> None:
        """Clear trajectory history for a device or all devices."""
        self.trajectory_viz.clear(device_name)


def log_xgripper_data(
    wrist_img: np.ndarray | None = None,
    gripper_position: float | None = None,
    gripper_velocity: float | None = None,
    vive_poses: dict[str, Any] | None = None,
) -> None:
    """
    Convenience function to log all XGripper data in one call.

    This is a stateless function - for trajectory visualization,
    use XGripperVisualizer class instead.

    Args:
        wrist_img: Camera image (HWC format).
        gripper_position: Gripper position value.
        gripper_velocity: Gripper velocity value.
        vive_poses: Dictionary of device_name -> PoseData or pose dict.
    """
    if not check_rerun_available():
        return

    # Log camera
    if wrist_img is not None:
        log_camera_image(wrist_img, camera_name="wrist")

    # Log gripper
    log_gripper_state(
        position=gripper_position,
        velocity=gripper_velocity,
    )

    # Log Vive poses
    if vive_poses:
        for device_name, pose_data in vive_poses.items():
            if pose_data is None or device_name.startswith("LH"):
                continue

            if hasattr(pose_data, "position") and hasattr(pose_data, "rotation"):
                position = pose_data.position
                rotation = pose_data.rotation
            else:
                position = pose_data.get("position")
                rotation = pose_data.get("rotation")

            if position is not None and rotation is not None:
                log_vive_pose(
                    device_name=device_name, position=position, rotation=rotation
                )
