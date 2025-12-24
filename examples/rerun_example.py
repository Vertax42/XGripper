#!/usr/bin/env python
"""
Rerun example: Visualizing Vive Tracker pose and its inverse.

Demonstrates how to visualize position (xyz) + rotation (quaternion) using Rerun.
"""

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation


def log_pose_with_axes(
    entity_path: str,
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
    label: str,
    axis_length: float = 0.1,
    color: tuple[int, int, int] = (255, 255, 255),
) -> None:
    """
    Log a 3D pose with coordinate axes visualization.

    Args:
        entity_path: Rerun entity path for this pose.
        position: Position [x, y, z] in meters.
        quaternion_xyzw: Quaternion [qx, qy, qz, qw] (scipy convention).
        label: Text label to display.
        axis_length: Length of the coordinate axes arrows.
        color: RGB color for the origin point and label.
    """
    # Log the transform (this defines the coordinate frame)
    rr.log(
        f"{entity_path}/transform",
        rr.Transform3D(
            translation=position,
            rotation=rr.Quaternion(xyzw=quaternion_xyzw),
        ),
    )

    # Log coordinate axes as arrows (child of transform, so they inherit the transform)
    # X-axis: Red, Y-axis: Green, Z-axis: Blue
    rr.log(
        f"{entity_path}/transform/axes",
        rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[
                [axis_length, 0, 0],  # X
                [0, axis_length, 0],  # Y
                [0, 0, axis_length],  # Z
            ],
            colors=[
                [255, 0, 0],  # Red for X
                [0, 255, 0],  # Green for Y
                [0, 0, 255],  # Blue for Z
            ],
            radii=0.005,
        ),
    )

    # Log a point at the origin of the pose
    rr.log(
        f"{entity_path}/transform/origin",
        rr.Points3D([[0, 0, 0]], radii=0.015, colors=[color]),
    )

    # Log the text label slightly above the pose
    rr.log(
        f"{entity_path}/label",
        rr.Points3D(
            [position + np.array([0, 0, 0.05])],  # Offset above
            labels=[label],
            radii=0.005,
            colors=[color],
        ),
    )


def compute_pose_inverse(
    position: np.ndarray, quaternion_xyzw: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute the inverse of a pose (position + quaternion).

    For a pose T = (R, t), the inverse is T^-1 = (R^-1, -R^-1 * t)

    Args:
        position: Position [x, y, z].
        quaternion_xyzw: Quaternion [qx, qy, qz, qw].

    Returns:
        Tuple of (inverse_position, inverse_quaternion_xyzw).
    """
    # Create rotation from quaternion
    rot = Rotation.from_quat(quaternion_xyzw)

    # Inverse rotation
    rot_inv = rot.inv()

    # Inverse position: -R^-1 * t
    pos_inv = -rot_inv.apply(position)

    # Get quaternion of inverse rotation
    quat_inv = rot_inv.as_quat()  # [qx, qy, qz, qw]

    return pos_inv, quat_inv


def main():
    # Initialize Rerun
    rr.init("vive_tracker_pose_example")
    rr.spawn()

    print("Visualizing Vive Tracker poses in Rerun...")

    # === Vive Tracker pose (from user input) ===
    # Format: x, y, z, qw, qx, qy, qz
    # raw_data = [0.000, 0.021, 0.160, 0.676, -0.207, 0.207, 0.676]
    raw_data = [0.000, 0.021, 0.160, 0.207, 0.676, 0.676, -0.207]
    # Parse: xyz + qw,qx,qy,qz -> convert to scipy format (qx,qy,qz,qw)
    position = np.array(raw_data[0:3])  # [x, y, z]
    qw, qx, qy, qz = raw_data[3], raw_data[4], raw_data[5], raw_data[6]
    quaternion_xyzw = np.array([qx, qy, qz, qw])  # scipy format

    # Convert to Euler for display
    euler_deg = Rotation.from_quat(quaternion_xyzw).as_euler("xyz", degrees=True)

    # === Log world origin axes (for reference) ===
    rr.log(
        "world/axes",
        rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[
                [0.3, 0, 0],
                [0, 0.3, 0],
                [0, 0, 0.3],
            ],
            colors=[
                [128, 0, 0],  # Dark red for X
                [0, 128, 0],  # Dark green for Y
                [0, 0, 128],  # Dark blue for Z
            ],
            radii=0.008,
            labels=["X", "Y", "Z"],
        ),
    )

    # === Log raw_data pose (vive_tracker -> EE) ===
    label = (
        f"vive_tracker_to_ee\n"
        f"pos: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]\n"
        f"rot: [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]"
    )

    log_pose_with_axes(
        entity_path="vive_tracker_to_ee",
        position=position,
        quaternion_xyzw=quaternion_xyzw,
        label=label,
        axis_length=0.1,
        color=(0, 255, 255),  # Cyan
    )

    # Print pose info to console
    print("\n=== Vive Tracker -> EE Pose (raw_data) ===")
    print(f"Position (xyz): {position}")
    print(f"Quaternion (qw,qx,qy,qz): [{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]")
    print(f"Euler (xyz, deg): {euler_deg}")

    print("\n✓ Pose logged to Rerun. Check the viewer!")
    print("  - Cyan point: vive_tracker_to_ee")
    print("  - Red axis = X, Green axis = Y, Blue axis = Z")


if __name__ == "__main__":
    main()
