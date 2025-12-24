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
XGripper Visualization Example

This example demonstrates how to use the Rerun-based visualization module
to visualize camera images, gripper state, and Vive tracker poses in real-time.

Usage:
    python examples/rerun_visualization_example.py --mac YOUR_MAC_ADDRESS

Requirements:
    pip install rerun-sdk
"""

import sys
import time
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xesne_gripper import FlareGrip  # noqa: E402
from utils.visualization import XGripperVisualizer, RERUN_AVAILABLE  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="XGripper Visualization Example")
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Disable gripper (for testing without hardware)",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Disable camera",
    )
    parser.add_argument(
        "--no-vive",
        action="store_true",
        help="Disable Vive tracker",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=30.0,
        help="Update rate in Hz (default: 30)",
    )
    args = parser.parse_args()

    if not RERUN_AVAILABLE:
        print("Error: Rerun is not installed. Install it with: pip install rerun-sdk")
        sys.exit(1)

    print("=" * 50)
    print("XGripper Visualization Example")
    print("=" * 50)

    # Initialize visualizer with unique session name (avoids loading cached blueprint)
    print("\nInitializing Rerun visualizer...")
    session_name = f"xgripper_{int(time.time())}"
    viz = XGripperVisualizer(session_name=session_name)
    if not viz.init():
        print("Error: Failed to initialize Rerun visualizer")
        sys.exit(1)
    print("Rerun visualizer initialized! A viewer window should open.")

    # Initialize FlareGrip
    print(f"\nInitializing FlareGrip (MAC: {args.mac})...")
    try:
        flare = FlareGrip(
            mac_addr=args.mac,
            log_level="INFO",
            no_gripper=args.no_gripper,
            no_sensor=True,  # Skip sensors for visualization demo
            no_vive=args.no_vive,
            no_cam=args.no_camera,
        )
        print("FlareGrip initialized!")
    except Exception as e:
        print(f"Error initializing FlareGrip: {e}")
        print("\nRunning in demo mode with synthetic data...")
        run_demo_mode(viz, args.hz)
        return

    # Main visualization loop
    print("\nStarting visualization loop...")
    print("Press Ctrl+C to stop.\n")

    interval = 1.0 / args.hz

    try:
        while True:
            start_time = time.time()

            # Get data from FlareGrip
            data = flare.recv_data(
                ee_pose=not args.no_vive,
                gripper=not args.no_gripper,
                wrist_img=not args.no_camera,
            )

            # Log all data to Rerun
            viz.log_all(data)

            # Maintain update rate
            elapsed = time.time() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nStopping visualization...")
    finally:
        flare.close()
        print("Done!")


def run_demo_mode(viz: XGripperVisualizer, hz: float = 30.0):
    """
    Run visualization with synthetic demo data.

    Useful for testing visualization without actual hardware.
    """
    import numpy as np

    print("\n" + "=" * 50)
    print("DEMO MODE - Using synthetic data")
    print("=" * 50)
    print("\nGenerating synthetic data for visualization...")
    print("Press Ctrl+C to stop.\n")

    interval = 1.0 / hz
    t = 0.0

    try:
        while True:
            start_time = time.time()

            # Generate synthetic camera image (gradient with moving pattern)
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            # Create gradient background
            img[:, :, 0] = np.linspace(50, 150, 640).astype(np.uint8)  # Blue gradient
            img[:, :, 1] = 80  # Green
            img[:, :, 2] = np.linspace(100, 200, 480).reshape(-1, 1).astype(np.uint8)  # Red gradient
            # Add moving circle
            cx = int(320 + 200 * np.sin(t * 2))
            cy = int(240 + 100 * np.cos(t * 3))
            y, x = np.ogrid[:480, :640]
            mask = (x - cx) ** 2 + (y - cy) ** 2 < 50**2
            img[mask] = [255, 200, 100]

            # Generate synthetic gripper data (oscillating)
            gripper_pos = 50 + 40 * np.sin(t)
            gripper_vel = 40 * np.cos(t)
            gripper_force = 10 + 5 * np.abs(np.sin(t * 0.5))

            # Generate synthetic Vive tracker pose (circular motion)
            tracker_pos = [
                0.3 * np.cos(t * 0.5),  # x
                0.2 * np.sin(t * 0.5),  # y
                0.5 + 0.1 * np.sin(t),  # z
            ]
            # Simple rotation quaternion (rotating around z-axis)
            angle = t * 0.3
            tracker_rot = [0, 0, np.sin(angle / 2), np.cos(angle / 2)]  # qx, qy, qz, qw

            # Log to visualizer
            viz.log_camera(img, camera_name="wrist")
            viz.log_gripper(
                position=gripper_pos,
                velocity=gripper_vel,
                force=gripper_force,
            )
            viz.log_tracker_pose(
                device_name="T20_demo",
                position=tracker_pos,
                rotation=tracker_rot,
            )

            t += interval

            # Maintain update rate
            elapsed = time.time() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nStopping demo...")
        print("Done!")


if __name__ == "__main__":
    main()
