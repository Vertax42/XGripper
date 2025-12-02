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
XGripper Example Script
Test the functionalities of XenseGripper and print the data
"""

import time
import logging
import argparse
import numpy as np

from xesne_gripper import FlareGrip


def button_callback(event):
    """button callback function"""
    print(f"[Button Event] {event}")


def print_sensor_data(flare: FlareGrip):
    """print sensor data"""
    print("\n" + "=" * 50)
    print("Sensor Data:")
    print("=" * 50)

    # try to get the first sensor
    sensor = flare.sensor(0)
    if sensor is not None:
        try:
            # get sensor data (adjust according to the actual SDK)
            print(f"  Sensor found: {sensor}")
        except Exception as e:
            print(f"  Error reading sensor: {e}")
    else:
        print("  No sensors available")


def print_pose_data(flare: FlareGrip):
    """print Vive Tracker pose data"""
    print("\n" + "=" * 50)
    print("Vive Tracker Pose Data:")
    print("=" * 50)

    pose = flare.get_pose()
    if pose:
        if isinstance(pose, dict):
            for device_name, pose_data in pose.items():
                print(f"  Device: {device_name}")
                if hasattr(pose_data, 'position') and hasattr(pose_data, 'rotation'):
                    pos = pose_data.position
                    rot = pose_data.rotation
                    print(f"    Position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
                    print(f"    Rotation: qx={rot[0]:.4f}, qy={rot[1]:.4f}, qz={rot[2]:.4f}, qw={rot[3]:.4f}")
                else:
                    print(f"    Data: {pose_data}")
        elif hasattr(pose, 'position') and hasattr(pose, 'rotation'):
            pos = pose.position
            rot = pose.rotation
            print(f"  Position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
            print(f"  Rotation: qx={rot[0]:.4f}, qy={rot[1]:.4f}, qz={rot[2]:.4f}, qw={rot[3]:.4f}")
        else:
            print(f"  Pose: {pose}")
    else:
        print("  No pose data available")


def print_recv_data(data: dict):
    """print received data"""
    print("\n" + "-" * 50)
    print("Received Data:")
    print("-" * 50)

    # print end-effector pose
    ee_pose = data.get("ee_pose")
    if ee_pose is not None:
        # PoseData object has position [x, y, z] and rotation [qx, qy, qz, qw]
        if hasattr(ee_pose, 'position') and hasattr(ee_pose, 'rotation'):
            pos = ee_pose.position
            rot = ee_pose.rotation
            print("  EE Pose:")
            print(f"    Position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
            print(f"    Rotation: qx={rot[0]:.4f}, qy={rot[1]:.4f}, qz={rot[2]:.4f}, qw={rot[3]:.4f}")
        else:
            print(f"  EE Pose: {ee_pose}")
    else:
        print("  EE Pose: N/A")

    # print gripper status
    if data.get("gripper_position") is not None:
        print(f"  Gripper Position: {data['gripper_position']:.4f}")
        print(f"  Gripper Velocity: {data['gripper_velocity']:.4f}")
        print(f"  Gripper Force: {data['gripper_force']:.4f}")
    else:
        print("  Gripper: N/A")

    # print camera image information
    if data.get("wrist_img") is not None:
        img = data["wrist_img"]
        if isinstance(img, np.ndarray):
            print(f"  Wrist Image: shape={img.shape}, dtype={img.dtype}")
        else:
            print(f"  Wrist Image: {type(img)}")
    else:
        print("  Wrist Image: N/A")


def main():
    parser = argparse.ArgumentParser(description="FlareGrip Example Script")
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    parser.add_argument(
        "--vive-sn",
        type=str,
        default=None,
        help="Vive Tracker serial number",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Disable gripper initialization",
    )
    parser.add_argument(
        "--no-sensor",
        action="store_true",
        help="Disable sensor initialization",
    )
    parser.add_argument(
        "--no-vive",
        action="store_true",
        help="Disable Vive Tracker initialization",
    )
    parser.add_argument(
        "--no-cam",
        action="store_true",
        help="Disable camera initialization",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Continuously print data in a loop",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Interval between data prints (seconds)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("FlareGrip Example Script")
    print("=" * 60)
    print(f"MAC Address: {args.mac}")
    print(f"Vive SN: {args.vive_sn}")
    print(f"Options: gripper={'OFF' if args.no_gripper else 'ON'}, "
          f"sensor={'OFF' if args.no_sensor else 'ON'}, "
          f"vive={'OFF' if args.no_vive else 'ON'}, "
          f"cam={'OFF' if args.no_cam else 'ON'}")
    print("=" * 60)

    # initialize FlareGrip
    print("\nInitializing FlareGrip...")
    flare = FlareGrip(
        mac_addr=args.mac,
        vive_sn=args.vive_sn,
        cam_size=(640, 480),
        log_level=logging.INFO,
        no_gripper=args.no_gripper,
        no_sensor=args.no_sensor,
        no_vive=args.no_vive,
        no_cam=args.no_cam,
    )
    print("FlareGrip initialized successfully!")

    # register button callback
    # Valid event types: CLICK, DOUBLE_CLICK, LONG_PRESS, PRESS, RELEASE
    if not args.no_gripper:
        flare.register_button_callback("PRESS", button_callback)
        flare.register_button_callback("RELEASE", button_callback)
        print("Button callbacks registered.")

    try:
        # print sensor data
        if not args.no_sensor:
            print_sensor_data(flare)

        # print pose data
        if not args.no_vive:
            print_pose_data(flare)

        # loop to print data
        if args.loop:
            print("\nStarting data loop (Press Ctrl+C to stop)...")
            while True:
                data = flare.recv_data(
                    ee_pose=not args.no_vive,
                    gripper=not args.no_gripper,
                    wrist_img=not args.no_cam,
                )
                print_recv_data(data)
                time.sleep(args.interval)
        else:
            # get data once
            print("\nFetching data once...")
            data = flare.recv_data(
                ee_pose=not args.no_vive,
                gripper=not args.no_gripper,
                wrist_img=not args.no_cam,
            )
            print_recv_data(data)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        print("\nClosing FlareGrip...")
        flare.close()
        print("Done.")


if __name__ == "__main__":
    main()
