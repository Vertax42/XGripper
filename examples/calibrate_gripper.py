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
Gripper Calibration Script
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse  # noqa: E402

from xesne_gripper import FlareGrip  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Gripper Calibration Script")
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    args = parser.parse_args()

    print("=" * 50)
    print("Gripper Calibration")
    print("=" * 50)

    print(f"\nInitializing FlareGrip (MAC: {args.mac})...")
    flare = FlareGrip(
        mac_addr=args.mac,
        log_level="INFO",
        no_sensor=True,
        no_vive=True,
        no_cam=True,
    )
    print("FlareGrip initialized!")

    try:
        print("\nPlease ensure the gripper is in a safe position.")
        print("The gripper will move during calibration.")
        input("\nPress Enter to start calibration...")

        print("\nCalibrating gripper...")
        flare.calibrate_gripper()
        print("\nGripper calibration complete!")

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled.")
    finally:
        flare.close()


if __name__ == "__main__":
    main()
