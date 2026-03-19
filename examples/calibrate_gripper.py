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

Usage:
    python examples/calibrate_gripper.py --port /dev/ttyUSB0
"""

import sys
import time
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xensegripper import XenseSerialGripper  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Gripper Calibration Script")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="Serial port path (default: /dev/ttyUSB0)")
    args = parser.parse_args()

    print("=" * 50)
    print("Gripper Calibration")
    print("=" * 50)

    print(f"\nInitializing gripper on port: {args.port}...")
    gripper = XenseSerialGripper(args.port)
    print("Gripper initialized!")

    try:
        print("\nPlease ensure the gripper is in a safe position.")
        print("The gripper will move during calibration.")
        input("\nPress Enter to start calibration...")

        print("\nCalibrating gripper...")
        gripper.calibrate()
        time.sleep(3)
        print("\nGripper calibration complete!")

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled.")
    finally:
        gripper.release()


if __name__ == "__main__":
    main()
