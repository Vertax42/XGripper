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
Scan and list all Xense sensors connected to the system.

Usage:
    python examples/scan_sensors.py --mac YOUR_MAC_ADDRESS
"""

import argparse
from xesne_gripper import scan_sensors, list_sensors


def main():
    parser = argparse.ArgumentParser(description="Scan for Xense sensors")
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    args = parser.parse_args()

    print("=" * 50)
    print("Xense Sensor Scanner")
    print("=" * 50)
    print(f"MAC Address: {args.mac}")
    print()

    # Method 1: Get full sensor info
    print("Scanning for sensors (full info)...")
    sensors = scan_sensors(args.mac, verbose=True)
    
    print()
    
    # Method 2: Just get sensor serial numbers
    print("Sensor serial numbers only:")
    sns = list_sensors(args.mac)
    print(f"  {sns}")


if __name__ == "__main__":
    main()
