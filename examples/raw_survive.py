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
Raw pysurvive Example - Direct access to pysurvive for debugging

This script provides direct access to the pysurvive library without
the ViveTracker wrapper, useful for debugging and understanding
the raw data from Vive tracking system.

Usage:
    python examples/raw_survive.py
"""

import pysurvive
import sys
import time
import signal

from utils.spdlogger import get_logger

logger = get_logger("raw_survive")

running = True


def signal_handler(sig, frame):
    global running
    logger.info("Received Ctrl+C, stopping...")
    running = False


def main():
    global running
    
    signal.signal(signal.SIGINT, signal_handler)
    
    logger.info("Initializing pysurvive...")
actx = pysurvive.SimpleContext(sys.argv)

    logger.info("Waiting for devices...")
    time.sleep(2.0)
    
    # Print initial device list from Objects()
    devices = list(actx.Objects())
    logger.info(f"Objects() returned {len(devices)} devices:")
    for obj in devices:
        name = str(obj.Name(), "utf-8")
        logger.info(f"  - {name}")
    
    # Track devices seen via NextUpdated()
    seen_devices = {}
    
    logger.info("Starting pose streaming (Ctrl+C to stop)...")
    logger.info("Note: Trackers (WM*) only appear via NextUpdated(), not Objects()")
    print()
    
    last_print_time = {}

    while actx.Running() and running:
    updated = actx.NextUpdated()
    if updated:
            name = str(updated.Name(), "utf-8")
            
            # Track new devices
            if name not in seen_devices:
                seen_devices[name] = {"count": 0, "first_seen": time.time()}
                device_type = "Tracker" if name.startswith("WM") or name.startswith("T2") else "Lighthouse" if name.startswith("LH") else "Other"
                logger.info(f"New device via NextUpdated: {name} ({device_type})")
            
            seen_devices[name]["count"] += 1
            
            # Get pose data
        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]
            
            # Only print tracker data (not lighthouses), and limit rate
            if not name.startswith("LH"):
                current_time = time.time()
                if name not in last_print_time or current_time - last_print_time[name] >= 0.1:
                    last_print_time[name] = current_time
                    print(f"{name}: T:{poseTimestamp:10.4f} "
                          f"P:[{poseData.Pos[0]:+8.4f}, {poseData.Pos[1]:+8.4f}, {poseData.Pos[2]:+8.4f}] "
                          f"R:[{poseData.Rot[0]:+7.4f}, {poseData.Rot[1]:+7.4f}, {poseData.Rot[2]:+7.4f}, {poseData.Rot[3]:+7.4f}]")
        
        time.sleep(0.001)
    
    print()
    logger.info("Summary:")
    for name, info in seen_devices.items():
        logger.info(f"  {name}: {info['count']} updates")
    logger.info("Done!")


if __name__ == "__main__":
    main()
