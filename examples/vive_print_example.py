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
Vive Tracker Print Example - Print pose data to terminal
"""

import time
import argparse
import signal
import sys

from vive_tracker import ViveTracker
from utils.spdlogger import get_logger

# Get logger for this module
logger = get_logger("vive_print_example")

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    """Handle Ctrl+C for graceful shutdown"""
    global running
    print("\n\nReceived Ctrl+C, stopping...")
    running = False


def clear_line():
    """Clear current terminal line"""
    print("\033[K", end="")


def move_cursor_up(lines: int):
    """Move cursor up N lines"""
    print(f"\033[{lines}A", end="")


def print_pose_data(trackers_data: dict, update_count: int):
    """
    Print pose data for all trackers in a formatted table.
    
    Args:
        trackers_data: Dictionary of {device_name: PoseData}
        update_count: Current update count for display
    """
    # Header
    print(f"\n{'='*70}")
    print(f"  Vive Tracker Pose Data (Update #{update_count})")
    print(f"{'='*70}")
    
    if not trackers_data:
        print("  No tracker data available")
        print(f"{'='*70}\n")
        return
    
    for device_name, pose in trackers_data.items():
        if pose is None:
            print(f"  {device_name}: No pose data")
            continue
            
        pos = pose.position
        rot = pose.rotation
        
        print(f"\n  [{device_name}]")
        print(f"    Position (m):  X={pos[0]:+8.4f}  Y={pos[1]:+8.4f}  Z={pos[2]:+8.4f}")
        print(f"    Rotation (q):  X={rot[0]:+8.4f}  Y={rot[1]:+8.4f}  Z={rot[2]:+8.4f}  W={rot[3]:+8.4f}")
    
    print(f"\n{'='*70}")
    print("  Press Ctrl+C to stop")
    print(f"{'='*70}\n")


def run_terminal_visualization(rate_hz: float = 10.0):
    """
    Run terminal visualization for Vive Trackers.
    
    Args:
        rate_hz: Update rate in Hz (default 10 Hz)
    """
    global running
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    logger.info("Initializing Vive Tracker...")
    
    vt = ViveTracker()
    
    if not vt.connect():
        logger.error("Failed to connect to Vive Tracker")
        return False
    
    logger.info("Connected! Waiting for devices...")
    
    # Wait for devices
    devices = vt.wait_for_devices(timeout=10.0, required_trackers=1)
    
    # Log reference frame info
    vt.log_reference_frame_info()
    
    trackers = devices["trackers"]
    lighthouses = devices["lighthouses"]
    
    if not trackers:
        logger.error("No trackers detected!")
        vt.disconnect()
        return False
    
    logger.info(f"Found {len(trackers)} tracker(s): {trackers}")
    logger.info(f"Found {len(lighthouses)} lighthouse(s): {lighthouses}")
    logger.info(f"Starting terminal visualization at {rate_hz} Hz...")
    logger.info("Press Ctrl+C to stop\n")
    
    time.sleep(1.0)  # Give time for user to read
    
    update_count = 0
    sleep_time = 1.0 / rate_hz
    
    try:
        while running:
            # Get pose data for all trackers
            trackers_data = {}
            for tracker_name in trackers:
                pose = vt.get_pose(tracker_name)
                trackers_data[tracker_name] = pose
            
            # Clear screen and print data
            print("\033[2J\033[H", end="")  # Clear screen and move to top
            print_pose_data(trackers_data, update_count)
            
            update_count += 1
            time.sleep(sleep_time)
            
    except Exception as e:
        logger.error(f"Error during visualization: {e}")
    finally:
        logger.info("Disconnecting Vive Tracker...")
        vt.disconnect()
        logger.info("Done!")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Vive Tracker Terminal Visualization - Print pose data to terminal"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Update rate in Hz (default: 10)",
    )
    args = parser.parse_args()
    
    run_terminal_visualization(rate_hz=args.rate)


if __name__ == "__main__":
    main()
