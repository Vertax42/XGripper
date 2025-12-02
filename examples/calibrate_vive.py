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
Vive Tracker Calibration Script
This script forces recalibration of the Vive Tracker system using pysurvive.

Usage:
    python calibrate_vive.py [--timeout 60]

Instructions:
    1. Make sure all Lighthouse base stations are powered on and visible
    2. Place the tracker in a stable position with clear line of sight to all lighthouses
    3. Keep the tracker stationary during calibration
    4. Wait for calibration to complete
"""

import pysurvive
import sys
import time
import argparse


def print_banner():
    print("=" * 60)
    print("         Vive Tracker Calibration Tool")
    print("=" * 60)
    print()


def calibrate_vive(timeout=60):
    """
    Force recalibration of the Vive Tracker system.

    Args:
        timeout: Maximum time to wait for calibration (seconds)
    """
    print_banner()

    print("[INFO] Starting forced calibration...")
    print("[INFO] Please ensure:")
    print("       - All Lighthouse base stations are powered on")
    print("       - Tracker has clear line of sight to lighthouses")
    print("       - Tracker is stationary during calibration")
    print()

    # Build arguments with force-calibrate flag
    args = sys.argv[:1]  # Keep program name
    args.extend(["--force-calibrate"])

    # Initialize pysurvive context with calibration flag
    print("[INFO] Initializing pysurvive with --force-calibrate...")
    try:
        actx = pysurvive.SimpleContext(args)
    except Exception as e:
        print(f"[ERROR] Failed to initialize pysurvive: {e}")
        return False

    print("[INFO] Waiting for devices...")
    time.sleep(2.0)

    # Check for detected devices
    devices = list(actx.Objects())
    lighthouses = []
    trackers = []

    for obj in devices:
        name = str(obj.Name(), "utf-8")
        if name.startswith("LH"):
            lighthouses.append(name)
        else:
            trackers.append(name)

    print(f"[INFO] Detected Lighthouses: {lighthouses}")
    print(f"[INFO] Detected Trackers: {trackers}")

    if len(lighthouses) < 2:
        print(f"[WARNING] Only {len(lighthouses)} lighthouse(s) detected. "
              "For best results, 2 lighthouses are recommended.")

    if not trackers:
        print("[WARNING] No trackers detected yet. Continuing to wait...")

    print()
    print("[INFO] Calibrating... Keep the tracker stationary!")
    print("[INFO] Press Ctrl+C to stop")
    print()

    # Run calibration loop
    start_time = time.time()
    last_print_time = 0
    sample_count = 0
    valid_samples = 0

    try:
        while actx.Running():
            elapsed = time.time() - start_time

            # Check timeout
            if elapsed > timeout:
                print(f"\n[INFO] Calibration timeout ({timeout}s) reached.")
                break

            updated = actx.NextUpdated()
            if updated:
                sample_count += 1
                pose_obj = updated.Pose()
                pose_data = pose_obj[0]
                device_name = str(updated.Name(), "utf-8")

                # Check if position is reasonable (not NaN and within reasonable range)
                pos = [pose_data.Pos[0], pose_data.Pos[1], pose_data.Pos[2]]
                if all(abs(p) < 10 for p in pos) and not any(p != p for p in pos):  # Check NaN
                    valid_samples += 1

                # Print progress every 2 seconds
                current_time = time.time()
                if current_time - last_print_time >= 2.0:
                    last_print_time = current_time
                    print(f"[{elapsed:.0f}s] {device_name}: "
                          f"Pos=[{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}] "
                          f"Samples: {sample_count}, Valid: {valid_samples}")

            # Small sleep to prevent CPU overload
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[INFO] Calibration interrupted by user.")

    print()
    print("=" * 60)
    print("         Calibration Summary")
    print("=" * 60)
    print(f"Total samples collected: {sample_count}")
    print(f"Valid samples: {valid_samples}")
    print(f"Duration: {time.time() - start_time:.1f} seconds")
    print()

    if valid_samples > 100:
        print("[SUCCESS] Calibration completed successfully!")
        print("[INFO] Configuration should be saved to ~/.config/libsurvive/")
        return True
    else:
        print("[WARNING] Low number of valid samples. Calibration may not be accurate.")
        print("[TIP] Try the following:")
        print("      - Check if lighthouses are powered on and have green LED")
        print("      - Ensure tracker has clear line of sight to lighthouses")
        print("      - Move tracker closer to lighthouses")
        print("      - Run calibration for longer duration with --timeout")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Vive Tracker Calibration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python calibrate_vive.py                  # Default 60 second calibration
    python calibrate_vive.py --timeout 120    # 2 minute calibration
        """
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Calibration timeout in seconds (default: 60)"
    )

    args = parser.parse_args()
    calibrate_vive(args.timeout)


if __name__ == "__main__":
    main()
