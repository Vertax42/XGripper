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
Gripper Test Example - Test gripper data reading from FlareGrip

Usage:
    python examples/test_gripper.py --mac 6ebbc5f53240
"""

import sys
import time
import argparse
import signal

from xesne_gripper import FlareGrip
from utils.spdlogger import get_logger

logger = get_logger("test_gripper")

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    """Handle Ctrl+C for graceful shutdown"""
    global running
    logger.info("\nReceived Ctrl+C, stopping...")
    running = False


def main():
    parser = argparse.ArgumentParser(
        description="Test gripper data reading from FlareGrip"
    )
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Target update rate in Hz (default: 30)",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Run benchmark mode (no rate limiting)",
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    logger.info("=" * 70)
    logger.info("Gripper Test - FlareGrip")
    logger.info("=" * 70)
    logger.info(f"MAC Address: {args.mac}")
    logger.info(f"Mode: {'Benchmark' if args.benchmark else f'Normal ({args.rate} Hz)'}")
    logger.info("=" * 70)

    # Initialize FlareGrip (gripper only, disable other components)
    logger.info("Initializing FlareGrip (gripper only)...")
    flare = FlareGrip(
        mac_addr=args.mac,
        log_level="INFO",
        no_gripper=False,  # Enable gripper
        no_sensor=True,
        no_vive=True,
        no_cam=True,
    )

    # Check if gripper is initialized
    if flare._fg_gripper is None:
        logger.error("Gripper failed to initialize!")
        flare.close()
        sys.exit(1)

    logger.info("FlareGrip initialized!")
    logger.info("=" * 70)

    # Wait for gripper data to be available
    logger.info("Waiting for gripper data...")
    gripper_ready = False
    for i in range(20):  # Wait up to 10 seconds
        data = flare.recv_data(gripper=True, ee_pose=False, wrist_img=False, sensor=False)
        if data.get("gripper_position") is not None:
            logger.info(f"Gripper data received after {i+1} attempts!")
            logger.info(f"  Position: {data['gripper_position']:.2f}")
            logger.info(f"  Velocity: {data['gripper_velocity']:.2f}")
            logger.info(f"  Force: {data['gripper_force']:.2f}")
            gripper_ready = True
            break
        time.sleep(0.5)

    if not gripper_ready:
        logger.warn("Gripper data not available after 10 seconds")
        logger.warn("The gripper may need calibration: python examples/calibrate_gripper.py --mac " + args.mac)

    logger.info("=" * 70)
    logger.info("Starting continuous reading... Press Ctrl+C to stop")
    logger.info("=" * 70)

    # Performance tracking
    frame_count = 0
    last_fps_time = time.time()
    fps_update_interval = 10
    time_gripper_read = 0.0
    
    sleep_time = 1.0 / args.rate if not args.benchmark else 0

    try:
        while running:
            loop_start = time.perf_counter()

            # Read gripper data
            t0 = time.perf_counter()
            data = flare.recv_data(gripper=True, ee_pose=False, wrist_img=False, sensor=False)
            t1 = time.perf_counter()
            time_gripper_read += (t1 - t0)

            frame_count += 1

            # Print stats every N frames
            if frame_count % fps_update_interval == 0:
                current_time = time.time()
                fps = fps_update_interval / (current_time - last_fps_time)
                last_fps_time = current_time

                avg_read = (time_gripper_read / fps_update_interval) * 1000

                pos = data.get("gripper_position")
                vel = data.get("gripper_velocity")
                force = data.get("gripper_force")

                if pos is not None:
                    logger.info(
                        f"FPS: {fps:.1f} | Read: {avg_read:.2f}ms | "
                        f"Pos: {pos:.2f} | Vel: {vel:.2f} | Force: {force:.2f}"
                    )
                else:
                    logger.info(
                        f"FPS: {fps:.1f} | Read: {avg_read:.2f}ms | "
                        f"Gripper data: None"
                    )

                time_gripper_read = 0.0

            # Rate limiting (if not benchmark mode)
            if not args.benchmark and sleep_time > 0:
                elapsed = time.perf_counter() - loop_start
                remaining = sleep_time - elapsed
                if remaining > 0:
                    time.sleep(remaining)

    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        logger.info("Closing FlareGrip...")
        flare.close()
        logger.info("Done!")


if __name__ == "__main__":
    main()
