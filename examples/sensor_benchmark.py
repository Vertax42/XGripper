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
Sensor Benchmark Example

Compare sensor reading performance between:
1. Direct xensesdk usage (remote_example.py style)
2. FlareGrip class usage

This example uses FlareGrip to read two sensors' Rectify images
and visualize them in Rerun.

Usage:
    python examples/sensor_benchmark.py --mac 6ebbc5f53240
"""

import sys
import time
import argparse

from xensesdk import Sensor
from xesne_gripper import FlareGrip
from utils.visualization import XGripperVisualizer, RERUN_AVAILABLE, log_camera_image
from utils.spdlogger import get_logger

logger = get_logger("sensor_benchmark")
# @profile
def main():
    parser = argparse.ArgumentParser(
        description="Sensor Benchmark - Compare FlareGrip sensor reading with Rerun visualization",
    )
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    parser.add_argument(
        "--no-rerun",
        action="store_true",
        help="Disable Rerun visualization",
    )
    args = parser.parse_args()

    # Check Rerun availability
    if not args.no_rerun and not RERUN_AVAILABLE:
        logger.error("Rerun is not installed. Install with: pip install rerun-sdk")
        sys.exit(1)

    logger.info("=" * 70)
    logger.info("Sensor Benchmark - FlareGrip + Rerun")
    logger.info("=" * 70)
    logger.info(f"MAC Address: {args.mac}")
    logger.info(f"Rerun: {'OFF' if args.no_rerun else 'ON'}")
    logger.info("=" * 70)

    # Initialize Rerun visualizer
    viz = None
    if not args.no_rerun:
        logger.info("Initializing Rerun visualizer...")
        viz = XGripperVisualizer(session_name="sensor_benchmark")
        if not viz.init():
            logger.error("Failed to initialize Rerun visualizer")
            sys.exit(1)
        logger.info("Rerun visualizer initialized!")

    # Initialize FlareGrip (only sensors, disable other components)
    logger.info("Initializing FlareGrip (sensors only)...")
    flare = FlareGrip(
        mac_addr=args.mac,
        log_level="INFO",
        no_gripper=True,
        no_vive=True,
        no_cam=True,
        no_sensor=False,  # Enable sensors
    )
    
    # Check sensors
    sensor_count = len(flare._fg_sensors)
    if sensor_count < 2:
        logger.error(f"Need at least 2 sensors, found {sensor_count}")
        flare.close()
        sys.exit(1)
    
    sensor_sns = list(flare._fg_sensors.keys())
    logger.info(f"Found {sensor_count} sensors: {sensor_sns}")

    # FPS tracking
    frame_count = 0
    last_fps_time = time.time()
    fps_update_interval = 10

    # Timing accumulators
    time_sensor0_read = 0.0
    time_sensor1_read = 0.0
    time_rerun = 0.0

    logger.info("=" * 70)
    logger.info("Starting benchmark loop... Press Ctrl+C to stop")
    logger.info("=" * 70)

    try:
        while True:
            # Read sensor 0
            t0 = time.perf_counter()
            sensor_0 = flare._fg_sensors[sensor_sns[0]]
            rectify_0 = sensor_0.selectSensorInfo(Sensor.OutputType.Rectify)
            t1 = time.perf_counter()
            time_sensor0_read += (t1 - t0)

            # Read sensor 1
            sensor_1 = flare._fg_sensors[sensor_sns[1]]
            rectify_1 = sensor_1.selectSensorInfo(Sensor.OutputType.Rectify)
            t2 = time.perf_counter()
            time_sensor1_read += (t2 - t1)

            # Log to Rerun
            if viz and rectify_0 is not None and rectify_1 is not None:
                t3 = time.perf_counter()
                log_camera_image(
                    image=rectify_0,
                    entity_path=f"sensors/{sensor_sns[0]}/rectify",
                    color_format="BGR",
                )
                log_camera_image(
                    image=rectify_1,
                    entity_path=f"sensors/{sensor_sns[1]}/rectify",
                    color_format="BGR",
                )
                t4 = time.perf_counter()
                time_rerun += (t4 - t3)

            frame_count += 1

            # Print stats every N frames
            if frame_count % fps_update_interval == 0:
                current_time = time.time()
                fps = fps_update_interval / (current_time - last_fps_time)
                last_fps_time = current_time

                # Calculate average timing (in ms)
                avg_s0 = (time_sensor0_read / fps_update_interval) * 1000
                avg_s1 = (time_sensor1_read / fps_update_interval) * 1000
                avg_rr = (time_rerun / fps_update_interval) * 1000
                total = avg_s0 + avg_s1 + avg_rr

                logger.info(
                    f"FPS: {fps:.1f} | "
                    f"S0({sensor_sns[0][:8]}): {avg_s0:.1f}ms | "
                    f"S1({sensor_sns[1][:8]}): {avg_s1:.1f}ms | "
                    f"Rerun: {avg_rr:.1f}ms | "
                    f"Total: {total:.1f}ms"
                )

                # Reset accumulators
                time_sensor0_read = 0.0
                time_sensor1_read = 0.0
                time_rerun = 0.0

    except KeyboardInterrupt:
        logger.info("\nInterrupted by user.")
    finally:
        logger.info("Closing FlareGrip...")
        flare.close()
        logger.info("Done.")


if __name__ == "__main__":
    main()

