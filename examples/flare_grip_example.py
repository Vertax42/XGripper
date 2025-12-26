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
FlareGrip Example Script

Demonstrates FlareGrip functionality with Rerun visualization.
All sensor data (Vive tracker, gripper, camera, tactile sensors) is 
visualized in the Rerun viewer.

Usage:
    python examples/flare_grip_example.py --mac YOUR_MAC_ADDRESS
"""

import sys
import time
import argparse
import numpy as np

from xesne_gripper import FlareGrip
from utils.visualization import XGripperVisualizer, RERUN_AVAILABLE
from utils.spdlogger import get_logger

# Create logger
logger = get_logger("flare_grip_example")


def move_cursor_up(lines: int):
    """Move cursor up N lines"""
    sys.stdout.write(f"\033[{lines}A")
    sys.stdout.flush()


def button_callback(event):
    """Button callback function"""
    logger.info(f"[Button Event] {event}")


def format_live_data(flare: FlareGrip, data: dict, frame_count: int, fps: float, timing: dict = None) -> list[str]:
    """Format live data for terminal display, returns list of lines"""
    lines = []
    lines.append("=" * 70)
    lines.append(f"  FlareGrip Live Monitor | Frame: {frame_count:6d} | FPS: {fps:5.1f}")
    lines.append("=" * 70)
    
    # Timing Section (if available)
    if timing:
        lines.append("")
        lines.append("┌─ ⏱️  Latency (ms) ──────────────────────────────────────────────────")
        lines.append(f"│  Data (recv_data): {timing.get('data', 0):.1f}  |  "
                    f"Rerun: {timing.get('rerun', 0):.1f}  |  Total: {timing.get('total', 0):.1f}")
        lines.append("└" + "─" * 69)
    
    # Vive Tracker Section
    lines.append("")
    lines.append("┌─ 🎯 Vive Tracker ─────────────────────────────────────────────────")
    pose = data.get("ee_pose")
    if pose is not None:
        if isinstance(pose, dict):
            for device_name, pose_data in pose.items():
                if hasattr(pose_data, 'position') and hasattr(pose_data, 'rotation'):
                    pos = pose_data.position
                    rot = pose_data.rotation
                    # Format device type indicator
                    if device_name.startswith("LH"):
                        icon = "🔦"  # Lighthouse
                    elif device_name.startswith("WM") or device_name.startswith("T"):
                        icon = "📡"  # Tracker
                    else:
                        icon = "📍"
                    lines.append(f"│  {icon} {device_name:6s} Pos: [{pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}]  "
                               f"Rot: [{rot[0]:+6.3f}, {rot[1]:+6.3f}, {rot[2]:+6.3f}, {rot[3]:+6.3f}]")
        elif hasattr(pose, 'position') and hasattr(pose, 'rotation'):
            pos = pose.position
            rot = pose.rotation
            lines.append(f"│  📡 Tracker Pos: [{pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}]  "
                       f"Rot: [{rot[0]:+6.3f}, {rot[1]:+6.3f}, {rot[2]:+6.3f}, {rot[3]:+6.3f}]")
    else:
        lines.append("│  (No Vive data)")
    lines.append("└" + "─" * 69)
    
    # Gripper Section
    lines.append("")
    lines.append("┌─ 🤖 Gripper ───────────────────────────────────────────────────────")
    if data.get("gripper_position") is not None:
        pos = data['gripper_position']
        vel = data['gripper_velocity']
        # Create visual bar for position (0-85, where 85 is fully open)
        GRIPPER_MAX_POS = 85.0
        bar_width = 20
        filled = int(max(0, min(GRIPPER_MAX_POS, pos)) / GRIPPER_MAX_POS * bar_width)
        bar = "█" * filled + "░" * (bar_width - filled)
        lines.append(f"│  Position: [{bar}] {pos:6.2f}")
        lines.append(f"│  Velocity: {vel:+8.3f}")
    else:
        lines.append("│  (No gripper data)")
    lines.append("└" + "─" * 69)
    
    # Sensor Section
    lines.append("")
    lines.append("┌─ 🦖 Sensors ───────────────────────────────────────────────────────")
    if flare._fg_sensors:
        for i, (sn, sensor) in enumerate(flare._fg_sensors.items()):
            lines.append(f"│  [{i}] SN: {sn} ✓ (Rerun: sensors/{sn}/rectify)")
    else:
        lines.append("│  (No sensors)")
    lines.append("└" + "─" * 69)
    
    # Camera Section
    lines.append("")
    lines.append("┌─ 📷 Camera ────────────────────────────────────────────────────────")
    if data.get("wrist_img") is not None:
        img = data["wrist_img"]
        if isinstance(img, np.ndarray):
            h, w = img.shape[:2]
            channels = img.shape[2] if len(img.shape) > 2 else 1
            lines.append(f"│  Image: {w}x{h} ({channels}ch) | dtype: {img.dtype}")
            lines.append(f"│  (View in Rerun: camera/wrist)")
        else:
            lines.append(f"│  Image type: {type(img)}")
    else:
        lines.append("│  (No camera data)")
    lines.append("└" + "─" * 69)
    
    lines.append("")
    lines.append("Press Ctrl+C to quit | View all data in Rerun viewer")
    
    return lines


def run_live_monitor(flare: FlareGrip, args, viz: XGripperVisualizer = None):
    """Run continuous live monitoring with optional Rerun visualization"""
    logger.info("=" * 70)
    if viz:
        logger.info("Starting Live Monitor with Rerun Visualization")
        logger.info("  📊 All data is being logged to Rerun viewer")
    else:
        logger.info("Starting Live Monitor (Rerun visualization disabled)")
    logger.info("=" * 70)
    
    frame_count = 0
    fps = 0.0
    fps_update_interval = 10  # Update FPS every N frames
    last_fps_time = time.time()
    
    # Number of lines we print (for cursor movement)
    num_lines = 0
    
    # Timing accumulators
    time_data = 0.0  # Total recv_data() time
    time_rerun = 0.0  # Rerun logging time
    
    # Last timing info for display
    last_timing_info = None
    
    # Debug logging flag
    gripper_debug_logged = False
    
    try:
        while True:
            loop_start = time.time()
            
            # Get data using FlareGrip.recv_data() API
            t0 = time.perf_counter()
            data = flare.recv_data(
                ee_pose=not args.no_vive,
                gripper=not args.no_gripper,
                wrist_img=not args.no_cam,
                sensor=not args.no_sensor,
            )
            t1 = time.perf_counter()
            
            # Accumulate data acquisition time
            time_data += (t1 - t0)
            
            # Debug log gripper status once
            if not gripper_debug_logged and not args.no_gripper:
                if data.get("gripper_position") is not None:
                    logger.info(f"Gripper data received: pos={data['gripper_position']:.2f}, "
                               f"vel={data['gripper_velocity']:.2f}")
                    gripper_debug_logged = True
                elif frame_count == 0:
                    logger.warn("Gripper status is None - may need calibration")
                    gripper_debug_logged = True
            
            # Log to Rerun (only if visualization is enabled)
            if viz:
                t2 = time.perf_counter()
                viz.log_all(data)
                time_rerun += time.perf_counter() - t2
            
            frame_count += 1
            
            # Update FPS and timing stats
            if frame_count % fps_update_interval == 0:
                current_time = time.time()
                fps = fps_update_interval / (current_time - last_fps_time)
                last_fps_time = current_time
                
                # Calculate average timing (in ms)
                avg_data = (time_data / fps_update_interval) * 1000
                avg_rerun = (time_rerun / fps_update_interval) * 1000
                total = avg_data + avg_rerun
                
                # Store timing info for display
                last_timing_info = {
                    "data": avg_data,
                    "rerun": avg_rerun,
                    "total": total,
                }
                
                # Reset accumulators
                time_data = 0.0
                time_rerun = 0.0
            
            # Format and print data to terminal (only if not disabled)
            if not args.no_print and frame_count % args.print_interval == 0:
                lines = format_live_data(flare, data, frame_count, fps, timing=last_timing_info)
                
                # Move cursor up to overwrite previous output
                if num_lines > 0:
                    move_cursor_up(num_lines)
                
                # Print all lines (use stdout for terminal formatting)
                for line in lines:
                    # Clear line and print
                    sys.stdout.write(f"\033[K{line}\n")
                    sys.stdout.flush()
                
                num_lines = len(lines)
            
            # Control loop rate (only if interval is set and we're ahead of schedule)
            elapsed = time.time() - loop_start
            if args.interval > 0:
                sleep_time = max(0, args.interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        logger.info("\nInterrupted by user.")


def main():
    parser = argparse.ArgumentParser(
        description="FlareGrip Example - Live monitoring with Rerun visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python examples/flare_grip_example.py --mac 6ebbc5f53240
  python examples/flare_grip_example.py --mac 6ebbc5f53240 --no-sensor
  python examples/flare_grip_example.py --mac 6ebbc5f53240 --interval 0.1
        """
    )
    parser.add_argument(
        "--mac",
        type=str,
        default="e2b26adbb104",
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
        "--interval",
        type=float,
        default=0.0,
        help="Interval between data reads in seconds (default: 0.0 = no delay, run as fast as possible)",
    )
    parser.add_argument(
        "--print-interval",
        type=int,
        default=1,
        help="Print to terminal every N frames (default: 1 = every frame, set higher to reduce overhead)",
    )
    parser.add_argument(
        "--no-print",
        action="store_true",
        help="Disable terminal printing completely for maximum performance",
    )
    parser.add_argument(
        "--no-rerun",
        action="store_true",
        help="Disable Rerun visualization for maximum performance",
    )
    args = parser.parse_args()

    # Check Rerun availability only if needed
    if not args.no_rerun:
        if not RERUN_AVAILABLE:
            logger.error("Rerun is not installed.")
            logger.error("Install with: pip install rerun-sdk")
            sys.exit(1)

    logger.info("=" * 70)
    if args.no_rerun:
        logger.info("FlareGrip Example - Data Collection Mode")
    else:
        logger.info("FlareGrip Example - Rerun Visualization")
    logger.info("=" * 70)
    logger.info(f"MAC Address: {args.mac}")
    logger.info(f"Vive SN: {args.vive_sn}")
    logger.info(f"Components: gripper={'OFF' if args.no_gripper else 'ON'}, "
                f"sensor={'OFF' if args.no_sensor else 'ON'}, "
                f"vive={'OFF' if args.no_vive else 'ON'}, "
                f"cam={'OFF' if args.no_cam else 'ON'}")
    logger.info(f"Visualization: rerun={'OFF' if args.no_rerun else 'ON'}")
    logger.info("=" * 70)

    # Initialize Rerun visualizer (only if not disabled)
    viz = None
    if not args.no_rerun:
        logger.info("Initializing Rerun visualizer...")
        viz = XGripperVisualizer(session_name="flare_grip")
        if not viz.init():
            logger.error("Failed to initialize Rerun visualizer")
            sys.exit(1)
        logger.info("Rerun visualizer initialized! Viewer window should open.")

    # Initialize FlareGrip
    logger.info("Initializing FlareGrip...")
    flare = FlareGrip(
        mac_addr=args.mac,
        cam_size=(640, 480),
        log_level="INFO",
        no_gripper=args.no_gripper,
        no_sensor=args.no_sensor,
        no_vive=args.no_vive,
        no_cam=args.no_cam,
    )
    logger.info("FlareGrip initialized successfully!")

    # Register button callback
    # Valid event types: CLICK, DOUBLE_CLICK, LONG_PRESS, PRESS, RELEASE
    if not args.no_gripper:
        flare.register_button_callback("PRESS", button_callback)
        flare.register_button_callback("RELEASE", button_callback)
        logger.info("Button callbacks registered.")

    try:
        # Run continuous live monitoring with Rerun
        run_live_monitor(flare, args, viz=viz)

    except KeyboardInterrupt:
        logger.info("\nInterrupted by user.")
    finally:
        logger.info("Closing FlareGrip...")
        flare.close()
        logger.info("Done.")


if __name__ == "__main__":
    main()
