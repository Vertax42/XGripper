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
Vive Tracker Print Example - Print pose data to terminal with optional Rerun visualization
"""

import time
import argparse
import signal
import sys

from vive_tracker import ViveTracker
from utils.spdlogger import get_logger
from utils.visualization import (
    XGripperVisualizer,
    RERUN_AVAILABLE,
    log_vive_pose,
)

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


def print_pose_data(trackers_data: dict, update_count: int, fps: float = 0.0, timing: dict = None):
    """
    Print pose data for all trackers in a formatted table.
    
    Args:
        trackers_data: Dictionary of {device_name: PoseData}
        update_count: Current update count for display
        fps: Current frames per second
        timing: Dictionary with timing info (pose_read_ms, total_ms)
    """
    # Header
    print(f"\n{'='*70}")
    print(f"  Vive Tracker Pose Data (Update #{update_count}) | FPS: {fps:.1f}")
    print(f"{'='*70}")
    
    # Performance section
    if timing:
        rerun_str = f" | Rerun: {timing.get('rerun', 0):.2f}ms" if timing.get('rerun', 0) > 0 else ""
        print(f"\n  ⏱️  Latency: Pose read: {timing.get('pose_read', 0):.2f}ms{rerun_str} | "
              f"Total: {timing.get('total', 0):.2f}ms")
    
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
        print(f"    Rotation (q):  W={rot[0]:+8.4f}  X={rot[1]:+8.4f}  Y={rot[2]:+8.4f}  Z={rot[3]:+8.4f}")
    
    print(f"\n{'='*70}")
    print("  Press Ctrl+C to stop")
    print(f"{'='*70}\n")


def run_terminal_visualization(rate_hz: float = 10.0, use_rerun: bool = False):
    """
    Run terminal visualization for Vive Trackers with optional Rerun visualization.
    
    Args:
        rate_hz: Update rate in Hz (default 10 Hz)
        use_rerun: Whether to enable Rerun visualization
    """
    global running
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize Rerun if requested
    viz = None
    if use_rerun:
        if not RERUN_AVAILABLE:
            logger.error("Rerun is not installed. Install with: pip install rerun-sdk")
            return False
        logger.info("Initializing Rerun visualizer...")
        viz = XGripperVisualizer(session_name="vive_tracker")
        if not viz.init():
            logger.error("Failed to initialize Rerun visualizer")
            return False
        logger.info("Rerun visualizer initialized!")
    
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
    if use_rerun:
        logger.info("Rerun visualization enabled - view in Rerun viewer")
    logger.info("Press Ctrl+C to stop\n")
    
    time.sleep(1.0)  # Give time for user to read
    
    update_count = 0
    sleep_time = 1.0 / rate_hz
    
    # FPS tracking
    fps = 0.0
    fps_update_interval = 10
    last_fps_time = time.time()
    
    # Timing accumulators
    time_pose_read = 0.0
    time_rerun = 0.0
    time_total = 0.0
    
    # Last timing info for display
    last_timing_info = None
    
    try:
        while running:
            loop_start = time.perf_counter()
            
            # Get pose data for all devices (trackers + lighthouses)
            t0 = time.perf_counter()
            all_poses = vt.get_pose()  # Get all poses
            trackers_data = {name: all_poses.get(name) for name in trackers}
            t1 = time.perf_counter()
            time_pose_read += (t1 - t0)
            
            # Log to Rerun if enabled
            if viz and all_poses:
                t2 = time.perf_counter()
                for device_name, pose in all_poses.items():
                    if pose is not None:
                        # Log lighthouse as static, tracker with trajectory
                        if device_name.startswith("LH"):
                            viz._log_lighthouse(device_name, pose.position, pose.rotation)
                        else:
                            log_vive_pose(
                                device_name=device_name,
                                position=pose.position,
                                rotation=pose.rotation,
                            )
                            viz.trajectory_viz.add_point(device_name, pose.position)
                time_rerun += time.perf_counter() - t2
            
            # Clear screen and print data
            print("\033[2J\033[H", end="")  # Clear screen and move to top
            print_pose_data(trackers_data, update_count, fps=fps, timing=last_timing_info)
            
            update_count += 1
            
            # Calculate total loop time (before sleep)
            loop_end = time.perf_counter()
            time_total += (loop_end - loop_start)
            
            # Update FPS and timing stats
            if update_count % fps_update_interval == 0:
                current_time = time.time()
                fps = fps_update_interval / (current_time - last_fps_time)
                last_fps_time = current_time
                
                # Calculate average timing (in ms)
                avg_pose_read = (time_pose_read / fps_update_interval) * 1000
                avg_rerun = (time_rerun / fps_update_interval) * 1000
                avg_total = (time_total / fps_update_interval) * 1000
                
                last_timing_info = {
                    "pose_read": avg_pose_read,
                    "rerun": avg_rerun,
                    "total": avg_total,
                }
                
                # Reset accumulators
                time_pose_read = 0.0
                time_rerun = 0.0
                time_total = 0.0
            
            # Dynamic sleep: only sleep for remaining time to achieve target rate
            elapsed = time.perf_counter() - loop_start
            remaining_sleep = sleep_time - elapsed
            if remaining_sleep > 0:
                time.sleep(remaining_sleep)
            
    except Exception as e:
        logger.error(f"Error during visualization: {e}")
    finally:
        logger.info("Disconnecting Vive Tracker...")
        vt.disconnect()
        logger.info("Done!")
    
    return True


def pose_changed(pose1, pose2, pos_threshold=1e-6, rot_threshold=1e-6):
    """Check if pose has changed beyond threshold."""
    if pose1 is None or pose2 is None:
        return pose1 is not pose2
    
    # Check position change
    for i in range(3):
        if abs(pose1.position[i] - pose2.position[i]) > pos_threshold:
            return True
    
    # Check rotation change
    for i in range(4):
        if abs(pose1.rotation[i] - pose2.rotation[i]) > rot_threshold:
            return True
    
    return False


def run_benchmark():
    """
    Run benchmark to test actual Vive Tracker data update rate.
    Measures how often the pose data actually changes (not just read speed).
    """
    global running
    
    signal.signal(signal.SIGINT, signal_handler)
    
    logger.info("=" * 70)
    logger.info("Vive Tracker Benchmark - Testing actual data update rate")
    logger.info("=" * 70)
    
    vt = ViveTracker()
    
    if not vt.connect():
        logger.error("Failed to connect to Vive Tracker")
        return False
    
    devices = vt.wait_for_devices(timeout=10.0, required_trackers=1)
    trackers = devices["trackers"]
    
    if not trackers:
        logger.error("No trackers detected!")
        vt.disconnect()
        return False
    
    logger.info(f"Found {len(trackers)} tracker(s): {trackers}")
    logger.info("Starting benchmark... Press Ctrl+C to stop")
    logger.info("=" * 70)
    
    # Tracking stats
    read_count = 0  # Total reads
    update_count = {t: 0 for t in trackers}  # Actual data updates per tracker
    last_pose = {t: None for t in trackers}  # Last pose for each tracker
    
    # Time tracking
    start_time = time.time()
    last_report_time = start_time
    report_interval = 1.0  # Report every 1 second
    
    # Timing accumulator
    time_pose_read = 0.0
    reads_since_report = 0
    
    try:
        while running:
            t0 = time.perf_counter()
            
            # Read all trackers and check for updates
            for tracker_name in trackers:
                pose = vt.get_pose(tracker_name)
                
                # Check if data actually changed
                if pose_changed(last_pose[tracker_name], pose):
                    update_count[tracker_name] += 1
                    last_pose[tracker_name] = pose
            
            t1 = time.perf_counter()
            time_pose_read += (t1 - t0)
            
            read_count += 1
            reads_since_report += 1
            
            # Report stats every interval
            current_time = time.time()
            if current_time - last_report_time >= report_interval:
                elapsed = current_time - last_report_time
                
                # Calculate rates
                read_rate = reads_since_report / elapsed
                avg_read_time = (time_pose_read / reads_since_report) * 1000 if reads_since_report > 0 else 0
                
                # Build update rate string for each tracker
                update_rates = []
                for tracker_name in trackers:
                    updates = update_count[tracker_name]
                    update_rate = updates / (current_time - start_time)
                    update_rates.append(f"{tracker_name}: {update_rate:.1f}Hz")
                    
                logger.info(
                    f"Read rate: {read_rate:.0f}/s | "
                    f"Read time: {avg_read_time:.3f}ms | "
                    f"Data update: {' | '.join(update_rates)}"
                )
                
                # Reset for next interval
                last_report_time = current_time
                time_pose_read = 0.0
                reads_since_report = 0
                
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        # Final summary
        total_time = time.time() - start_time
        logger.info("=" * 70)
        logger.info("Summary:")
        logger.info(f"  Total time: {total_time:.1f}s")
        logger.info(f"  Total reads: {read_count}")
        for tracker_name in trackers:
            updates = update_count[tracker_name]
            rate = updates / total_time if total_time > 0 else 0
            logger.info(f"  {tracker_name}: {updates} updates ({rate:.1f} Hz)")
        logger.info("=" * 70)
        
        logger.info("Disconnecting...")
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
        default=100.0,
        help="Update rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Enable Rerun visualization for 3D pose display",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Run benchmark mode to test maximum throughput (no rate limiting)",
    )
    args = parser.parse_args()
    
    if args.benchmark:
        run_benchmark()
    else:
        run_terminal_visualization(rate_hz=args.rate, use_rerun=args.rerun)


if __name__ == "__main__":
    main()
