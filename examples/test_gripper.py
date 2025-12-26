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
Gripper Test Example - Test and control gripper via FlareGrip

Usage:
    python examples/test_gripper.py --mac 6ebbc5f53240
    python examples/test_gripper.py --mac 6ebbc5f53240 --benchmark
    python examples/test_gripper.py --mac 6ebbc5f53240 --interactive
    python examples/test_gripper.py --mac 6ebbc5f53240 --callback-test
"""

import sys
import time
import argparse
import signal
import threading
from collections import defaultdict

from xesne_gripper import FlareGrip
from utils.spdlogger import get_logger

logger = get_logger("test_gripper")

# Global flag for graceful shutdown
running = True
current_command = None

# Button event tracking
button_events = defaultdict(int)
last_button_event = None
last_button_event_time = 0.0


def button_callback(event):
    """Callback function for gripper button events"""
    global last_button_event, last_button_event_time
    event_type = str(event)
    button_events[event_type] += 1
    last_button_event = event_type
    last_button_event_time = time.time()
    # Note: Don't log here as it interferes with screen clearing display
    # The event info is shown in the formatted display instead


def signal_handler(sig, frame):
    """Handle Ctrl+C for graceful shutdown"""
    global running
    print("\n\nReceived Ctrl+C, stopping...")
    running = False


GRIPPER_MAX_POS = 85.0


def print_gripper_data(
    data: dict,
    frame_count: int,
    fps: float,
    timing: dict = None,
    stats: dict = None,
    args = None,
):
    """
    Print gripper data in a formatted table (similar to vive_print_example.py).
    
    Args:
        data: Gripper data dictionary
        frame_count: Current frame count
        fps: Current FPS
        timing: Dictionary with timing info
        stats: Dictionary with min/max stats
        args: Command line arguments
    """
    # Header
    print(f"\n{'='*70}")
    print(f"  🤖 Gripper Test | Frame: {frame_count:6d} | FPS: {fps:5.1f}")
    print(f"{'='*70}")
    
    # Timing Section
    if timing:
        print("")
        print("┌─ ⏱️  Latency ──────────────────────────────────────────────────────")
        print(f"│  Read: {timing.get('read', 0):.2f}ms  |  Total loop: {timing.get('total', 0):.2f}ms")
        print("└" + "─" * 69)
    
    # Gripper Section
    print("")
    print("┌─ 🤖 Gripper ───────────────────────────────────────────────────────")
    
    pos = data.get("gripper_position")
    vel = data.get("gripper_velocity")
    
    if pos is not None:
        # Create visual bar for position
        bar_width = 20
        filled = int(max(0, min(GRIPPER_MAX_POS, pos)) / GRIPPER_MAX_POS * bar_width)
        bar = "█" * filled + "░" * (bar_width - filled)
        
        print(f"│  Position: [{bar}] {pos:6.2f}")
        print(f"│  Velocity: {vel:+8.2f}")
        
        # Show range if we have stats
        if stats:
            min_pos = stats.get("min_pos", pos)
            max_pos = stats.get("max_pos", pos)
            min_vel = stats.get("min_vel", vel)
            max_vel = stats.get("max_vel", vel)
            print(f"│  Range:    Pos [{min_pos:5.1f} - {max_pos:5.1f}]  |  Vel [{min_vel:+7.2f} - {max_vel:+7.2f}]")
    else:
        print("│  (No gripper data - may need calibration)")
    
    print("└" + "─" * 69)
    
    # Button Events Section (if callback test is enabled)
    if args and not args.no_callback:
        print("")
        print("┌─ 🔘 Button Events ─────────────────────────────────────────────────")
        
        total_events = sum(button_events.values())
        current_time = time.time()
        
        if total_events > 0:
            # Show recent event if within 2 seconds
            if last_button_event and (current_time - last_button_event_time) < 2.0:
                print(f"│  Latest: {last_button_event}")
            
            # Show event counts
            events_str = "  ".join([f"{k}: {v}" for k, v in sorted(button_events.items())])
            print(f"│  {events_str}")
            print(f"│  Total: {total_events}")
        else:
            print("│  (Press gripper button to test)")
        
        print("└" + "─" * 69)
    
    # Interactive Mode Section
    if args and not args.no_interactive:
        print("")
        print("┌─ ⌨️  Commands ─────────────────────────────────────────────────────")
        print("│  [o]pen  [c]lose  [s]top  [0-85] move to position  [q]uit")
        print("└" + "─" * 69)
    
    # Footer
    print("")
    print(f"{'='*70}")
    print("  Press Ctrl+C to stop")
    print(f"{'='*70}\n")


def keyboard_listener():
    """Listen for keyboard commands in interactive mode"""
    global running, current_command
    
    logger.info("Keyboard commands: [o]pen, [c]lose, [s]top, [q]uit")
    
    while running:
        try:
            cmd = input().strip().lower()
            if cmd in ['o', 'open']:
                current_command = 'open'
            elif cmd in ['c', 'close']:
                current_command = 'close'
            elif cmd in ['s', 'stop']:
                current_command = 'stop'
            elif cmd in ['q', 'quit']:
                running = False
            elif cmd.isdigit():
                current_command = ('position', int(cmd))
        except EOFError:
            break


def run_continuous_mode(flare, args):
    """Run continuous gripper reading mode"""
    global running, current_command, last_button_event, last_button_event_time
    
    # Performance tracking
    frame_count = 0
    last_fps_time = time.time()
    fps_update_interval = 10
    time_gripper_read = 0.0
    time_total = 0.0
    fps = 0.0
    
    # Stats
    min_pos, max_pos = float('inf'), float('-inf')
    min_vel, max_vel = float('inf'), float('-inf')
    
    # Last timing info for display
    last_timing_info = None
    
    sleep_time = 1.0 / args.rate if not args.benchmark else 0
    
    # Give user time to read init messages
    time.sleep(0.5)

    while running:
        loop_start = time.perf_counter()

        # Process commands (interactive mode)
        if not args.no_interactive and current_command:
            cmd = current_command
            current_command = None
            
            if cmd == 'open':
                flare.open_gripper()
            elif cmd == 'close':
                flare.close_gripper()
            elif cmd == 'stop':
                flare.stop_gripper()
            elif isinstance(cmd, tuple) and cmd[0] == 'position':
                pos = cmd[1]
                flare.move_gripper(pos)

        # Read gripper data
        t0 = time.perf_counter()
        data = flare.recv_data(gripper=True, ee_pose=False, wrist_img=False, sensor=False)
        t1 = time.perf_counter()
        time_gripper_read += (t1 - t0)

        frame_count += 1

        # Update stats
        pos = data.get("gripper_position")
        vel = data.get("gripper_velocity")
        
        if pos is not None:
            min_pos = min(min_pos, pos)
            max_pos = max(max_pos, pos)
        if vel is not None:
            min_vel = min(min_vel, vel)
            max_vel = max(max_vel, vel)

        # Calculate total loop time (before sleep)
        loop_end = time.perf_counter()
        time_total += (loop_end - loop_start)

        # Update FPS and timing stats
        if frame_count % fps_update_interval == 0:
            current_time = time.time()
            fps = fps_update_interval / (current_time - last_fps_time)
            last_fps_time = current_time

            # Calculate average timing (in ms)
            avg_read = (time_gripper_read / fps_update_interval) * 1000
            avg_total = (time_total / fps_update_interval) * 1000
            
            last_timing_info = {
                "read": avg_read,
                "total": avg_total,
            }
            
            # Reset accumulators
            time_gripper_read = 0.0
            time_total = 0.0

        # Clear screen and print formatted data
        print("\033[2J\033[H", end="")  # Clear screen and move to top
        
        stats = {
            "min_pos": min_pos if min_pos != float('inf') else 0,
            "max_pos": max_pos if max_pos != float('-inf') else 0,
            "min_vel": min_vel if min_vel != float('inf') else 0,
            "max_vel": max_vel if max_vel != float('-inf') else 0,
        }
        
        print_gripper_data(
            data=data,
            frame_count=frame_count,
            fps=fps,
            timing=last_timing_info,
            stats=stats,
            args=args,
        )

        # Rate limiting (if not benchmark mode)
        if not args.benchmark and sleep_time > 0:
            elapsed = time.perf_counter() - loop_start
            remaining = sleep_time - elapsed
            if remaining > 0:
                time.sleep(remaining)
    
    # Print button event summary if callback test was enabled
    if not args.no_callback and button_events:
        logger.info("=" * 60)
        logger.info("Button Event Summary:")
        for event_type, count in sorted(button_events.items()):
            logger.info(f"  {event_type}: {count}")
        logger.info(f"  Total: {sum(button_events.values())}")
        logger.info("=" * 60)
    
    return frame_count, min_pos, max_pos


def run_benchmark_mode(flare):
    """Run pure benchmark mode - maximum speed test"""
    logger.info("Running benchmark (5 seconds)...")
    
    frame_count = 0
    start_time = time.perf_counter()
    
    while time.perf_counter() - start_time < 5.0:
        data = flare.recv_data(gripper=True, ee_pose=False, wrist_img=False, sensor=False)
        frame_count += 1
    
    elapsed = time.perf_counter() - start_time
    fps = frame_count / elapsed
    avg_time = (elapsed / frame_count) * 1000
    
    logger.info("=" * 60)
    logger.info("Benchmark Results:")
    logger.info(f"  Total frames: {frame_count}")
    logger.info(f"  Elapsed time: {elapsed:.2f}s")
    logger.info(f"  Average FPS: {fps:.1f}")
    logger.info(f"  Average read time: {avg_time:.3f}ms")
    logger.info(f"  Max theoretical FPS: {1000/avg_time:.1f}")
    logger.info("=" * 60)
    
    return fps


def main():
    global running
    
    parser = argparse.ArgumentParser(
        description="Test and control gripper via FlareGrip",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python examples/test_gripper.py --mac 6ebbc5f53240
    python examples/test_gripper.py --mac 6ebbc5f53240 --rate 60
    python examples/test_gripper.py --mac 6ebbc5f53240 --benchmark
    python examples/test_gripper.py --mac 6ebbc5f53240 --no-interactive
    python examples/test_gripper.py --mac 6ebbc5f53240 --no-callback
    
Interactive mode (enabled by default):
    o/open   - Open gripper
    c/close  - Close gripper
    s/stop   - Stop gripper
    0-85     - Move to position (0=closed, 85=open)
    q/quit   - Exit
    
Button callback events (enabled by default):
    PRESS        - Button pressed down
    RELEASE      - Button released
    CLICK        - Single click detected
    DOUBLE_CLICK - Double click detected
    LONG_PRESS   - Long press detected
        """
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
        help="Run benchmark mode (maximum speed test)",
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        dest="no_interactive",
        help="Disable interactive mode (keyboard controls)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Calibrate gripper before testing",
    )
    parser.add_argument(
        "--no-callback",
        action="store_true",
        dest="no_callback",
        help="Disable button callback testing",
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    logger.info("=" * 70)
    logger.info("Gripper Test - FlareGrip")
    logger.info("=" * 70)
    logger.info(f"MAC Address: {args.mac}")
    mode_str = "Benchmark" if args.benchmark else f"Normal ({args.rate} Hz)"
    if not args.no_interactive:
        mode_str += " + Interactive"
    if not args.no_callback:
        mode_str += " + Callback"
    logger.info(f"Mode: {mode_str}")
    logger.info("=" * 70)

    # Initialize FlareGrip (gripper only, disable other components)
    logger.info("Initializing FlareGrip (gripper only)...")
    flare = FlareGrip(
        mac_addr=args.mac,
        log_level="INFO",
        no_gripper=False,
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

    # Register button callbacks (enabled by default)
    if not args.no_callback:
        logger.info("Registering button callbacks...")
        # Available event types: CLICK, DOUBLE_CLICK, LONG_PRESS, PRESS, RELEASE
        event_types = ["PRESS", "RELEASE", "CLICK", "DOUBLE_CLICK", "LONG_PRESS"]
        for event_type in event_types:
            flare.register_button_callback(event_type, button_callback)
        logger.info(f"Button callbacks registered for: {', '.join(event_types)}")
        logger.info("Press the gripper button to test callbacks!")

    # Calibrate if requested
    if args.calibrate:
        logger.info("Calibrating gripper...")
        flare.calibrate_gripper()
        logger.info("Calibration complete!")
        time.sleep(1)

    # Wait for gripper data to be available
    logger.info("Waiting for gripper data...")
    gripper_ready = False
    for i in range(20):
        data = flare.recv_data(gripper=True, ee_pose=False, wrist_img=False, sensor=False)
        if data.get("gripper_position") is not None:
            logger.info(f"Gripper data received!")
            logger.info(f"  Position: {data['gripper_position']:.2f}")
            logger.info(f"  Velocity: {data['gripper_velocity']:.2f}")
            gripper_ready = True
            break
        time.sleep(0.5)

    if not gripper_ready:
        logger.warn("Gripper data not available after 10 seconds")
        logger.warn(f"Try calibration: python examples/calibrate_gripper.py --mac {args.mac}")

    logger.info("=" * 70)
    
    # Start keyboard listener thread for interactive mode (enabled by default)
    if not args.no_interactive:
        kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
        kb_thread.start()
    
    logger.info("Starting... Press Ctrl+C to stop")
    logger.info("=" * 70)

    try:
        if args.benchmark and args.no_interactive:
            # Pure benchmark mode
            run_benchmark_mode(flare)
        else:
            # Continuous reading mode
            run_continuous_mode(flare, args)

    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        logger.info("Closing FlareGrip...")
        flare.close()
        logger.info("Done!")


if __name__ == "__main__":
    main()
