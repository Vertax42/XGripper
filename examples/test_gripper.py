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
Gripper Test Example - Test and control gripper via XenseSerialGripper

Usage:
    python examples/test_gripper.py --port /dev/ttyUSB0
    python examples/test_gripper.py --port /dev/ttyUSB0 --benchmark
    python examples/test_gripper.py --port /dev/ttyUSB0 --no-interactive
    python examples/test_gripper.py --port /dev/ttyUSB0 --no-callback
"""

import sys
import time
import argparse
import signal
import threading
from collections import defaultdict

from xensegripper import XenseSerialGripper
from utils.spdlogger import get_logger

logger = get_logger("test_gripper")

# Global flag for graceful shutdown
running = True
current_command = None

# Button event tracking
button_events = defaultdict(int)
last_button_event = None
last_button_event_time = 0.0


def make_button_callback(event_type: str):
    """Create a callback function for a specific button event type"""
    def callback():
        global last_button_event, last_button_event_time
        button_events[event_type] += 1
        last_button_event = event_type
        last_button_event_time = time.time()
        total = sum(button_events.values())
        sys.stderr.write(f"\r\033[K Button: {event_type} (total: {total})\n")
        sys.stderr.flush()
    return callback


def signal_handler(sig, frame):
    global running
    print("\n\nReceived Ctrl+C, stopping...")
    running = False


GRIPPER_MAX_POS = 85.0


def print_gripper_data(data, frame_count, fps, timing=None, stats=None, args=None):
    print(f"\n{'='*70}")
    print(f"  Gripper Test | Frame: {frame_count:6d} | FPS: {fps:5.1f}")
    print(f"{'='*70}")

    if timing:
        print("")
        print("┌─ Latency ──────────────────────────────────────────────────────────")
        print(f"│  Read: {timing.get('read', 0):.2f}ms  |  Total loop: {timing.get('total', 0):.2f}ms")
        print("└" + "─" * 69)

    print("")
    print("┌─ Gripper ──────────────────────────────────────────────────────────")

    pos = data.get("position")
    vel = data.get("velocity")
    force = data.get("force")
    temp = data.get("temperature")

    if pos is not None:
        bar_width = 20
        filled = int(max(0, min(GRIPPER_MAX_POS, pos)) / GRIPPER_MAX_POS * bar_width)
        bar = "#" * filled + "." * (bar_width - filled)
        print(f"│  Position:    [{bar}] {pos:6.2f} mm")
        print(f"│  Velocity:    {vel:+8.2f} mm/s")
        print(f"│  Force:       {force:+8.2f} N")
        print(f"│  Temperature: {temp:6.1f} C")
        if stats:
            print(f"│  Range: Pos [{stats['min_pos']:5.1f} - {stats['max_pos']:5.1f}]  Vel [{stats['min_vel']:+7.2f} - {stats['max_vel']:+7.2f}]")
    else:
        print("│  (No gripper data - may need calibration)")

    print("└" + "─" * 69)

    show_callback = args is None or not getattr(args, 'no_callback', False)
    if show_callback:
        print("")
        print("┌─ Button Events ─────────────────────────────────────────────────────")
        total_events = sum(button_events.values())
        if total_events > 0:
            if last_button_event and (time.time() - last_button_event_time) < 2.0:
                print(f"│  Latest: {last_button_event}")
            events_str = "  ".join([f"{k}: {v}" for k, v in sorted(button_events.items())])
            print(f"│  {events_str}  (Total: {total_events})")
        else:
            print("│  (Press gripper button to test)")
        print("└" + "─" * 69)

    show_interactive = args is None or not getattr(args, 'no_interactive', False)
    if show_interactive:
        print("")
        print("┌─ Commands ──────────────────────────────────────────────────────────")
        print("│  [o]pen  [c]lose  [s]top  [0-85] move to position  [q]uit")
        print("└" + "─" * 69)

    print(f"\n{'='*70}")
    print("  Press Ctrl+C to stop")
    print(f"{'='*70}\n")


def keyboard_listener(gripper):
    global running, current_command
    logger.info("Keyboard commands: [o]pen, [c]lose, [s]top, [0-85] position, [q]uit")
    while running:
        try:
            cmd = input().strip().lower()
            if cmd in ['o', 'open']:
                gripper.set_position(85, vmax=40, fmax=27)
            elif cmd in ['c', 'close']:
                gripper.set_position(0, vmax=40, fmax=27)
            elif cmd in ['s', 'stop']:
                gripper.set_speed(0)
            elif cmd.lstrip('-').isdigit():
                pos = max(0, min(85, int(cmd)))
                gripper.set_position(pos, vmax=80, fmax=27)
            elif cmd in ['q', 'quit']:
                running = False
        except EOFError:
            break


def run_benchmark_mode(gripper):
    logger.info("Running benchmark (5 seconds)...")
    frame_count = 0
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < 5.0:
        gripper.get_gripper_status()
        frame_count += 1
    elapsed = time.perf_counter() - start_time
    fps = frame_count / elapsed
    avg_ms = (elapsed / frame_count) * 1000
    logger.info("=" * 60)
    logger.info(f"  Total frames:     {frame_count}")
    logger.info(f"  Elapsed time:     {elapsed:.2f}s")
    logger.info(f"  Average FPS:      {fps:.1f}")
    logger.info(f"  Average read:     {avg_ms:.3f}ms")
    logger.info("=" * 60)


def run_continuous_mode(gripper, args):
    global running
    frame_count = 0
    last_fps_time = time.time()
    fps_update_interval = 10
    time_read = 0.0
    time_total = 0.0
    fps = 0.0
    min_pos, max_pos = float('inf'), float('-inf')
    min_vel, max_vel = float('inf'), float('-inf')
    last_timing_info = None
    sleep_time = 1.0 / args.rate if not args.benchmark else 0

    time.sleep(0.5)

    while running:
        loop_start = time.perf_counter()

        t0 = time.perf_counter()
        data = gripper.get_gripper_status()
        t1 = time.perf_counter()
        time_read += (t1 - t0)

        frame_count += 1

        if data:
            pos = data.get("position")
            vel = data.get("velocity")
            if pos is not None:
                min_pos = min(min_pos, pos)
                max_pos = max(max_pos, pos)
            if vel is not None:
                min_vel = min(min_vel, vel)
                max_vel = max(max_vel, vel)

        loop_end = time.perf_counter()
        time_total += (loop_end - loop_start)

        if frame_count % fps_update_interval == 0:
            current_time = time.time()
            fps = fps_update_interval / (current_time - last_fps_time)
            last_fps_time = current_time
            last_timing_info = {
                "read": (time_read / fps_update_interval) * 1000,
                "total": (time_total / fps_update_interval) * 1000,
            }
            time_read = 0.0
            time_total = 0.0

        print("\033[2J\033[H", end="")
        stats = {
            "min_pos": min_pos if min_pos != float('inf') else 0,
            "max_pos": max_pos if max_pos != float('-inf') else 0,
            "min_vel": min_vel if min_vel != float('inf') else 0,
            "max_vel": max_vel if max_vel != float('-inf') else 0,
        }
        print_gripper_data(data or {}, frame_count, fps, last_timing_info, stats, args)

        if not args.benchmark and sleep_time > 0:
            elapsed = time.perf_counter() - loop_start
            remaining = sleep_time - elapsed
            if remaining > 0:
                time.sleep(remaining)

    if not args.no_callback and button_events:
        logger.info("Button Event Summary:")
        for k, v in sorted(button_events.items()):
            logger.info(f"  {k}: {v}")
        logger.info(f"  Total: {sum(button_events.values())}")


def main():
    parser = argparse.ArgumentParser(
        description="Test and control gripper via XenseSerialGripper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python examples/test_gripper.py --port /dev/ttyUSB0
    python examples/test_gripper.py --port /dev/ttyUSB0 --rate 60
    python examples/test_gripper.py --port /dev/ttyUSB0 --benchmark
    python examples/test_gripper.py --port /dev/ttyUSB0 --no-interactive
    python examples/test_gripper.py --port /dev/ttyUSB0 --no-callback

Interactive commands:
    o/open   - Open gripper (85 mm)
    c/close  - Close gripper (0 mm)
    s/stop   - Stop (speed = 0)
    0-85     - Move to position in mm
    q/quit   - Exit
        """
    )
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="Serial port path (default: /dev/ttyUSB0)")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="Target update rate in Hz (default: 30)")
    parser.add_argument("--benchmark", action="store_true",
                        help="Run benchmark mode (maximum speed test)")
    parser.add_argument("--no-interactive", action="store_true", dest="no_interactive",
                        help="Disable interactive keyboard controls")
    parser.add_argument("--no-callback", action="store_true", dest="no_callback",
                        help="Disable button callback testing")
    parser.add_argument("--calibrate", action="store_true",
                        help="Calibrate gripper before testing")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    logger.info("=" * 70)
    logger.info("Gripper Test - XenseSerialGripper")
    logger.info("=" * 70)
    logger.info(f"Serial port: {args.port}")

    logger.info("Initializing gripper...")
    gripper = XenseSerialGripper(args.port)
    logger.info("Gripper initialized!")

    if not args.no_callback:
        logger.info("Registering button callbacks...")
        for event_type in ["PRESS", "RELEASE", "CLICK", "DOUBLE_CLICK", "LONG_PRESS"]:
            gripper.register_button_callback(event_type, make_button_callback(event_type))
        logger.info("Button callbacks registered. Press the gripper button to test!")

    if args.calibrate:
        logger.info("Calibrating gripper...")
        gripper.calibrate()
        logger.info("Calibration complete!")
        time.sleep(1)

    logger.info("Waiting for gripper data...")
    for _ in range(20):
        data = gripper.get_gripper_status()
        if data is not None:
            logger.info(f"Gripper ready! Position: {data['position']:.2f} mm")
            break
        time.sleep(0.5)
    else:
        logger.warn("Gripper data not available after 10 seconds. Try calibration.")

    if not args.no_interactive:
        kb_thread = threading.Thread(target=keyboard_listener, args=(gripper,), daemon=True)
        kb_thread.start()

    logger.info("Starting... Press Ctrl+C to stop")
    logger.info("=" * 70)

    try:
        if args.benchmark and args.no_interactive:
            run_benchmark_mode(gripper)
        else:
            run_continuous_mode(gripper, args)
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        logger.info("Releasing gripper...")
        gripper.release()
        logger.info("Done!")


if __name__ == "__main__":
    main()
