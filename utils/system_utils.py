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
System utilities for XGripper.
Includes device discovery and system information functions.
"""

import subprocess
import re
from typing import Optional

from .spdlogger import get_logger

logger = get_logger("xgripper.system")

__all__ = [
    "discover_devices",
    "run_ezros",
    "parse_ezros",
]


def run_ezros(args: list[str] | None = None, timeout: float = 15.0, show_progress: bool = True) -> tuple[bool, str]:
    """
    Run ezros command and return output.
    
    Args:
        args: Command line arguments for ezros (e.g., ["-a"])
        timeout: Command timeout in seconds
        show_progress: If True, show a simple progress indicator
        
    Returns:
        tuple: (success: bool, output: str)
    """
    import sys
    import time
    import threading
    
    cmd = ["ezros"]
    if args:
        cmd.extend(args)
    
    try:
        if show_progress:
            # Show our own progress animation while running ezros silently
            stop_progress = threading.Event()
            
            def progress_animation():
                dots = 0
                while not stop_progress.is_set():
                    # Animated dots: . -> .. -> ... -> .
                    dots = (dots % 3) + 1
                    dot_str = "." * dots + " " * (3 - dots)
                    sys.stdout.write(f"\rScanning network{dot_str}")
                    sys.stdout.flush()
                    time.sleep(0.4)
                
                # Show complete
                sys.stdout.write(f"\rScanning network... ✓\n")
                sys.stdout.flush()
            
            # Start progress animation in background
            progress_thread = threading.Thread(target=progress_animation, daemon=True)
            progress_thread.start()
            
            # Run ezros silently and capture output
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            # Stop progress animation
            stop_progress.set()
            progress_thread.join(timeout=1)
            
            if result.returncode == 0:
                return True, result.stdout
            else:
                return False, result.stderr if result.stderr else "Command failed"
        else:
            # Capture both stdout and stderr quietly
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            if result.returncode == 0:
                return True, result.stdout
            else:
                return False, result.stderr if result.stderr else "Command failed"
            
    except FileNotFoundError:
        logger.error("ezros command not found. Make sure xensesdk is installed.")
        return False, "ezros command not found"
    except subprocess.TimeoutExpired:
        logger.error(f"ezros command timed out after {timeout}s")
        return False, "Command timed out"
    except Exception as e:
        logger.error(f"Error running ezros: {e}")
        return False, str(e)


def parse_ezros(output: str) -> dict:
    """
    Parse ezros -a output to extract device information.
    
    Args:
        output: Raw output from ezros -a command
        
    Returns:
        dict: Parsed device information with hierarchical structure
        {
            "devices": {
                "e2b26adbb104": {
                    "mac": "e2b26adbb104",
                    "master_node": "master_e2b26adbb104",
                    "gripper_node": "gripper_e2b26adbb104",
                    "cameras": ["Cam-e2b26adbb104-0"],
                    "sensors": ["OG000450", "OG000657"],
                },
            },
            "all_nodes": {...}  # flat node info for reference
        }
    """
    result = {
        "devices": {},
        "all_nodes": {},
    }
    
    # First pass: find all MAC addresses from master nodes
    master_pattern = r"master_([a-f0-9]{12})"
    macs = list(set(re.findall(master_pattern, output, re.IGNORECASE)))
    
    # Initialize device structures
    for mac in macs:
        result["devices"][mac] = {
            "mac": mac,
            "master_node": f"master_{mac}",
            "gripper_node": f"gripper_{mac}",
            "cameras": [],
            "sensors": [],
        }
    
    # Parse camera nodes and assign to devices (Cam-XXXXXXXXXXXX-N)
    camera_pattern = r"(Cam-([a-f0-9]{12})-\d+)"
    for match in re.finditer(camera_pattern, output, re.IGNORECASE):
        cam_name = match.group(1)
        cam_mac = match.group(2).lower()
        if cam_mac in result["devices"]:
            if cam_name not in result["devices"][cam_mac]["cameras"]:
                result["devices"][cam_mac]["cameras"].append(cam_name)
    
    # Parse sensor serial numbers and assign to devices
    # Sensors (OG followed by digits) - we need to find which device they belong to
    # Looking at ezros output format, sensors appear after their master node
    sensor_pattern = r"\b(OG\d{6})\b"
    sensors = list(set(re.findall(sensor_pattern, output)))
    
    # For now, assign sensors to the first (or only) device
    # In multi-device scenarios, we might need smarter parsing
    if macs and sensors:
        # Try to determine sensor ownership from output context
        # If only one device, all sensors belong to it
        if len(macs) == 1:
            result["devices"][macs[0]]["sensors"] = sensors
        else:
            # Multiple devices - try to parse context
            # Split output by device sections and match sensors
            for mac in macs:
                device_section_pattern = rf"master_{mac}.*?(?=master_[a-f0-9]{{12}}|$)"
                device_section = re.search(device_section_pattern, output, re.IGNORECASE | re.DOTALL)
                if device_section:
                    section_text = device_section.group(0)
                    device_sensors = re.findall(sensor_pattern, section_text)
                    result["devices"][mac]["sensors"] = list(set(device_sensors))
    
    # Parse all nodes for reference
    node_pattern = r"Node:\s*(\S+)\s*\(([^)]+)\)"
    for match in re.finditer(node_pattern, output):
        node_name = match.group(1)
        node_addr = match.group(2)
        
        node_info = {
            "name": node_name,
            "address": node_addr,
            "type": "unknown",
            "mac": None,
        }
        
        # Determine node type
        if node_name.startswith("master_"):
            node_info["type"] = "master"
            node_info["mac"] = node_name.replace("master_", "")
        elif node_name.startswith("gripper_"):
            node_info["type"] = "gripper"
            node_info["mac"] = node_name.replace("gripper_", "")
        elif node_name.startswith("Cam-"):
            node_info["type"] = "camera"
            cam_mac_match = re.search(r"Cam-([a-f0-9]{12})", node_name, re.IGNORECASE)
            if cam_mac_match:
                node_info["mac"] = cam_mac_match.group(1).lower()
        elif re.match(r"OG\d{6}", node_name):
            node_info["type"] = "sensor"
            node_info["sn"] = node_name
        
        result["all_nodes"][node_name] = node_info
    
    return result


def discover_devices(verbose: bool = True, show_progress: bool = True) -> list[str]:
    """
    Discover all XGripper devices on the network using ezros.
    
    Args:
        verbose: If True, print discovery results
        show_progress: If True, show ezros progress bar in terminal
        
    Returns:
        list: List of MAC addresses of discovered devices
    """
    success, output = run_ezros(["-a"], show_progress=show_progress)
    
    if not success:
        if verbose:
            logger.error(f"Device discovery failed: {output}")
        return []
    
    parsed = parse_ezros(output)
    devices = parsed["devices"]
    macs = list(devices.keys())
    
    if verbose:
        if macs:
            logger.info(f"Found {len(macs)} device(s):")
            for mac, device_info in devices.items():
                logger.info(f"  - {mac}")
                sensors = device_info.get("sensors", [])
                cameras = device_info.get("cameras", [])
                if sensors:
                    logger.info(f"      Sensors: {sensors}")
                if cameras:
                    logger.info(f"      Cameras: {cameras}")
        else:
            logger.warn("No devices found")
    
    return macs


def discover_all(verbose: bool = True, show_progress: bool = True) -> dict:
    """
    Discover all XGripper devices and return complete information.
    
    Args:
        verbose: If True, print discovery summary
        show_progress: If True, show ezros progress bar in terminal
        
    Returns:
        dict: Complete discovery information with hierarchical device structure
        {
            "success": True,
            "devices": {
                "e2b26adbb104": {
                    "mac": "e2b26adbb104",
                    "master_node": "master_e2b26adbb104",
                    "gripper_node": "gripper_e2b26adbb104",
                    "cameras": ["Cam-e2b26adbb104-0"],
                    "sensors": ["OG000450", "OG000657"],
                },
            },
            "all_nodes": {...}
        }
    """
    success, output = run_ezros(["-a"], show_progress=show_progress)
    
    if not success:
        if verbose:
            logger.error(f"Discovery failed: {output}")
        return {
            "success": False,
            "error": output,
            "devices": {},
            "all_nodes": {},
        }
    
    parsed = parse_ezros(output)
    parsed["success"] = True
    parsed["raw_output"] = output
    
    if verbose:
        devices = parsed["devices"]
        total_sensors = sum(len(d.get("sensors", [])) for d in devices.values())
        total_cameras = sum(len(d.get("cameras", [])) for d in devices.values())
        logger.info(f"Found {len(devices)} device(s), {total_sensors} sensor(s), {total_cameras} camera(s)")
    
    return parsed


def print_discovered_devices(show_progress: bool = True):
    """
    Discover and print all devices in a formatted way.
    
    Args:
        show_progress: If True, show ezros progress bar during scanning
    """
    info = discover_all(verbose=False, show_progress=show_progress)
    
    print()
    print("=" * 70)
    print("                 XGripper Device Discovery")
    print("=" * 70)
    
    if not info.get("success"):
        print(f"  ❌ Discovery failed: {info.get('error', 'Unknown error')}")
        print("=" * 70)
        return info
    
    devices = info.get("devices", {})
    
    if not devices:
        print("  No devices found on the network.")
        print()
        print("  Please check:")
        print("    - XGripper device is powered on")
        print("    - Network connection is working")
        print("    - xensesdk is properly installed")
    else:
        print(f"  Found {len(devices)} device(s):")
        print()
        
        for mac, device_info in devices.items():
            cameras = device_info.get("cameras", [])
            sensors = device_info.get("sensors", [])
            
            print(f"  📱 Device: {mac}")
            print(f"      ├── 🟢 Master Node: {device_info.get('master_node', f'master_{mac}')}")
            print(f"      ├──    Gripper Node: {device_info.get('gripper_node', f'gripper_{mac}')}")
            
            # Cameras
            if cameras:
                print(f"      ├── 📷 Cameras ({len(cameras)}):")
                for i, cam in enumerate(cameras):
                    prefix = "│" if sensors else " "
                    print(f"      {prefix}       └── {cam}")
            else:
                prefix = "├" if sensors else "└"
                print(f"      {prefix}── 📷 Cameras: (none)")
            
            # Sensors
            if sensors:
                print(f"      └── 🦖 Sensors ({len(sensors)}):")
                for i, sn in enumerate(sensors):
                    if i == len(sensors) - 1:
                        print(f"              └── {sn}")
                    else:
                        print(f"              ├── {sn}")
            else:
                print(f"      └── 🦖 Sensors: (none)")
            
            print()
        
    print("=" * 70)
    
    return info

