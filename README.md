# XGripper
# 0, 0, 0.16, 0.207, 0.676, 0.676, -0.207
XGripper is a Python package for controlling Xense robotic grippers with integrated Vive Tracker support for spatial tracking.

## Installation

### Install libsurvive (Vive Tracking)

```bash
cd ~ && git clone https://github.com/cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt update && sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
sudo make install -j4
```

### Install XGripper Package

```bash
cd ~/XGripper
pip install -e .
```

## Hardware Setup

### Vive Tracking System

1. **Base Stations (Lighthouses)**
   - Mount at least 2 Base Stations in opposite corners
   - Height: ~2m (6.5 ft), angled down 30-45°
   - Base Station 2.0 supports up to 16 channels (set via SteamVR or physical button)

2. **Vive Trackers**
   - Each Tracker needs a paired Dongle (USB receiver)
   - Pair via SteamVR: Menu → Devices → Pair Controller
   - LED colors: 🟢 Green = tracking, 🔵 Blue = pairing mode, 🔴 Red = error

3. **Reference Coordinate System**
   - Default origin: Tracker position during calibration
   - Optional: Use `--center-on-lh0` to set LH0 as origin
   - All poses are relative to the origin
   - Coordinate system: Right-handed
   - Z-axis: Aligned with gravity (up)
   - Rotation reference: Based on LH0 orientation

## Hardware Specifications

| Component | Max Update Rate | Latency | Notes |
|-----------|----------------|---------|-------|
| **Vive Tracker** | 100 Hz | ~0.02ms | Non-blocking read from cache |
| **Tactile Sensor** | ~33 Hz (single) | ~30ms | Blocking call, ~16 Hz for 2 sensors in serial |

## Example Scripts

All examples are in the `examples/` directory.

### Vive Tracker Examples

#### `test_lighthouse.py` - Quick System Check

Test if base stations and trackers are detected correctly.

```bash
python examples/test_lighthouse.py
```

**Output:**
```
Detected Lighthouses: ['LH0', 'LH1']
Trackers: ['WM0', 'WM1']
✅ 检测到两个基站和两个 Tracker！系统就绪。
```

#### `vive_print_example.py` - Print Tracker Pose Data

Print real-time pose data for all detected trackers.

```bash
# Default 10 Hz update rate
python examples/vive_print_example.py

# Custom update rate (5 Hz)
python examples/vive_print_example.py --rate 5
```

**Output:**
```
======================================================================
  Vive Tracker Pose Data (Update #42)
======================================================================
  [WM0]
    Position (m):  X= +0.2086  Y= +0.0734  Z= +3.2577
    Rotation (q):  X= -0.1586  Y= +0.4070  Z= +0.4195  W= +0.7958
======================================================================
```

#### `calibrate_vive.py` - Force Recalibration

Force recalibration of the Vive tracking system with configurable origin mode.

```bash
# Default: Tracker as origin, 60 second calibration
python examples/calibrate_vive.py

# LH0 as origin (recommended for fixed setups)
python examples/calibrate_vive.py --origin lh0

# Tracker as origin (explicit)
python examples/calibrate_vive.py --origin tracker

# Extended calibration with LH0 origin
python examples/calibrate_vive.py --origin lh0 --timeout 120
```

**Origin Modes:**

| Mode | Origin | Coordinate System | Use Case |
|------|--------|------------------|----------|
| `--origin tracker` | Tracker position during calibration | LH0 on +Y axis | Robot-centric applications |
| `--origin lh0` | Lighthouse 0 (LH0) position | LH0 looks in +X direction | Fixed room/scene applications |

**Instructions:**
1. Ensure all base stations are powered on (green LED)
2. Place tracker(s) with clear line of sight to lighthouses
3. Keep tracker(s) stationary during calibration
4. Choose origin mode based on your application needs

#### `raw_survive.py` - Raw pysurvive Access

Direct access to pysurvive for debugging.

```bash
python examples/raw_survive.py
```

### Gripper Examples

#### `calibrate_gripper.py` - Gripper Calibration

Calibrate the gripper position.

```bash
python examples/calibrate_gripper.py --mac YOUR_MAC_ADDRESS
```

#### `flare_grip_example.py` - Full FlareGrip Demo

Comprehensive example showing gripper, camera, sensors, and Vive tracking.

```bash
# Full demo with all components
python examples/flare_grip_example.py --mac YOUR_MAC_ADDRESS

# With specific options
python examples/flare_grip_example.py --mac YOUR_MAC --no-gripper --no-cam --loop

# Available options:
#   --mac         MAC address of FlareGrip device
#   --no-gripper  Disable gripper
#   --no-sensor   Disable sensors
#   --no-vive     Disable Vive tracking
#   --no-cam      Disable camera
#   --loop        Continuously print data
#   --interval    Data print interval (default: 0.5s)
```

### Visualization Examples

#### `rerun_visualization_example.py` - Rerun Visualization

Real-time visualization using Rerun SDK.

```bash
# Install Rerun SDK first
pip install rerun-sdk

# Run visualization
python examples/rerun_visualization_example.py --mac YOUR_MAC_ADDRESS

# Options:
#   --mac         MAC address
#   --no-gripper  Disable gripper
#   --no-camera   Disable camera
#   --no-vive     Disable Vive tracking
#   --hz          Update rate (default: 30)
```

## Calibration Details

### Understanding the Config File

After calibration, the configuration is saved to `~/.config/libsurvive/config.json`.

**Example with `--origin lh0`:**
```json
"lighthouse0": {
    "pose": ["0.000000", "0.000000", "0.000000", ...],  // LH0 at origin (0,0,0)
}
"lighthouse1": {
    "pose": ["-0.200", "-3.260", "-0.126", ...],  // LH1 relative to LH0
}
```

**Example with `--origin tracker` (default):**
```json
"lighthouse0": {
    "pose": ["0.000000", "1.406", "0.683", ...],  // LH0 relative to tracker
}
"lighthouse1": {
    "pose": ["0.103", "-1.854", "0.567", ...],  // LH1 relative to tracker
}
```

### Coordinate System Visualization

```
With --origin lh0:

        +Z (up)
        ↑
        |     LH0 ●────→ +X (LH0 looking direction)
        |      (0,0,0)
        |
        └──────────────→ +Y

With --origin tracker (default):

        +Z (up)
        ↑
        |         ● LH0 (+Y direction)
        |         |
        |    Tracker ● (0,0,0)
        |         |
        |         ● LH1 (-Y direction)
        └──────────────→ +X
```

## Troubleshooting

### No Trackers Detected

1. Check tracker LED is green (not blue/red)
2. Ensure tracker is paired with dongle (via SteamVR)
3. Verify tracker has clear line of sight to base stations
4. Try moving tracker slightly

### Only Lighthouses Detected

Trackers (WM*) only appear after receiving pose data. Wait a few seconds and ensure tracker is visible to base stations.

### Base Station Not Detected

1. Check power connection (LED should be green)
2. For Base Station 2.0, ensure channels don't conflict (use SteamVR to set)
3. Try running `survive-cli` directly to check libsurvive

### USB Device Permissions

If you get permission errors:

```bash
sudo cp ~/libsurvive/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then reconnect USB devices.

## API Reference

### ViveTracker

```python
from vive_tracker import ViveTracker

# Initialize and connect
vt = ViveTracker()
vt.connect()

# Wait for devices
devices = vt.wait_for_devices(timeout=10.0, required_trackers=1)
print(f"Lighthouses: {devices['lighthouses']}")
print(f"Trackers: {devices['trackers']}")

# Get reference frame info
vt.log_reference_frame_info()

# Get pose for specific tracker
pose = vt.get_pose("WM0")
if pose:
    print(f"Position: {pose.position}")
    print(f"Rotation: {pose.rotation}")

# Get all tracker poses
all_poses = vt.get_pose()  # Returns dict

# Helper methods
trackers = vt.get_tracker_devices()    # ['WM0', 'WM1']
lighthouses = vt.get_lighthouse_devices()  # ['LH0', 'LH1']

# Disconnect
vt.disconnect()
```

### FlareGrip

```python
from xesne_gripper import FlareGrip

# Initialize
flare = FlareGrip(
    mac_addr="YOUR_MAC_ADDRESS",
    log_level="INFO",
    no_gripper=False,
    no_sensor=False,
    no_vive=False,
    no_cam=False,
)

# Get data
data = flare.recv_data(ee_pose=True, gripper=True, wrist_img=True)

# Access components
pose = flare.get_pose()  # Vive tracker pose
flare.calibrate_gripper()  # Calibrate gripper

# Clean up
flare.close()
```

## License

Apache License 2.0
