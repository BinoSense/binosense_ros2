# BinoSense ROS2 - Quick Start Guide

## Step 1: System Preparation

### 1. Create required directories

```bash
sudo mkdir -p /usr/Evo_BionicEyes/tmp_path
sudo chmod 777 /usr/Evo_BionicEyes/tmp_path
```

### 2. Ensure ROS2 is installed

ROS2 Humble, Iron, or Jazzy is recommended.

```bash
# Check ROS2 version
echo $ROS_DISTRO
```

## Step 2: Get the Code and SDK

### 1. Clone this driver package

```bash
mkdir -p ~/binosense_ws/src
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/binosense_ros2.git
```

### 2. Get the BinoSense SDK

This driver depends on the [BinoSense SDK](https://github.com/BinoSense/be_sdk). Choose one of the following methods to configure it:

**Method A — Sibling directory (simplest):**

```bash
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/be_sdk.git
```

This directory structure will be automatically recognized by CMake:

```text
binosense_ws/src/
├── binosense_ros2/   # Driver package
└── be_sdk/           # SDK
```

**Method B — Environment variable:**

```bash
# Clone the SDK to any location
git clone https://github.com/BinoSense/be_sdk.git ~/be_sdk

# Set environment variable
export BE_SDK_DIR=~/be_sdk

# Persist the setting (optional)
echo 'export BE_SDK_DIR=$HOME/be_sdk' >> ~/.bashrc
```

**Method C — Build parameter:**

```bash
colcon build --cmake-args -DBE_SDK_DIR=/path/to/be_sdk
```

## Step 3: Build

```bash
cd ~/binosense_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> Verify that CMake outputs the following during build, indicating the SDK path is configured correctly:
>
> ```text
> -- Using BinoSense SDK: /path/to/your/be_sdk
> --   Include: /path/to/your/be_sdk/include
> --   Library: /path/to/your/be_sdk/lib/linux
> ```

## Step 4: Run

### Launch driver and RViz (recommended)

```bash
# Replace <YOUR_DEVICE_IP> with your BinoSense device IP address
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>
```

This will launch:

- BinoSense Active Binocular Vision System device driver
- RViz visualization interface

### Launch driver only

```bash
ros2 run binosense_ros2 binosense_driver
```

### Specify device IP

```bash
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>
```

### List topics

```bash
ros2 topic list
```

You should see:

- `/left/image_raw`
- `/left/camera_info`
- `/right/image_raw`
- `/right/camera_info`
- `/depth/image_raw`
- `/depth/image_color`
- `/depth/points`
- `/motor_state`
- `/binosense_data`
- `/tf`

## Step 5: Control the Device

### Method 1: Command line

```bash
# Return to home position
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Look left
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 15.0, 0.0, 0.0, 15.0]}"

# Look up
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [10.0, 0.0, 0.0, 10.0, 0.0, 0.0]}"
```

### Method 2: Python interactive control script

```bash
ros2 run binosense_ros2 simple_control.py
```

Interactive commands:

- `home` - Return to home position
- `left` - Look left
- `right` - Look right
- `up` - Look up
- `down` - Look down
- `sweep` - Auto sweep demo

### Method 3: C++ example

```bash
# Auto sweep mode
ros2 launch binosense_ros2 demo_control.launch.py mode:=sweep

# Position mode (hold at initial position)
ros2 launch binosense_ros2 demo_control.launch.py mode:=position
```

## Step 6: View Images

### Method 1: Using RViz

After launch, RViz will automatically display the left and right eye images.

### Method 2: Using rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select the `/left/image_raw` or `/right/image_raw` topic in the interface.

## Motor Index

The 6 values sent to the `/motor_command` topic correspond to:

| Index | Motor | Description | Range |
| ------ | ----- | ----------- | ----- |
| 0 | Right Pitch | Vertical rotation | ±30° |
| 1 | Right Roll | Rotation | ±20° |
| 2 | Right Yaw | Horizontal rotation | ±30° |
| 3 | Left Pitch | Vertical rotation | ±30° |
| 4 | Left Roll | Rotation | ±20° |
| 5 | Left Yaw | Horizontal rotation | ±30° |

## FAQ

### Q: Build fails with "BinoSense SDK not found"?

A: The driver package requires the BinoSense SDK to compile. Follow Step 2 Method A/B/C to configure the SDK path. Make sure the SDK directory contains `include/` and `lib/linux/` subdirectories.

### Q: Cannot find the device?

A: Check the following:

1. The BinoSense device is powered on and connected
2. The `/usr/Evo_BionicEyes/tmp_path` directory exists with write permissions
3. Network connectivity is normal (ensure your PC and device are on the same network segment)

### Q: No images are published?

A: Verify:

1. Connection mode is set to "ic" or "i"
2. The device is transmitting data
3. Check terminal for error messages

### Q: Runtime error: libbionic_eyes_c.so not found?

A: Add the SDK library directory to the search path:

```bash
export LD_LIBRARY_PATH=$BE_SDK_DIR/lib/linux:$LD_LIBRARY_PATH
```

Or copy to a system path:

```bash
sudo cp $BE_SDK_DIR/lib/linux/lib*.so /usr/local/lib/
sudo ldconfig
```

## Next Steps

- Read the full [README.md](README.md) for detailed information
- Check the [BinoSense SDK](https://github.com/BinoSense/be_sdk) for the underlying API
- Modify launch file parameters as needed
