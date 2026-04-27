# BinoSense ROS2 Driver

ROS2 driver for the BinoSense Active Binocular Vision System, providing image streaming, motor control, depth perception, and more.

## Features

- Image streaming and display (left/right images, stitched image)
- 6-axis motor control
- Depth map and point cloud display
- Camera parameter control (exposure, white balance, gain)
- VOR/SV function control
- Data recording and saving (snap/record/save)
- Neck motor control
- Extra module control (wiper, zoom, illumination, etc.)
- TF coordinate transform broadcasting
- RViz visualization support

## Prerequisites

- ROS2 (Humble/Iron/Jazzy)
- OpenCV 4.x
- CMake 3.16+
- [BinoSense SDK](https://github.com/BinoSense/be_sdk) (obtained separately)

## Installation and Build

### Step 1: Clone this package

```bash
mkdir -p ~/binosense_ws/src
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/binosense_ros2.git
```

### Step 2: Configure SDK dependency

This driver depends on the headers and shared libraries from the [BinoSense SDK](https://github.com/BinoSense/be_sdk). There are three ways to make the SDK discoverable by the build system:

#### Method 1: Environment variable (recommended)

```bash
# Clone the SDK to any location
git clone https://github.com/BinoSense/be_sdk.git ~/be_sdk

# Set environment variable
export BE_SDK_DIR=~/be_sdk
```

To persist the setting, add the `export` command to `~/.bashrc`:

```bash
echo 'export BE_SDK_DIR=$HOME/be_sdk' >> ~/.bashrc
source ~/.bashrc
```

#### Method 2: CMake variable

Specify the SDK path via the `-D` flag at build time:

```bash
colcon build --cmake-args -DBE_SDK_DIR=/path/to/be_sdk
```

#### Method 3: Directory convention

Place the SDK alongside this package, named `be_sdk`. The build system will discover it automatically:

```text
your_ws/src/
├── binosense_ros2/    # This driver package
└── be_sdk/            # SDK (auto-discovered)
```

```bash
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/be_sdk.git be_sdk
```

### Step 3: System setup

Create the temporary directory required by the SDK at runtime:

```bash
sudo mkdir -p /usr/Evo_BionicEyes/tmp_path
sudo chmod 777 /usr/Evo_BionicEyes/tmp_path
```

### Step 4: Build

```bash
cd ~/binosense_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **Verify SDK path**: CMake will print the SDK path during build:
>
> ```text
> -- Using BinoSense SDK: /path/to/your/be_sdk
> --   Include: /path/to/your/be_sdk/include
> --   Library: /path/to/your/be_sdk/lib/linux
> ```
>
> If you see a `FATAL_ERROR`, the SDK path is not configured correctly — go back to Step 2.

## Basic Usage

### 1. Launch the driver node

```bash
source install/setup.bash

# Auto-detect device
ros2 run binosense_ros2 binosense_driver

# Specify device IP (replace with your BinoSense device IP)
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP>

# Enable depth
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP> -p depth_enabled:=true
```

### 2. Launch driver + RViz

```bash
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>

# Enable depth point cloud
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP> depth_enabled:=true
```

### 3. Launch control example

```bash
# Sweep mode (auto scanning)
ros2 run binosense_ros2 binosense_control_example --ros-args -p mode:=sweep

# Position mode (reset)
ros2 run binosense_ros2 binosense_control_example --ros-args -p mode:=position
```

### 4. Python interactive control

```bash
ros2 run binosense_ros2 simple_control.py
```

Commands:

```text
home     - Return to home position
left     - Look left
right    - Look right
up       - Look up
down     - Look down
sweep    - Auto sweep demo
pos <R_P> <R_R> <R_Y> <L_P> <L_R> <L_Y> - Set specific angles
quit     - Exit
```

## Topics

### Published Topics

| Topic | Type | Description |
| ------- | ------ | ------------- |
| `/left/image_raw` | sensor_msgs/Image | Left raw image |
| `/left/camera_info` | sensor_msgs/CameraInfo | Left camera info |
| `/right/image_raw` | sensor_msgs/Image | Right raw image |
| `/right/camera_info` | sensor_msgs/CameraInfo | Right camera info |
| `/depth/image_raw` | sensor_msgs/Image | Depth map (MONO16, in mm) |
| `/depth/image_color` | sensor_msgs/Image | Color depth map (JET colormap, normalized) |
| `/depth/points` | sensor_msgs/PointCloud2 | Color point cloud |
| `/depth/rectified_left/image_raw` | sensor_msgs/Image | Rectified left image |
| `/depth/rectified_right/image_raw` | sensor_msgs/Image | Rectified right image |
| `/motor_state` | std_msgs/Float32MultiArray | Motor state [R_P, R_R, R_Y, L_P, L_R, L_Y] |
| `/binosense_data` | binosense_ros2/BinosenseData | Complete device data |
| `/tf` | tf2_msgs/TFMessage | Coordinate transforms |

### Subscribed Topics

| Topic | Type | Description |
| ------- | ------ | ------------- |
| `/motor_command` | std_msgs/Float32MultiArray | Motor position command [6 angles] |
| `/cmd_vel` | geometry_msgs/Twist | Velocity control command |

## Services

### Motor Control

#### set_motor_position

```bash
# Set absolute position
ros2 service call /set_motor_position binosense_ros2/srv/SetMotorPosition "{
  angles: [0.0, 0.0, 10.0, 0.0, 0.0, 10.0],
  absolute: true,
  move_pattern: 1,
  move_base: 0
}"

# Set relative position
ros2 service call /set_motor_position binosense_ros2/srv/SetMotorPosition "{
  angles: [0.0, 0.0, 5.0, 0.0, 0.0, 5.0],
  absolute: false,
  move_pattern: 1,
  move_base: 0
}"
```

Parameters:
- `move_pattern`: 0=Saccade (fast), 1=SmoothPursuit (smooth)
- `move_base`: 0=independent, 1=left eye reference, 2=right eye reference

#### set_motor_speed

```bash
ros2 service call /set_motor_speed binosense_ros2/srv/SetMotorSpeed "{
  active_flag: [true, true, true, true, true, true],
  speeds: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
}"
```

### Camera Control

#### set_camera_exposure

```bash
# Auto exposure
ros2 service call /set_camera_exposure binosense_ros2/srv/SetCameraExposure "{
  camera: -1, auto_exposure: true, exposure_time: 0.0
}"

# Manual exposure
ros2 service call /set_camera_exposure binosense_ros2/srv/SetCameraExposure "{
  camera: -1, auto_exposure: false, exposure_time: 100.0
}"
```

- `camera`: -1=both eyes, 0=right eye, 1=left eye

#### set_camera_white_balance

```bash
ros2 service call /set_camera_white_balance binosense_ros2/srv/SetCameraWhiteBalance "{
  camera: -1, auto_white_balance: false, temperature: 4500.0
}"
```

#### set_camera_gain

```bash
ros2 service call /set_camera_gain binosense_ros2/srv/SetCameraGain "{
  camera: -1, auto_gain: false, gain: 100
}"
```

### VOR/SV Control

#### set_vor

```bash
ros2 service call /set_vor binosense_ros2/srv/SetVOR "{vor_eye: false, vor_neck: false}"
```

#### set_sv

```bash
ros2 service call /set_sv binosense_ros2/srv/SetSV "{onoff: true}"
```

### Depth Control

#### set_depth_control

```bash
ros2 service call /set_depth_control binosense_ros2/srv/SetDepthControl "{
  enabled: true,
  sv_enabled: false,
  precision: 1,
  distance_min: 200.0,
  distance_max: 5000.0
}"
```

- `precision`: Depth precision. 1=mm, 10=cm, 100=dm

### Data Saving

#### save_data

```bash
# Start continuous save
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 0, folder_path: './data_save', description: ''
}"

# Stop saving
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 1, folder_path: '', description: ''
}"

# Single frame snap
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 2, folder_path: './snap', description: 'test snapshot'
}"

# Record current frame
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 3, folder_path: './record', description: 'test record'
}"

# Stop recording
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 4, folder_path: '', description: ''
}"
```

### Image Resolution

#### set_image_resolution

```bash
ros2 service call /set_image_resolution binosense_ros2/srv/SetImageResolution "{
  width: 1920, height: 1080, transfer_only: false
}"
```

### Motor Limits

#### get_motor_limits

```bash
# Eye motors
ros2 service call /get_motor_limits binosense_ros2/srv/GetMotorLimits "{motor_type: 0}"

# Neck motors (100=Pitch, 101=Roll, 102=Yaw)
ros2 service call /get_motor_limits binosense_ros2/srv/GetMotorLimits "{motor_type: 100}"
```

### Neck Control

#### set_neck_position

```bash
ros2 service call /set_neck_position binosense_ros2/srv/SetNeckPosition "{
  angles: [5.0, 0.0, 10.0], absolute: true, move_pattern: 1
}"
```

### Extra Module Control

#### set_extra_module

```bash
# Laser rangefinder
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 0, param0: 0, param1: 1, param2: 0
}"

# IR fill light
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 1, param0: 0, param1: 1, param2: 255
}"

# Zoom camera
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 2, param0: 10, param1: 0, param2: 0
}"

# Wiper
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 3, param0: 0, param1: 200, param2: 0
}"

# Illumination
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 5, param0: 0, param1: 200, param2: 0
}"
```

Module types:
- 0: Laser rangefinder
- 1: IR fill light
- 2: Zoom camera
- 3: Wiper
- 4: Mouth
- 5: Illumination
- 6: Robotic arm

## Topic Command Examples

### Motor Position Control

```bash
# Turn both eyes right by 10 degrees
ros2 topic pub /motor_command std_msgs/Float32MultiArray "{
  data: [0.0, 0.0, 10.0, 0.0, 0.0, 10.0]
}" -1
```

### Twist Velocity Control

```bash
# Turn right (yaw)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{
  angular: {z: -2.0}
}" -1

# Look up (pitch)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{
  angular: {y: 1.0}
}" -1
```

### View Images

```bash
# View left image
ros2 run image_view image_view image:=/left/image_raw

# View right image
ros2 run image_view image_view image:=/right/image_raw

# View grayscale depth map
ros2 run image_view image_view image:=/depth/image_raw

# View color depth map (JET colormap)
ros2 run image_view image_view image:=/depth/image_color
```

### View Point Cloud

```bash
# View in RViz
rviz2
# Add PointCloud2 display, select topic /depth/points
```

## Launch Parameters

| Parameter | Default | Description |
| ----------- | --------- | ------------- |
| `frame_prefix` | "binosense" | TF frame name prefix |
| `connection_mode` | "ic" | Connection mode: i=image, c=control, ic=both |
| `device_ip` | "" | BinoSense device IP address (empty=auto-detect) |
| `publish_rate` | 25.0 | Publish rate (Hz) |
| `publish_tf` | true | Whether to publish TF transforms |
| `stereo_enabled` | true | Whether to publish stereo images |
| `depth_enabled` | false | Whether to enable depth and point cloud |
| `depth_precision` | 1 | Depth precision (1=mm) |
| `depth_min` | 200.0 | Minimum valid depth distance (mm) |
| `depth_max` | 5000.0 | Maximum valid depth distance (mm) |
| `depth_sv_enabled` | false | Whether depth computation uses SV |

## Motor Index

Data order: [Right Pitch, Right Roll, Right Yaw, Left Pitch, Left Roll, Left Yaw]

| Index | Name | Range |
| ------- | ------ | ------- |
| 0 | Right Pitch | ±30° |
| 1 | Right Roll | ±20° |
| 2 | Right Yaw | ±30° |
| 3 | Left Pitch | ±30° |
| 4 | Left Roll | ±20° |
| 5 | Left Yaw | ±30° |

## TF Frames

- `binosense/base_link` - Base frame
- `binosense/left_camera` - Left camera frame (dynamic, follows motors)
- `binosense/right_camera` - Right camera frame (dynamic, follows motors)

## SDK Path Configuration

CMake searches for the BinoSense SDK in the following priority order:

| Priority | Method | Example |
| ---------- | -------- | --------- |
| 1 | CMake variable `-DBE_SDK_DIR` | `colcon build --cmake-args -DBE_SDK_DIR=/opt/be_sdk` |
| 2 | Environment variable `BE_SDK_DIR` | `export BE_SDK_DIR=~/be_sdk` |
| 3 | Sibling directory `../be_sdk` | Clone SDK to `binosense_ros2/../be_sdk` |
| 4 | Parent directory (monorepo mode) | Package is at `be_sdk/ros2_ws/src/binosense_ros2` |

The SDK directory structure should be:

```text
be_sdk/
├── include/
│   ├── bionic_eyes_c.h
│   └── bionic_eyes_cpp_wrapper.h
└── lib/
    └── linux/
        ├── libbionic_eyes_c.so
        └── libevo_openglutils64.so
```

## Troubleshooting

### 1. SDK not found during build

Error message: `BinoSense SDK not found`

Solution: Make sure the SDK path is configured via one of the three methods above, and the SDK directory contains `include/` and `lib/linux/` subdirectories.

### 2. Device not found

Ensure your PC and the BinoSense device are on the same network segment, and check firewall settings:

```bash
ping <YOUR_DEVICE_IP>
```

### 3. Depth point cloud not published

Depth must be explicitly enabled:

```bash
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP> -p depth_enabled:=true
```

### 4. Image stuttering

Reduce the publish rate or lower the transfer resolution:

```bash
ros2 service call /set_image_resolution binosense_ros2/srv/SetImageResolution "{
  width: 960, height: 540, transfer_only: true
}"
```

### 5. No point cloud in RViz

Make sure the fixed frame is set to `binosense/base_link`, and add a PointCloud2 display plugin with topic `/depth/points`.

### 6. Runtime error: libbionic_eyes_c.so not found

The SDK library is installed to the ROS2 package's lib directory during build. If the issue persists:

```bash
# Method 1: Add SDK lib directory to search path
export LD_LIBRARY_PATH=$BE_SDK_DIR/lib/linux:$LD_LIBRARY_PATH

# Method 2: Copy to system path
sudo cp $BE_SDK_DIR/lib/linux/lib*.so /usr/local/lib/
sudo ldconfig
```

## Data Recording

### Record all data

```bash
ros2 bag record -a
```

### Record specific topics

```bash
ros2 bag record /left/image_raw /right/image_raw /depth/image_raw /depth/image_color /depth/points /motor_state
```

## License

This package uses the BinoSense SDK. See the [SDK license](https://github.com/BinoSense/be_sdk) for details.
