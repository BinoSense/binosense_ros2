# BinoSense ROS2 Driver

BinoSense 动态双目视觉系统 (Active Binocular Vision System) 的 ROS2 驱动包，提供图像传输、电机控制、深度感知等完整功能。

## 功能概览

- 图像传输与显示（左右目图像、拼接图像）
- 六轴电机控制
- 深度图与点云显示
- 相机参数控制（曝光、白平衡、增益）
- VOR/SV 功能控制
- 数据记录与保存（snap/record/save）
- 颈部电机控制
- 额外模块控制（雨刮、变焦、照明等）
- TF 坐标变换广播
- RViz 可视化支持

## 前置条件

- ROS2 (Humble/Iron/Jazzy)
- OpenCV 4.x
- CMake 3.16+
- [BinoSense SDK](https://github.com/BinoSense/be_sdk)（需单独获取）

## 安装与编译

### 第一步：获取本包

```bash
mkdir -p ~/binosense_ws/src
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/binosense_ros2.git
```

### 第二步：配置 SDK 依赖

本驱动依赖 [BinoSense SDK](https://github.com/BinoSense/be_sdk) 的头文件和动态库。有以下三种方式让编译系统找到 SDK：

#### 方式 1：环境变量（推荐）

```bash
# 将 SDK 克隆到任意位置
git clone https://github.com/BinoSense/be_sdk.git ~/be_sdk

# 设置环境变量
export BE_SDK_DIR=~/be_sdk
```

建议将 `export` 命令写入 `~/.bashrc` 以便持久化：

```bash
echo 'export BE_SDK_DIR=$HOME/be_sdk' >> ~/.bashrc
source ~/.bashrc
```

#### 方式 2：CMake 变量

编译时通过 `-D` 参数指定：

```bash
colcon build --cmake-args -DBE_SDK_DIR=/path/to/be_sdk
```

#### 方式 3：目录约定

将 SDK 放在本包的同级目录下，命名为 `be_sdk`，编译系统会自动发现：

```text
your_ws/src/
├── binosense_ros2/    # 本驱动包
└── be_sdk/            # SDK（自动发现）
```

```bash
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/be_sdk.git be_sdk
```

### 第三步：系统设置

创建 SDK 运行时需要的临时目录：

```bash
sudo mkdir -p /usr/Evo_BionicEyes/tmp_path
sudo chmod 777 /usr/Evo_BionicEyes/tmp_path
```

### 第四步：编译

```bash
cd ~/binosense_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **验证 SDK 路径**：编译时 CMake 会打印 SDK 路径信息：
>
> ```text
> -- Using BinoSense SDK: /path/to/your/be_sdk
> --   Include: /path/to/your/be_sdk/include
> --   Library: /path/to/your/be_sdk/lib/linux
> ```
>
> 如果看到 `FATAL_ERROR` 提示，说明 SDK 路径未正确配置，请返回第二步。

## 基础使用

### 1. 启动驱动节点

```bash
source install/setup.bash

# 自动检测设备
ros2 run binosense_ros2 binosense_driver

# 指定设备 IP（替换为你的 BinoSense 设备 IP）
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP>

# 启用深度功能
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP> -p depth_enabled:=true
```

### 2. 启动驱动 + RViz

```bash
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>

# 启用深度点云
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP> depth_enabled:=true
```

### 3. 启动控制示例

```bash
# sweep 模式（自动扫动）
ros2 run binosense_ros2 binosense_control_example --ros-args -p mode:=sweep

# position 模式（复位）
ros2 run binosense_ros2 binosense_control_example --ros-args -p mode:=position
```

### 4. Python 交互控制

```bash
ros2 run binosense_ros2 simple_control.py
```

命令说明：

```text
home     - 归位
left     - 向左看
right    - 向右看
up       - 向上看
down     - 向下看
sweep    - 自动扫动演示
pos <R_P> <R_R> <R_Y> <L_P> <L_R> <L_Y> - 设置指定角度
quit     - 退出
```

## 话题列表

### 发布的话题

| 话题名 | 类型 | 说明 |
| -------- | ------ | ------ |
| `/left/image_raw` | sensor_msgs/Image | 左目原始图像 |
| `/left/camera_info` | sensor_msgs/CameraInfo | 左目相机参数 |
| `/right/image_raw` | sensor_msgs/Image | 右目原始图像 |
| `/right/camera_info` | sensor_msgs/CameraInfo | 右目相机参数 |
| `/depth/image_raw` | sensor_msgs/Image | 深度图 (MONO16, mm单位) |
| `/depth/image_color` | sensor_msgs/Image | 彩色深度图 (JET色带, 归一化显示) |
| `/depth/points` | sensor_msgs/PointCloud2 | 彩色点云 |
| `/depth/rectified_left/image_raw` | sensor_msgs/Image | 去畸变的左目图像 |
| `/depth/rectified_right/image_raw` | sensor_msgs/Image | 去畸变的右目图像 |
| `/motor_state` | std_msgs/Float32MultiArray | 电机状态[R_P, R_R, R_Y, L_P, L_R, L_Y] |
| `/binosense_data` | binosense_ros2/BinosenseData | 完整设备数据 |
| `/tf` | tf2_msgs/TFMessage | 坐标变换 |

### 订阅的话题

| 话题名 | 类型 | 说明 |
| -------- | ------ | ------ |
| `/motor_command` | std_msgs/Float32MultiArray | 电机位置指令[6个角度] |
| `/cmd_vel` | geometry_msgs/Twist | 速度控制指令 |

## 服务列表

### 电机控制

#### set_motor_position

```bash
# 设置绝对位置
ros2 service call /set_motor_position binosense_ros2/srv/SetMotorPosition "{
  angles: [0.0, 0.0, 10.0, 0.0, 0.0, 10.0],
  absolute: true,
  move_pattern: 1,
  move_base: 0
}"

# 设置相对位置
ros2 service call /set_motor_position binosense_ros2/srv/SetMotorPosition "{
  angles: [0.0, 0.0, 5.0, 0.0, 0.0, 5.0],
  absolute: false,
  move_pattern: 1,
  move_base: 0
}"
```

参数说明：
- `move_pattern`: 0=Saccade(快速), 1=SmoothPursuit(平滑)
- `move_base`: 0=独立运动, 1=左眼基准, 2=右眼基准

#### set_motor_speed

```bash
ros2 service call /set_motor_speed binosense_ros2/srv/SetMotorSpeed "{
  active_flag: [true, true, true, true, true, true],
  speeds: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
}"
```

### 相机控制

#### set_camera_exposure

```bash
# 自动曝光
ros2 service call /set_camera_exposure binosense_ros2/srv/SetCameraExposure "{
  camera: -1, auto_exposure: true, exposure_time: 0.0
}"

# 手动曝光
ros2 service call /set_camera_exposure binosense_ros2/srv/SetCameraExposure "{
  camera: -1, auto_exposure: false, exposure_time: 100.0
}"
```

- `camera`: -1=双眼, 0=右目, 1=左目

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

### VOR/SV 控制

#### set_vor

```bash
ros2 service call /set_vor binosense_ros2/srv/SetVOR "{vor_eye: false, vor_neck: false}"
```

#### set_sv

```bash
ros2 service call /set_sv binosense_ros2/srv/SetSV "{onoff: true}"
```

### 深度控制

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

- `precision`: 深度精度。1=毫米, 10=厘米, 100=分米

### 数据保存

#### save_data

```bash
# 开始持续保存
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 0, folder_path: './data_save', description: ''
}"

# 停止保存
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 1, folder_path: '', description: ''
}"

# 单帧快照 snap
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 2, folder_path: './snap', description: 'test snapshot'
}"

# 记录当前帧 record
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 3, folder_path: './record', description: 'test record'
}"

# 停止记录
ros2 service call /save_data binosense_ros2/srv/SaveData "{
  action: 4, folder_path: '', description: ''
}"
```

### 图像分辨率

#### set_image_resolution

```bash
ros2 service call /set_image_resolution binosense_ros2/srv/SetImageResolution "{
  width: 1920, height: 1080, transfer_only: false
}"
```

### 电机限位

#### get_motor_limits

```bash
# 眼部电机
ros2 service call /get_motor_limits binosense_ros2/srv/GetMotorLimits "{motor_type: 0}"

# 颈部电机 (100=Pitch,101=Roll,102=Yaw)
ros2 service call /get_motor_limits binosense_ros2/srv/GetMotorLimits "{motor_type: 100}"
```

### 颈部控制

#### set_neck_position

```bash
ros2 service call /set_neck_position binosense_ros2/srv/SetNeckPosition "{
  angles: [5.0, 0.0, 10.0], absolute: true, move_pattern: 1
}"
```

### 额外模块控制

#### set_extra_module

```bash
# 激光测距
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 0, param0: 0, param1: 1, param2: 0
}"

# 红外补光灯
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 1, param0: 0, param1: 1, param2: 255
}"

# 变焦相机
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 2, param0: 10, param1: 0, param2: 0
}"

# 雨刮器
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 3, param0: 0, param1: 200, param2: 0
}"

# 照明
ros2 service call /set_extra_module binosense_ros2/srv/SetExtraModule "{
  module_type: 5, param0: 0, param1: 200, param2: 0
}"
```

模块类型说明：
- 0: 激光测距
- 1: 红外补光灯
- 2: 变焦相机
- 3: 雨刮器
- 4: 嘴巴
- 5: 照明
- 6: 机械臂

## 话题命令示例

### 电机位置控制

```bash
# 让双眼同时右转 10 度
ros2 topic pub /motor_command std_msgs/Float32MultiArray "{
  data: [0.0, 0.0, 10.0, 0.0, 0.0, 10.0]
}" -1
```

### Twist 速度控制

```bash
# 向右转动 (yaw)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{
  angular: {z: -2.0}
}" -1

# 向上看 (pitch)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{
  angular: {y: 1.0}
}" -1
```

### 查看图像

```bash
# 查看左目图像
ros2 run image_view image_view image:=/left/image_raw

# 查看右目图像
ros2 run image_view image_view image:=/right/image_raw

# 查看灰度深度图
ros2 run image_view image_view image:=/depth/image_raw

# 查看彩色深度图 (JET色带)
ros2 run image_view image_view image:=/depth/image_color
```

### 查看点云

```bash
# 在 RViz 中查看
rviz2
# 添加 PointCloud2 显示，选择话题 /depth/points
```

## 启动参数说明

| 参数名 | 默认值 | 说明 |
| -------- | -------- | ------ |
| `frame_prefix` | "binosense" | TF 帧名称前缀 |
| `connection_mode` | "ic" | 连接模式: i=图像, c=控制, ic=两者 |
| `device_ip` | "" | BinoSense 设备 IP 地址 (空=自动检测) |
| `publish_rate` | 25.0 | 发布频率 (Hz) |
| `publish_tf` | true | 是否发布 TF 变换 |
| `stereo_enabled` | true | 是否发布双目图像 |
| `depth_enabled` | false | 是否启用深度和点云 |
| `depth_precision` | 1 | 深度精度 (1=mm) |
| `depth_min` | 200.0 | 最小有效深度距离 (mm) |
| `depth_max` | 5000.0 | 最大有效深度距离 (mm) |
| `depth_sv_enabled` | false | 深度计算是否启用 SV |

## 电机索引说明

数据顺序：[右目俯仰, 右目滚动, 右目偏航, 左目俯仰, 左目滚动, 左目偏航]

| 索引 | 名称 | 英文 | 范围 |
| ------ | ------ | ------ | ------ |
| 0 | 右目俯仰 | Right Pitch | ±30° |
| 1 | 右目滚动 | Right Roll | ±20° |
| 2 | 右目偏航 | Right Yaw | ±30° |
| 3 | 左目俯仰 | Left Pitch | ±30° |
| 4 | 左目滚动 | Left Roll | ±20° |
| 5 | 左目偏航 | Left Yaw | ±30° |

## TF 帧

- `binosense/base_link` - 基帧
- `binosense/left_camera` - 左相机帧（随电机动态变化）
- `binosense/right_camera` - 右相机帧（随电机动态变化）

## SDK 路径配置详解

CMake 按以下优先级查找 BinoSense SDK：

| 优先级 | 方式 | 示例 |
| -------- | ------ | ------ |
| 1 | CMake 变量 `-DBE_SDK_DIR` | `colcon build --cmake-args -DBE_SDK_DIR=/opt/be_sdk` |
| 2 | 环境变量 `BE_SDK_DIR` | `export BE_SDK_DIR=~/be_sdk` |
| 3 | 同级目录 `../be_sdk` | 将 SDK 克隆到 `binosense_ros2/../be_sdk` |
| 4 | 上级目录（monorepo 模式） | 本包位于 `be_sdk/ros2_ws/src/binosense_ros2` 时 |

SDK 目录结构应如下：

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

## 常见问题

### 1. 编译时找不到 SDK

错误信息：`BinoSense SDK not found`

解决方法：确保通过上述三种方式之一配置了 SDK 路径，且 SDK 目录下包含 `include/` 和 `lib/linux/` 子目录。

### 2. 找不到设备

确保电脑与 BinoSense 设备在同一网段，并检查防火墙设置：

```bash
ping <YOUR_DEVICE_IP>
```

### 3. 深度点云没有发布

需要显式启用深度功能：

```bash
ros2 run binosense_ros2 binosense_driver --ros-args -p device_ip:=<YOUR_DEVICE_IP> -p depth_enabled:=true
```

### 4. 图像卡顿

降低发布频率或降低传输分辨率：

```bash
ros2 service call /set_image_resolution binosense_ros2/srv/SetImageResolution "{
  width: 960, height: 540, transfer_only: true
}"
```

### 5. RViz 中没有点云显示

确保设置了正确的固定帧：`binosense/base_link`，并添加 PointCloud2 显示插件，选择话题 `/depth/points`。

### 6. 运行时找不到 libbionic_eyes_c.so

SDK 的动态库在编译时会被安装到 ROS2 包的 lib 目录中。如果仍有问题：

```bash
# 方法 1：将 SDK lib 目录加入搜索路径
export LD_LIBRARY_PATH=$BE_SDK_DIR/lib/linux:$LD_LIBRARY_PATH

# 方法 2：复制到系统路径
sudo cp $BE_SDK_DIR/lib/linux/lib*.so /usr/local/lib/
sudo ldconfig
```

## 数据录制

### 录制所有数据

```bash
ros2 bag record -a
```

### 录制指定话题

```bash
ros2 bag record /left/image_raw /right/image_raw /depth/image_raw /depth/image_color /depth/points /motor_state
```

## 故障排除

### 无法连接设备

- 确认 BinoSense 设备已通电并连接到网络
- 检查网络连接（ping 设备 IP）
- 确保 `/usr/Evo_BionicEyes/tmp_path` 存在并具有正确权限

### 没有图像发布

- 检查连接模式是否为 "i" 或 "ic"
- 验证设备是否正在传输数据
- 检查控制台是否有错误信息

### 编译错误

- 确保 ROS2 环境已正确配置：`source /opt/ros/<distro>/setup.bash`
- 确认 SDK 路径配置正确（查看 CMake 输出的 `-- Using BinoSense SDK:` 信息）
- 确认所有依赖已安装：`rosdep install --from-paths src --ignore-src -r -y`

## 许可证

本软件包使用 BinoSense SDK，详情请参考 [SDK 许可证](https://github.com/BinoSense/be_sdk)。
