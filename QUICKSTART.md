# BinoSense ROS2 - 快速入门指南

## 第一步：系统准备

### 1. 创建必要的目录

```bash
sudo mkdir -p /usr/Evo_BionicEyes/tmp_path
sudo chmod 777 /usr/Evo_BionicEyes/tmp_path
```

### 2. 确保已安装 ROS2

推荐使用 ROS2 Humble、Iron 或 Jazzy 版本。

```bash
# 检查 ROS2 版本
echo $ROS_DISTRO
```

## 第二步：获取代码与 SDK

### 1. 克隆本驱动包

```bash
mkdir -p ~/binosense_ws/src
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/binosense_ros2.git
```

### 2. 获取 BinoSense SDK

本驱动依赖 [BinoSense SDK](https://github.com/BinoSense/be_sdk)，请选择以下任一方式配置：

**方式 A — 同级目录（最简单）：**

```bash
cd ~/binosense_ws/src
git clone https://github.com/BinoSense/be_sdk.git
```

目录结构将自动被 CMake 识别：

```text
binosense_ws/src/
├── binosense_ros2/   # 驱动包
└── be_sdk/           # SDK
```

**方式 B — 环境变量：**

```bash
# 将 SDK 克隆到任意位置
git clone https://github.com/BinoSense/be_sdk.git ~/be_sdk

# 设置环境变量
export BE_SDK_DIR=~/be_sdk

# 持久化设置（可选）
echo 'export BE_SDK_DIR=$HOME/be_sdk' >> ~/.bashrc
```

**方式 C — 编译参数：**

```bash
colcon build --cmake-args -DBE_SDK_DIR=/path/to/be_sdk
```

## 第三步：编译

```bash
cd ~/binosense_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> 编译时请确认 CMake 输出如下信息，表示 SDK 路径配置正确：
>
> ```text
> -- Using BinoSense SDK: /path/to/your/be_sdk
> --   Include: /path/to/your/be_sdk/include
> --   Library: /path/to/your/be_sdk/lib/linux
> ```

## 第四步：运行

### 启动驱动和 RViz（推荐）

```bash
# 替换 <YOUR_DEVICE_IP> 为你的 BinoSense 设备 IP 地址
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>
```

这将启动：

- BinoSense 动态双目视觉系统设备驱动
- RViz 可视化界面

### 仅启动驱动

```bash
ros2 run binosense_ros2 binosense_driver
```

### 指定设备 IP

```bash
ros2 launch binosense_ros2 binosense.launch.py device_ip:=<YOUR_DEVICE_IP>
```

### 查看话题列表

```bash
ros2 topic list
```

你应该能看到：

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

## 第五步：控制设备

### 方式 1：使用命令行

```bash
# 回到初始位置
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 向左看
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 15.0, 0.0, 0.0, 15.0]}"

# 向上看
ros2 topic pub /motor_command std_msgs/msg/Float32MultiArray "{data: [10.0, 0.0, 0.0, 10.0, 0.0, 0.0]}"
```

### 方式 2：使用 Python 控制脚本

```bash
ros2 run binosense_ros2 simple_control.py
```

在交互式界面中：

- `home` - 回到初始位置
- `left` - 向左看
- `right` - 向右看
- `up` - 向上看
- `down` - 向下看
- `sweep` - 自动扫描演示

### 方式 3：使用 C++ 示例

```bash
# 自动扫描模式
ros2 launch binosense_ros2 demo_control.launch.py mode:=sweep

# 位置模式（保持在初始位置）
ros2 launch binosense_ros2 demo_control.launch.py mode:=position
```

## 第六步：查看图像

### 方式 1：使用 RViz

启动后，RViz 会自动显示左右眼的图像。

### 方式 2：使用 rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view
```

然后在界面中选择 `/left/image_raw` 或 `/right/image_raw` 话题。

## 电机顺序说明

发送到 `/motor_command` 话题的 6 个值分别对应：

| 索引 | 电机 | 说明 | 范围 |
| ------ | ------ | ------ | ------ |
| 0 | 右眼俯仰 (Right Pitch) | 上下转动 | ±30° |
| 1 | 右眼横滚 (Right Roll) | 旋转 | ±20° |
| 2 | 右眼偏航 (Right Yaw) | 左右转动 | ±30° |
| 3 | 左眼俯仰 (Left Pitch) | 上下转动 | ±30° |
| 4 | 左眼横滚 (Left Roll) | 旋转 | ±20° |
| 5 | 左眼偏航 (Left Yaw) | 左右转动 | ±30° |

## 常见问题

### Q: 编译时提示 "BinoSense SDK not found"？

A: 驱动包需要 BinoSense SDK 才能编译。请按第二步的方式 A/B/C 之一配置 SDK 路径，确保 SDK 目录包含 `include/` 和 `lib/linux/` 子目录。

### Q: 找不到设备怎么办？

A: 检查以下几点：

1. BinoSense 设备是否已上电并连接
2. `/usr/Evo_BionicEyes/tmp_path` 目录是否存在且有写权限
3. 网络连接是否正常（确保电脑与设备在同一网段）

### Q: 没有图像发布？

A: 确认：

1. 连接模式设置为 "ic" 或 "i"
2. 设备正在传输数据
3. 检查终端错误信息

### Q: 运行时找不到 libbionic_eyes_c.so？

A: 将 SDK 库目录加入搜索路径：

```bash
export LD_LIBRARY_PATH=$BE_SDK_DIR/lib/linux:$LD_LIBRARY_PATH
```

或复制到系统路径：

```bash
sudo cp $BE_SDK_DIR/lib/linux/lib*.so /usr/local/lib/
sudo ldconfig
```

## 下一步

- 阅读完整的 [README.md](README.md) 了解更多详细信息
- 查看 [BinoSense SDK](https://github.com/BinoSense/be_sdk) 了解底层 API
- 根据需要修改 launch 文件参数
