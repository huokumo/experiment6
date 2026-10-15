# 实验六基础工程：R1 位姿、运动与深度障碍接口

本工程接续实验五，提供三个基础模块：

1. 从 Gazebo 的实际模型位姿发布标准 ROS 2 里程计、位姿和动态 TF。
2. 接收统一速度指令，提供带碰撞阻挡的仿真执行层，以及独立的宇树 R1 SDK 真机适配源码。
3. 将深度图转换为机器人机身坐标系中的障碍点云和左、中、右距离信息。

当前阶段提供接口基础，不包含 Nav2 启动配置、自动绕墙算法、视觉 SLAM 或双足步态控制。仿真中的 R1 保持站姿做平面移动，不能将该效果解释为已实现真实双足行走。真机 SDK 后端与仿真后端分别启动，SDK 不承担 Gazebo 中的步态控制。

## 环境与实验五衔接

- Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6。
- 实验五已经构建，`experiment5_depth` 可通过 ROS 包索引找到。
- 复用实验五安装目录中的 R1 模型、网格、URDF、蓝色墙壁场景和深度桥接配置。
- 启动时在临时目录生成实验六场景，不修改实验五源码或安装文件。
- 基准参考：[huokumo/experiment5](https://github.com/huokumo/experiment5)，开发时核对的提交为 `807de9e2454d0c6bd7ab3067e6743acf429e19f9`。

## 部署

将本仓库放在 `/root/exp5/src/experiment6_navigation`。本仓库根目录就是 ROS 2 功能包，不需要在内部再创建同名目录。

实验五已有的运行依赖继续沿用；另安装插件编译及自定义消息生成依赖：

```bash
apt update
apt install -y libignition-gazebo6-dev ros-humble-ament-cmake-python \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-tf2-ros ros-humble-tf2-ros-py ros-humble-rclcpp \
  python3-numpy python3-opencv python3-yaml
```

使用与实验五相同的 Fortress 软件源。如果找不到 `libignition-gazebo6-dev`，应检查 Gazebo Fortress 软件源，而不是更换为 Gazebo Classic 或另一主版本。

```bash
cd /root/exp5
source /opt/ros/humble/setup.bash
source /root/exp5/install/setup.bash
colcon build --symlink-install --packages-select experiment6_navigation
source /root/exp5/install/setup.bash
```

默认不下载或编译宇树 SDK，仿真实验不依赖真机 SDK。

## 启动三个基础模块

先停止实验五正在运行的 Gazebo 场景。两个终端使用同一套 ROS/DDS 环境。

终端1：

```bash
source /opt/ros/humble/setup.bash
source /root/exp5/install/setup.bash
ros2 launch experiment6_navigation experiment6_interfaces.launch.py require_depth:=true
```

此命令启动仿真、桥接、机器人模型 TF、位姿发布、速度适配和深度障碍处理。初始没有速度指令时，机器人保持停止。

需要 RViz 时，沿用已安装的 RViz 2，增加 `rviz:=true`：

若尚未安装 RViz 2，可先执行 `apt install -y ros-humble-rviz2`。

```bash
ros2 launch experiment6_navigation experiment6_interfaces.launch.py require_depth:=true rviz:=true
```

主要启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_x`、`robot_y`、`robot_yaw` | 0 | 初始世界位置和朝向；yaw 单位为 rad |
| `robot_z` | 0 | 当前平面模型只接受0 |
| `depth_rate` | 10 | 深度图目标更新率，单位为仿真时间 Hz |
| `software_rendering` | true | 沿用虚拟机软件渲染配置 |
| `gui` | true | 显示 Gazebo 窗口 |
| `rviz` | false | 显示预置 RViz 界面 |
| `require_depth` | false | 为 true 时，深度状态不可用会停止速度输出 |

`require_depth:=false` 用于先调通位姿和手动运动入口；接入避障时使用 `true`。该开关只检查数据可用性，不代替避障算法，机器人不会仅因检测到障碍就自动选择绕行方向。

## 查看位姿与障碍信息

终端2加载环境后执行：

```bash
source /opt/ros/humble/setup.bash
source /root/exp5/install/setup.bash
ros2 topic echo /r1/pose --once
ros2 topic echo /r1/odom --once
ros2 topic echo /r1/obstacle_summary --once
```

接口如下：

| 话题 | 消息类型 | 内容 |
|---|---|---|
| `/r1/odom` | `nav_msgs/msg/Odometry` | 实际位姿及机身坐标系速度 |
| `/r1/pose` | `geometry_msgs/msg/PoseStamped` | 实际位姿 |
| `/tf`、`/tf_static` | TF | 地图、机身与相机坐标关系 |
| `/r1/cmd_vel` | `geometry_msgs/msg/Twist` | 前后、横移及转向速度输入 |
| `/r1/obstacle_points` | `sensor_msgs/msg/PointCloud2` | 机身坐标系障碍点云 |
| `/r1/obstacle_summary` | `experiment6_navigation/msg/ObstacleSummary` | 左、中、右距离与有效状态 |
| `/r1/depth/debug` | `sensor_msgs/msg/Image` | 带分区距离标签的深度伪彩图 |
| `/r1/motion_status` | `std_msgs/msg/String` | 仿真运动适配器状态，内容为 JSON |
| `/r1/sim/collision_blocked` | `std_msgs/msg/Bool` | 仿真碰撞保护是否阻挡平移 |

`/r1/sim/*` 是仿真内部接口。后续学生程序和导航程序使用公开的 `/r1/cmd_vel`、`/r1/odom` 及障碍接口。

## 前进、转向与停止

使用有明确持续时间的命令，不持续锁存非零速度。以下命令只用于 Gazebo 仿真启动后的小范围接口调试。

```bash
# 前进2秒。
ros2 run experiment6_navigation drive_for.py --vx 0.15 --seconds 2

# 向左转动1秒，角速度0.3 rad/s。
ros2 run experiment6_navigation drive_for.py --yaw-rate 0.3 --seconds 1

# 显式发送停止。
ros2 run experiment6_navigation drive_for.py --seconds 0.5
```

命令结束后工具主动发送零速度；运动节点另有0.4秒指令超时处理，仿真插件还有独立的0.35秒接收超时。最大平移速度为0.25 m/s、最大转向角速度为0.6 rad/s，均可通过配置调整，但不能超过仿真插件的上限。

`drive_for.py` 的持续时间按墙钟计算；仿真运行较慢时，同样的墙钟时间对应较短的仿真时间，实际位移以位姿反馈为准。

位姿应随运动变化，正的 `angular.z` 表示向左转。靠近墙体时，仿真插件按圆形身体占用范围阻挡继续穿墙；这属于执行层碰撞保护，不是自动避障规划。

## 深度处理规则

1. 订阅实验五的深度 Image 和对应 CameraInfo，检查编码、尺寸、时间及坐标帧。
2. `32FC1` 按米读取；`16UC1` 按毫米换算成米，正确处理步长和字节序。
3. 排除 NaN、Inf、非正数和配置量程以外的样本。
4. 用深度相机自身的 K/D 反投影，不使用320×240 RGB 标定参数。
5. 根据相机到 `base_link` 的静态 TF，将点转换为前、左、上的机身坐标。
6. 按高度、范围和机身邻域过滤地面与自身点，发布障碍点云。
7. 按机身方位角汇总左、中、右最近障碍距离，同时计算各分区有效深度比例。

`ObstacleSummary` 的数组顺序固定为 `[左, 前, 右]`：

- `UNKNOWN=0`：观测不足，距离为 NaN，不能当成空地。
- `CLEAR=1`：当前观测区域内没有达到阻挡阈值的障碍。
- `BLOCKED=2`：存在达到阻挡阈值的近距离障碍。

距离是障碍点到 `base_link` 原点的水平距离，单位为米。`CLEAR` 只描述相机实际覆盖的范围、高度和量程，不保证相机视野外或遮挡后的空间可通行。`max_range` 是未检出障碍时的报告上限，不是对整段空间的安全证明。

该相机视场有限，因此仅凭三个距离不能保证完整绕过4 m宽墙体。后续需要加入有记忆的绕行状态、机器人占用范围和目标位置判断，或接 Nav2。

## 仿真执行层的边界

- 保持实验五静态站姿，通过 Fortress 的模型位姿命令进行小步平面运动。
- 从仿真模型的实际位姿发布反馈，速度由实际位姿变化计算，不把控制指令直接当反馈。
- 保持高度、横滚角和俯仰角，允许平面位置和偏航角变化。
- 根据实验五场景中的静态盒体碰撞几何，进行带半径的连续线段碰撞检查；绕行算法不能访问这份碰撞几何。
- 采用0.45 m保守圆形占用半径，墙角使用保守扩张盒体，可能比真实轮廓提前阻挡。
- 场景活动范围为 X∈[-5.5,5.5]、Y∈[-3.5,3.5]，检查时还扣除身体半径。
- 当前不支持动态障碍的物理接触、非盒体场景碰撞、倾斜障碍或跨台阶。场景生成器会拒绝未支持的碰撞类型。
- 暂不提供 Gazebo 原地复位后的自动恢复，重新启动 launch 可恢复基准场景。

这是感知与导航教学用运动执行层，不用于验证双足稳定性、接触力、步态或真机运动性能。

## 可选：编译 R1 真机高层接口

源码位于 `src/r1_sdk_adapter.cpp`。它使用官方 `unitree::robot::r1::LocoClient`，映射：

```text
Twist.linear.x  → vx
Twist.linear.y  → vy
Twist.angular.z → omega
SetVelocity(vx, vy, omega, 0.2)
```

需要先按官方说明安装包含 R1 接口的 [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)，然后重新构建：

```bash
cd /root/exp5
colcon build --symlink-install --packages-select experiment6_navigation \
  --cmake-args -DBUILD_UNITREE_ADAPTER=ON
source install/setup.bash
```

真机启动文件为 `r1_hardware_adapter.launch.py`。只有明确设置 `enable_hardware:=true` 并指定连接机器人所用的网卡，节点才会初始化 SDK。代码先查询运动状态机，检查通信返回值，不自动调用 `StandUp` 或 `Start`，也不启用无限时长运动模式。

真机适配器默认速度上限更低，定时发送0.2秒有效期指令，接收超时后发送零速度。它只处理速度接口，不提供真机定位、相机驱动、同步或深度对齐。当前没有完成真机联调，具体机型、固件与可用运动状态需要在对应 R1 上核对。

适配器已使用官方 SDK 提交 `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36` 完成编译与链接验证；这不等同于真机动作验证。

不能同时运行仿真适配器与真机适配器并向同一 `/r1/cmd_vel` 发送命令。真机后端不会因为安装了 SDK 而自动变成 Gazebo 步态控制器。

## 检查与验证

离线检查（需要 NumPy）：

```bash
cd /root/exp5/src/experiment6_navigation
/usr/bin/python3 -s tools/check_source.py
```

构建后的 C++/Fortress 测试：

```bash
cd /root/exp5
colcon test --packages-select experiment6_navigation
colcon test-result --verbose
```

ROS 2 节点接口测试在独立 Domain ID 中运行，不启动 Gazebo 或真机 SDK：

```bash
source /root/exp5/install/setup.bash
cd /root/exp5/src/experiment6_navigation
ROS_DOMAIN_ID=173 /usr/bin/python3 -s -m unittest discover -s tests -p test_ros_integration.py -v
```

测试状态及未验证项见 [VALIDATION.md](VALIDATION.md)，接口约定见 [docs/INTERFACES.md](docs/INTERFACES.md)。

## 模型来源与许可

本仓库不重新分发宇树网格，运行时引用实验五已安装的模型。模型和资源继续遵循实验五的 `LICENSE.unitree` 与 `THIRD_PARTY_NOTICES.md`；本仓库新增代码采用 BSD-3-Clause。
