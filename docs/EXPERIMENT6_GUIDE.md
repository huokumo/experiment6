# 实验六：机器人视觉识别与避障

## 一、任务场景与目标

延续实验五的 R1、深度相机和蓝色墙壁。机器人从世界原点出发，读取自身位姿和深度障碍信息，发布速度指令，沿墙端绕行，到达墙后的 `(4.2, 0.0)` 并停止。实验重点是数据流、坐标系、未知区域处理和一个可解释的实时状态机。

本实验的 R1 是固定站姿平面运动教学模型。它不实现双足步态、视觉定位或 Nav2，也不向真实机器人发送指令。Gazebo 的碰撞保护只是最后的仿真约束，不能代替学生的避障判断。

完成后应能：

1. 说明 `/r1/odom`、`/r1/pose`、`map → odom → base_link` 各自含义。
2. 读取深度障碍汇总的左、前、右距离及 `UNKNOWN/CLEAR/BLOCKED`，识别无效或超时数据。
3. 通过 `/r1/cmd_vel` 控制前进与转向，观察位姿和障碍距离同步变化。
4. 补全一个实时绕墙状态机，给出成功、失败和停车条件。

## 二、环境与目录

- Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6。
- 工作空间 `/root/exp6`，实验六源码 `/root/exp6/src/experiment6`。
- 实验五资源包目录 `/root/exp6/src/experiment5_depth/experiment5_depth`。
- 两个 ROS 2 包名分别为 `experiment5_depth`、`experiment6_navigation`。源码目录名与 ROS 包名可以不同。

实验六启动时从实验五资源生成临时世界，保留同一块蓝墙与 R1。为使相机在墙端和目标区域转向观察时仍看得到地面，临时世界将地面延长到 16 m × 16 m；机器人活动边界仍按原实验六平面运动配置限制。实验五源文件不修改。

终端 1 保持仿真运行，终端 2 执行读取命令。所有终端使用同一 ROS_DOMAIN_ID；启动实验六前关闭实验五 Gazebo，不另行启动实验五的 `world_to_r1_base` 静态 TF。

## 三、部署

在教学平台获得实验六 ZIP 并解压到上述源码目录。实验五 ZIP 可按实验五指导书下载；当前教学资源地址：

```bash
cd /root/exp6/src
wget -O experiment5_depth.zip "http://study.zhonghui.vip:10188/api/web/resource/embodied-1/EnvironmentalPerception/05/expPackages/experiment5_depth.zip"
mkdir -p experiment5_depth
unzip -q experiment5_depth.zip -d experiment5_depth
```

检查实验五 `package.xml` 应位于 `experiment5_depth/experiment5_depth/package.xml`。ZIP 解压可能丢失 Python 脚本执行权限，构建前恢复：

```bash
cd /root/exp6
chmod +x src/experiment6/scripts/*.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix experiment5_depth
ros2 pkg prefix experiment6_navigation
```

两个 `ros2 pkg prefix` 均应返回 `/root/exp6/install/...` 路径。若报 `pose_node.py not found on the libexec directory`，先检查脚本执行权限，再重新启动；`--symlink-install` 情况下单纯修复权限无需重建。

## 四、核对三个基础接口

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /root/exp6/install/setup.bash
ros2 launch experiment6_navigation experiment6_interfaces.launch.py \
  gui:=true rviz:=true software_rendering:=true require_depth:=true
```

Gazebo 应显示一个 R1、一块蓝色墙壁及深度图窗口；RViz 应显示机器人、TF 和障碍点。终端 2：

```bash
source /opt/ros/humble/setup.bash
source /root/exp6/install/setup.bash
ros2 node list
ros2 topic echo /r1/pose --once
ros2 topic echo /r1/obstacle_summary --once
ros2 run tf2_ros tf2_echo map base_link
```

初始位姿约为 `(0,0)`；墙距约 2.45 m。`obstacle_summary.data_valid` 应为 `true`、`status` 为 `complete`，前方距离应随机器人靠近墙壁而下降。`tf2_echo` 初始可能显示短暂等待，随后应持续输出变换。

三个分区顺序固定为 `[左, 前, 右]`。`UNKNOWN=0` 表示观测不足，`CLEAR=1` 表示相机实际覆盖区域未达到阻挡阈值，`BLOCKED=2` 表示有近障碍。`CLEAR` 不能证明侧方、后方或墙后安全。

终端 2 发出限时小动作：

```bash
ros2 run experiment6_navigation drive_for.py --vx 0.15 --seconds 2
ros2 topic echo /r1/pose --once
ros2 topic echo /r1/obstacle_summary --once
ros2 run experiment6_navigation drive_for.py --yaw-rate 0.3 --seconds 1
ros2 run experiment6_navigation drive_for.py --seconds 0.5
```

前进约 0.3 m 时，`x` 应增大，前方墙距应相应减小。转向后四元数应改变。命令结束会主动发零速度；再次读取位姿，确认机器人停止。实际位移以位姿数据为准。

## 五、参考绕墙演示

在终端 1 按 `Ctrl+C` 结束基础接口启动，等待 Gazebo 与节点退出。重新启动，且不要同时运行 `drive_for.py`：

```bash
ros2 launch experiment6_navigation experiment6_autonomous.launch.py \
  controller:=reference gui:=true rviz:=true require_depth:=true
```

终端 2 查看状态与终点：

```bash
ros2 topic echo /r1/navigation_status
ros2 topic echo /r1/pose --once
```

状态机按以下逻辑运行：

| 状态 | 输入判断 | 输出动作 |
|---|---|---|
| `ADVANCE` | 前方墙距尚大于接近阈值 | 朝终点前进 |
| `TURN_SIDE` | 检测到墙体 | 左转，使相机朝侧方 |
| `SIDE_ADVANCE` | 侧方相机视野内可行 | 前进一小段 |
| `TURN_SCAN`、`SCAN` | 面向原行进方向，等待新深度帧 | 停车观察墙端 |
| `CROSS` | 连续两次发现前方开阔，且留出侧向余量 | 跨过墙体所在纵向区域 |
| `RETURN_GOAL` | 已越过墙体 | 朝终点前进 |
| `ALIGN_GOAL`、`DONE` | 进入终点 0.25 m 范围 | 对齐朝向并停车 |
| `FAILED` | 数据中断过久、路径受阻、无位姿进展或超时 | 停车并报告原因 |

`/r1/navigation_status` 是 JSON 文本，每 0.1 s 更新。默认终点 `(4.2,0.0)`、绕行左侧；可用 `goal_x:=... goal_y:=... side:=right` 调整，但应保持终点在活动范围内。状态机只读公开位姿和深度障碍接口；从碰撞告警只能得知本次运动已被保护层阻挡，不能用碰撞几何直接规划路线。

## 六、学生 TODO 实作

先停止参考仿真。打开 `/root/exp6/src/experiment6/experiment6_student/controller.py`，补全四个函数：

| 函数 | 输入 | 输出 | 提示 |
|---|---|---|---|
| `goal_reached` | 位姿、终点坐标 | `bool` | 使用 `math.hypot`，阈值 0.25 m |
| `front_is_safe` | 障碍汇总、所需余量 | `bool` | 数据有效、前方 `CLEAR`、距离有限且足够 |
| `turn_toward` | 当前与目标偏航角 | `Velocity` | 将角差折回 `[-π,π]`，限幅 0.35 rad/s |
| `decide_motion` | 位姿、障碍汇总、当前状态、终点 | 下一状态与 `Velocity` | 实现上表状态机；未知数据输出零速度 |

补全后重新构建并启动学生版：

```bash
cd /root/exp6
colcon build --symlink-install --packages-select experiment6_navigation
source install/setup.bash
ros2 launch experiment6_navigation experiment6_autonomous.launch.py \
  controller:=student gui:=true rviz:=true require_depth:=true
```

初始 TODO 版会持续发送零速度，状态为 `TODO`。学生补全后才可运动。每次只运行一个速度发布者。修改算法时先停止仿真，重新启动从原点开始比较。

## 七、异常与停止条件

- 深度或位姿无效、超时：先输出零速度；若持续超出宽限时间，状态进入 `FAILED`。
- 相机视野外区域不可推定安全；每次改变运动方向前主动转向，用相机观察前方。
- 转向或前进时位姿长时间不变化：停止并报告 `no_pose_progress`。
- 碰撞保护触发：停止并报告 `collision_guard_blocked`；不能当作成功避障。
- 找不到墙端、任务超时或终点不可达：停止并报告原因，重新启动仿真后调整任务参数。
- 真机适配器默认禁用。本实验不连接或驱动真机。

## 八、实验记录与报告

提交以下内容：

1. 构建成功及两个包查询结果截图。
2. Gazebo 单个 R1、蓝墙、深度图及 RViz 障碍点截图。
3. 初始、前进后、终点的位姿和前方障碍距离记录。
4. `map → odom → base_link` 的 TF 查询输出。
5. 绕行关键状态截图或日志：发现墙、侧向探查、跨越、终点停车。
6. 学生 TODO 四个函数的代码及解释；说明 `UNKNOWN` 为什么不能当作空地。
7. 至少记录一次失败或数据异常情形、停车结果与原因；如未能复现，应明确写未验证。

报告结论应区分仿真固定站姿的平面绕行与真实 R1 双足行走。不要把碰撞阻挡、启动成功或单次深度读数写成已完成自主避障的证据。
