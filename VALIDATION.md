# 验证记录

本文件记录本地实际执行的检查；没有执行的项目不能视为已通过。

## 已执行

- Windows Python 3.12 + NumPy 2.5.1：Python 语法、package.xml 解析及16项数值/场景单元测试通过。
- 覆盖32FC1/16UC1、字节序、行填充、损坏缓冲区、内参射线、相机到机身变换、地面过滤、分区方向、无效数据、低有效率近障碍、四元数归一化、速度限幅及场景生成保护。
- Ubuntu 22.04（WSL）+ GCC 11 + Fortress 6.18.0：仿真插件与测试程序编译通过；8项平面运动/连续碰撞几何检查通过。
- Fortress 真实无界面运行时测试通过：静态站姿模型实际平移、实际位姿与速度反馈、正偏航左转、墙前阻挡、倒退离墙、指令接收超时停车。
- 基于实验五参考提交生成的完整实验六 SDF 通过 `ign sdf -k` 校验。
- 完整实验五 R1 模型、原有网格、蓝色墙壁与真实深度传感器的无界面软件渲染测试通过：收到17帧深度图，机器人实际前移0.21 m，中心光轴深度从2.40476 m变为2.18172 m，确认相机随模型运动更新。
- ROS 2 Humble 完整 colcon 构建通过，包含实验五依赖安装、Fortress 插件、自定义消息的 C/C++/Python 类型支持及 Python 节点安装。
- 独立 ROS_DOMAIN_ID=173 中的3项 ROS 节点接口测试通过：实际位姿转发、深度点云/分区结果、速度限幅与指令超时停车。colcon/CTest 的2项原生测试也通过，无失败或跳过。
- 完整 ROS 2 + R1 + 深度渲染链路联调通过（独立 Domain ID=174）：收到33条位姿与14帧障碍点云；通过公开 `/r1/cmd_vel` 前移0.1575 m，前方水平障碍距离从2.452754 m下降到2.295074 m，最后一帧包含1520个有效障碍点，点云坐标帧为 base_link。
- 开启 `BUILD_UNITREE_ADAPTER=ON` 后，R1 SDK 适配器编译、链接和安装通过。使用官方 unitree_sdk2 提交 `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36`，没有连接或驱动任何真机。
- 真机后端未显式启用时的启动保护测试通过：程序在初始化 SDK 通道前拒绝启动，返回明确说明。

## 待执行或待更新

- 用户实验虚拟机中的深度输入中断保护、指令接收超时停车、墙体不同视角的观测及端到端自主绕行。
- R1 真机连接、固件兼容性、运动状态与指令执行。

ROS 2 集成测试和 Fortress 无界面测试源码已提供。离线检查脚本只执行数值测试，不会自动发布运动指令；ROS 接口测试要求显式指定独立的 ROS_DOMAIN_ID=173。

## Experiment2 教学虚拟机联调（2026-10-15）

- `/root/exp6` 中的 `experiment5_depth` 和 `experiment6_navigation` 均由 `colcon build --symlink-install` 成功构建，包索引可查询到两包。
- 教学平台 ZIP 解压后，`scripts/*.py` 缺少执行权限，初次 launch 报 `pose_node.py not found on the libexec directory`。执行 `chmod +x /root/exp6/src/experiment6/scripts/*.py` 后重新启动成功；该问题需在教学资源打包时修复或在部署步骤中保留权限恢复命令。
- `experiment6_interfaces.launch.py gui:=true rviz:=true software_rendering:=true require_depth:=true` 启动后，Gazebo GUI 显示单个 R1 与蓝色墙壁，深度图窗口有实时画面；RViz 启动并显示 `Global Status: Ok`。
- 节点列表包含位姿、运动、深度处理、桥接和机器人状态发布节点，未在节点列表中发现实验五运行节点或重复机器人节点。`/r1/pose`、`/r1/odom`、`/r1/obstacle_summary`、`/r1/obstacle_points`、`/r1/cmd_vel`、`/tf`、`/tf_static` 等话题存在。
- 深度图频率约 9 Hz。`tf2_echo map base_link` 在发现静态变换后，持续返回单位位姿变换。
- 初始 `/r1/pose` 为 `(x,y)=(0,0)`；初始前方障碍距离约 2.453 m，`data_valid=true`、`status=complete`，障碍点云计数为 1440。
- 执行 `drive_for.py --vx 0.15 --seconds 2` 后，`x=0.2985 m`，前方距离降至约 2.154 m，障碍点计数为 1600，证明位姿、运动及深度输出随前进同步变化。
- 执行 `drive_for.py --yaw-rate 0.3 --seconds 1` 后，位置基本不变，姿态四元数 `z≈0.13905, w≈0.99029`，对应正偏航约 0.279 rad。之后执行 `drive_for.py --seconds 0.5` 显式停车。
- GitHub Actions 的 `Build and interface contracts` 工作流在提交 `045178b` 上显示成功。虚拟机图形远程会话在后续读取运动状态时响应变慢，未取得该命令的终端结果；因此本节不将运动状态输出、深度中断和超时停车列为通过。

## 自主绕墙开发环境验证（2026-10-16）

- 在 WSL Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 中使用完整实验五 R1 模型、深度相机和蓝墙，启动 `experiment6_autonomous.launch.py gui:=false rviz:=false require_depth:=true`。参考节点只通过 `/r1/odom`、`/r1/obstacle_summary` 决定运动，碰撞告警仅用于失败停车。
- 原实验五地面为 12 m × 8 m。初次绕行试验到达墙端时，部分深度光线离开地面，导致分区有效率低于阈值，导航安全停车。将**仅在实验六临时世界中**生成的地面扩展为 16 m × 16 m，原实验五世界文件和机器人活动边界均不改动；阈值恢复原先的 `min_valid_ratio=0.35`。
- 重跑后依次进入 `ADVANCE`、`TURN_SIDE`、`SIDE_ADVANCE`、`TURN_SCAN`、`SCAN`、`CROSS`、`RETURN_GOAL`、`ALIGN_GOAL`、`DONE`。`CROSS` 起点约 `(1.061, 2.744)`，越墙后进入 `RETURN_GOAL` 的位姿约 `(3.320, 2.750)`；到达终点时约 `(4.131, 0.220, -0.068 rad)`，与目标 `(4.2, 0.0)` 相距约 0.231 m，小于 0.25 m 停止阈值。
- 终点时 `/r1/navigation_status` 持续报告 `DONE/goal_reached`，`/r1/cmd_vel` 为全零；该次运行未见 `collision_guard_blocked`。这证明完整感知—决策—运动闭环在本地目标场景跑通，不替代教学虚拟机复现。
- 19 项 Python 核心/导航测试、2 项原生 CTest、3 项 ROS 2 接口集成测试通过。额外三项自动绕墙测试覆盖默认场景到达、未知深度停车及非法参数。完整仿真演示不计入单元测试数量。
