# 接口与坐标约定

## 位姿

`/r1/odom` 的 `header.frame_id=odom`，`child_frame_id=base_link`。pose 是机身原点在 odom 中的位姿，twist 是机身坐标系速度。姿态采用单位四元数，位置单位为米，速度单位为 m/s 和 rad/s。`/r1/pose` 与同一帧 odom 的 pose、时间戳一致。

仿真插件发布的原始位姿为 Gazebo 世界坐标中的模型根坐标。当前教学场景明确约定 map、odom 与 Gazebo 世界坐标原点和轴向重合，因此 `map -> odom` 为单位静态变换。更换定位方案、原点或坐标轴时必须同步调整，不得继续默认重合。

`odom -> base_link` 只由位姿节点发布。机器人固定关节与相机安装变换由 robot_state_publisher 发布。不要同时启动实验五的 `world_to_r1_base`，也不要由多个节点重复发布同一条动态 TF。

## 速度

公开输入 `/r1/cmd_vel` 使用 Humble 的 `geometry_msgs/msg/Twist`。支持 `linear.x`、`linear.y`、`angular.z`；其他轴必须为0。非法值使当前指令失效。正 X 向前、正 Y 向左、正偏航向左转。

仿真适配器输出到 `/r1/sim/cmd_vel`，经 ros_gz_bridge 进入 Fortress 插件。插件以固定站姿执行平面运动，发布真实模型反馈。SDK 后端独立接收公开输入并调用真实 R1 运动服务，默认不参与仿真启动。

当前只允许一个公开速度指令来源。接入 Nav2 时将其速度输出重映射到 `/r1/cmd_vel`；如同时保留遥控和自动控制，应在后续增加明确的指令仲裁，而不是同时发布。

## 深度和障碍

输入为 `/r1/depth/image` 与 `/r1/depth/camera_info`，采用 sensor-data QoS。处理使用同分辨率内参、同光学帧及与图像时间对应的 TF。相机光学轴为右、下、前；输出点云为 `base_link` 中的前、左、上。

`ObstacleSummary.header.stamp` 是输入深度图的时间戳，`header.frame_id=base_link`。正常数据按图像频率更新；数据丢失、TF不可用、帧或尺寸不匹配时发布 UNKNOWN。看门狗使用单调墙钟检测接收中断，图像年龄用 ROS 时钟检测。空的未知数据不会用于清除旧障碍点云。

数组顺序为 LEFT、FRONT、RIGHT。分区由机身水平角划分，前方默认±10°，左右覆盖至±60°，实际覆盖仍受相机视场限制。`valid_ratio` 的分母包含无效像素对应的采样射线。一个明确的近距离障碍即使出现在低有效率分区，也保留 BLOCKED 状态。

`data_valid` 只有三个分区都能形成已知状态时才为真。运动适配器 `require_depth=true` 时将该字段作为数据可用条件；BLOCKED 本身不会在适配器中生成绕行动作。

## 仿真碰撞与感知分离

场景生成器读取世界中的静态盒体，为运动插件生成碰撞检查参数；感知节点不读取墙体坐标或碰撞参数。碰撞参数只约束执行层，不可用来代替相机识别或直接生成固定绕行路线。

Nav2 后续还需要预制地图、占用轮廓、代价地图和规划控制配置。本工程提供其所需的基础话题，但没有声称已经完成导航集成。
