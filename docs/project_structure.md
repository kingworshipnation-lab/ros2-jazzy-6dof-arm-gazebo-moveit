# 项目结构说明

仓库是标准 ROS 2 colcon 工作空间，源码集中在 `src/`。

```text
arm_workspace/
├── README.md
├── LICENSE
├── docs/
│   ├── quick_start.md
│   ├── project_structure.md
│   ├── troubleshooting.md
│   └── media/
│       ├── images/
│       └── videos/
└── src/
    ├── arm_description/
    ├── arm_moveit_config/
    └── arm_controller/
```

## arm_description

机械臂描述与 Gazebo 仿真入口。

关键内容：

- `urdf/robot_arm.urdf.xacro`: 6-DOF 机械臂模型、关节、惯量、Gazebo 插件和 `ros2_control` 配置入口。
- `config/ros2_controllers.yaml`: `joint_state_broadcaster` 和 `arm_controller` 的控制器配置。
- `launch/gazebo.launch.py`: 启动 Gazebo Harmonic、发布 `robot_description`、spawn 机械臂并加载控制器。
- `worlds/empty.sdf`: 简单 Gazebo 世界。

## arm_moveit_config

MoveIt2 配置包。

关键内容：

- `config/robot_arm.srdf`: MoveIt 语义模型，规划组为 `arm`。
- `config/kinematics.yaml`: KDL IK 插件配置。
- `config/joint_limits.yaml`: MoveIt 关节限制。
- `config/moveit_controllers.yaml`: MoveIt 到 `/arm_controller/follow_joint_trajectory` 的映射。
- `config/ompl_planning.yaml`: OMPL 规划配置。
- `launch/demo_gazebo.launch.py`: Gazebo + MoveIt + RViz + `pose_goal_follower` 联合启动。
- `rviz/moveit.rviz`: MoveIt RViz 配置。

## arm_controller

目标位姿控制与 demo 节点。

主要可执行程序：

| 可执行程序 | 作用 |
| --- | --- |
| `pose_goal_follower` | 订阅 `/arm/target_pose`，调用 MoveIt 规划并执行 |
| `send_pose_goal` | 通过参数发送一个 `PoseStamped` 目标 |
| `trajectory_target_publisher` | 按 line/circle/square 低频发布连续目标 |
| `target_pose_demo` | 固定目标点示例 |
| `target_pose_planner` | 早期规划测试工具 |
