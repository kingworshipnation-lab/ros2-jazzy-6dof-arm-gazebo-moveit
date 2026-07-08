# 常见问题排查

## Gazebo 中模型不显示

先确认 xacro 可以展开：

```bash
xacro src/arm_description/urdf/robot_arm.urdf.xacro > /tmp/robot_arm_check.urdf
```

再检查 `gazebo.launch.py` 输出中是否出现：

```text
Entity creation successful.
```

如果没有，清理生成目录后重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## controller inactive

检查控制器状态：

```bash
ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster active
arm_controller active
```

如果 inactive，重点检查：

- `gz_ros2_control` 插件是否加载；
- `arm_description/config/ros2_controllers.yaml` 是否被安装；
- 控制器中的 joint 名称是否和 URDF 完全一致。

## MoveIt 找不到 controller

检查 action 是否存在：

```bash
ros2 action list | grep follow_joint_trajectory
```

期望：

```text
/arm_controller/follow_joint_trajectory
```

如果 action 存在但 MoveIt 仍报错，检查：

- `arm_moveit_config/config/moveit_controllers.yaml`;
- controller 名称是否为 `arm_controller`;
- joint 列表是否为同一组 6 个关节。

## IK failed

常见原因：

- 目标超出可达空间；
- 目标姿态约束过强；
- 起始状态没有从 `/joint_states` 正确更新；
- 目标点碰撞或接近关节极限。

先用保守目标验证：

```bash
ros2 run arm_controller send_pose_goal --ros-args \
  -p x:=0.35 \
  -p y:=0.0 \
  -p z:=0.45 \
  -p qw:=1.0
```

当前 `pose_goal_follower` 默认使用 position-only target，这样 Gazebo demo 更稳；如果要强制完整姿态目标，可设置 `position_only_mode:=false`，但需要更仔细地选择可达姿态。

## 没有 joint_states

检查：

```bash
ros2 topic list | grep joint_states
ros2 topic echo /joint_states --once
```

如果没有数据：

- 确认 `joint_state_broadcaster` 为 active；
- 确认 Gazebo 没有暂停；
- 确认机器人成功 spawn；
- 确认 `use_sim_time` 和 `/clock` 正常发布。

## Gazebo 图形窗口闪退或黑屏

一些 NVIDIA / Wayland 环境会出现 `libEGL warning`。如果 Gazebo 进程没有退出、控制器 active、`/joint_states` 正常发布，这类 warning 通常不影响仿真。

如果窗口直接闪退，可尝试：

```bash
QT_QPA_PLATFORM=xcb ros2 launch arm_description gazebo.launch.py
```

或先用无 RViz 模式验证核心链路：

```bash
ros2 launch arm_moveit_config demo_gazebo.launch.py rviz:=false
```

## install 符号链接失效

使用 `--symlink-install` 时，移动或删除源码文件后 `install/` 中可能留下失效链接。清理生成目录即可：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

不要删除 `src/`。

## 轨迹跟踪速度太快

`trajectory_target_publisher` 发布的是离散目标点，`pose_goal_follower` 会对每个目标调用 MoveIt 规划与执行。推荐：

```bash
-p rate:=0.05
```

如果提高到 `0.5Hz`，发布器仍会发布目标，但执行链路可能来不及完成每一个点。需要高频实时笛卡尔跟踪时，应接入 MoveIt Servo。
