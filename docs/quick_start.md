# 快速开始

本文给出从干净工作空间到运行 demo 的最短路径。

## 1. 环境准备

```bash
source /opt/ros/jazzy/setup.bash
cd /home/brant/arm_workspace
rosdep install --from-paths src --ignore-src -r -y
```

## 2. 编译

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## 3. 启动 Gazebo

```bash
ros2 launch arm_description gazebo.launch.py
```

检查控制器：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list | grep follow_joint_trajectory
```

## 4. 启动 MoveIt + Gazebo

```bash
ros2 launch arm_moveit_config demo_gazebo.launch.py
```

如果只想终端验证：

```bash
ros2 launch arm_moveit_config demo_gazebo.launch.py rviz:=false
```

## 5. 发送单个末端目标

```bash
ros2 run arm_controller send_pose_goal --ros-args \
  -p x:=0.35 \
  -p y:=0.0 \
  -p z:=0.45 \
  -p qw:=1.0
```

## 6. 运行末端跟踪 demo

```bash
ros2 run arm_controller trajectory_target_publisher --ros-args \
  -p mode:=circle \
  -p radius:=0.05 \
  -p center_x:=0.35 \
  -p center_y:=0.0 \
  -p center_z:=0.45 \
  -p rate:=0.05 \
  -p points_per_cycle:=4
```

当前跟踪方式是低频 MoveIt 规划执行，不是高频笛卡尔伺服。推荐 `rate:=0.05` 到 `0.1`，避免目标发布快于轨迹执行。
