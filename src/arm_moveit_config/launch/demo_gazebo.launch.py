import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r') as file:
        return yaml.safe_load(file)


def load_text(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r') as file:
        return file.read()


def generate_launch_description():
    tracking_demo = LaunchConfiguration('tracking_demo')
    use_rviz = LaunchConfiguration('rviz')
    arm_description_dir = get_package_share_directory('arm_description')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                os.path.join(arm_description_dir, 'urdf', 'robot_arm.urdf.xacro'),
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        'robot_description_semantic': load_text('arm_moveit_config', 'config/robot_arm.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('arm_moveit_config', 'config/kinematics.yaml')
    }
    robot_description_planning = {
        'robot_description_planning': load_yaml('arm_moveit_config', 'config/joint_limits.yaml')
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_description'),
                'launch',
                'gazebo.launch.py',
            ])
        ),
        launch_arguments={'rviz': 'false'}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'moveit_rviz.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_rviz),
    )

    pose_goal_follower = Node(
        package='arm_controller',
        executable='pose_goal_follower',
        output='screen',
        arguments=['--ros-args', '--log-level', 'tf2_buffer:=error'],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': True},
        ],
    )

    tracker = Node(
        package='arm_controller',
        executable='tracking_demo',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(tracking_demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument('tracking_demo', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        TimerAction(period=6.0, actions=[move_group]),
        TimerAction(period=7.0, actions=[rviz]),
        TimerAction(period=8.0, actions=[pose_goal_follower]),
        TimerAction(period=10.0, actions=[tracker]),
    ])
