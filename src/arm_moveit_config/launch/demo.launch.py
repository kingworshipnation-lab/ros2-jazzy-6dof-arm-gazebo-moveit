import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    arm_description_dir = get_package_share_directory('arm_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time,
                'ignore_timestamp': True,
            },
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'moveit_rviz.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz,
        TimerAction(period=2.0, actions=[pose_goal_follower]),
    ])
