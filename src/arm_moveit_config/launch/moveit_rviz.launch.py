import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    moveit_config_dir = get_package_share_directory('arm_moveit_config')
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_config_dir, 'rviz', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rviz,
    ])
