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
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'robot_arm.urdf.xacro')

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
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

    planning_pipelines = load_yaml('arm_moveit_config', 'config/ompl_planning.yaml')
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 3.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    moveit_controllers = load_yaml('arm_moveit_config', 'config/moveit_controllers.yaml')
    planning_scene_monitor = load_yaml(
        'arm_moveit_config',
        'config/planning_scene_monitor_parameters.yaml',
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'tf2_buffer:=error'],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group,
    ])
