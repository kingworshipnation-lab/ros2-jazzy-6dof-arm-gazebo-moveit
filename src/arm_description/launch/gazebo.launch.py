import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_description_dir = get_package_share_directory('arm_description')
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'robot_arm.urdf.xacro')
    controllers_file = os.path.join(arm_description_dir, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(arm_description_dir, 'config', 'arm.rviz')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file,
                ' ros2_control_config:=',
                controllers_file,
            ]),
            value_type=str,
        )
    }

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': True,
                'ignore_timestamp': True,
            },
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'six_dof_arm',
            '-allow_renaming', 'true',
            '-z', '0.0',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(arm_description_dir, 'worlds', 'empty.sdf'),
            description='Gazebo Sim world file',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz together with Gazebo',
        ),
        gz_launch,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=4.0, actions=[arm_controller_spawner]),
        rviz,
    ])
