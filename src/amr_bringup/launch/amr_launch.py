"""Launch the AMR serial bridge, robot description, and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the basic AMR bringup launch description."""
    package_share = get_package_share_directory('amr_bringup')
    urdf_path = os.path.join(package_share, 'urdf', 'amr.urdf')
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    serial_bridge = Node(
        package='amr_bringup',
        executable='amr_serial_bridge',
        name='amr_serial_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'wheel_radius': 0.0435,
            'wheel_base': 0.20,
            'encoder_ppr': 4400,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'cmd_linear_gain': 1.5,
            'cmd_angular_gain': 1.3,
            'cmd_min_linear': 0.12,
            'cmd_min_angular': 0.35,
            'cmd_max_linear': 0.6,
            'cmd_max_angular': 1.5,
            'cmd_deadband': 0.01,
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/esp32'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        serial_bridge,
        rviz,
    ])
