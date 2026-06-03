"""Launch AMR mapping with selectable lidar and SLAM Toolbox."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    EqualsSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_share = get_package_share_directory('amr_bringup')

    slam_share = get_package_share_directory('slam_toolbox')

    robot_description = ParameterValue(
        Command([
            'cat ',
            os.path.join(package_share, 'urdf', 'amr.urdf')
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
    )

    serial_bridge = Node(
        package='amr_bringup',
        executable='amr_serial_bridge',
        name='amr_serial_bridge',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('esp_port'),
                'baud': 115200,
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
            }
        ],
    )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[
            {
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'inverted': False,
                'scan_mode': 'Sensitivity',
            }
        ],
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('lidar'),
                'rplidar'
            )
        ),
    )

    ldlidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[
            {
                'product_name': LaunchConfiguration('lidar_model'),
                'topic_name': 'scan',
                'frame_id': 'laser',
                'port_name': LaunchConfiguration('lidar_port'),
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
            }
        ],
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('lidar'),
                'ldlidar'
            )
        ),
    )

    laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '0.10',
            '0',
            '0.12',
            '0',
            '3.14159',
            '3.14159',
            'base_link',
            'laser'
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_share,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                package_share,
                'config',
                'slam_toolbox_params.yaml'
            )
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        ),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'lidar',
            default_value='rplidar'
        ),

        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/lidar'
        ),

        DeclareLaunchArgument(
            'esp_port',
            default_value='/dev/esp32'
        ),

        DeclareLaunchArgument(
            'lidar_model',
            default_value='LDLiDAR_LD19'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        robot_state_publisher,
        serial_bridge,
        rplidar,
        ldlidar,
        laser_transform,
        slam_toolbox,
        rviz,
    ])
