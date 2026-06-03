"""Launch AMR navigation with localization and the Nav2 servers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the navigation launch description."""
    package_share = get_package_share_directory('amr_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    robot_description = ParameterValue(
        Command(['cat ', os.path.join(package_share, 'urdf', 'amr.urdf')]),
        value_type=str,
    )

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
        }],
    )
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('lidar'), 'rplidar')),
    )
    ldlidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': LaunchConfiguration('lidar_model'),
            'topic_name': 'scan',
            'frame_id': 'laser',
            'port_name': LaunchConfiguration('lidar_port'),
            'port_baudrate': 230400,
            'laser_scan_dir': True,
        }],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('lidar'), 'ldlidar')),
    )
    laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.10', '0', '0.12', '0', '3.14159', '3.14159', 'base_link', 'laser'],
    )

    nav_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params, {
                'yaml_filename': LaunchConfiguration('map'),
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        # arguments=['--display-config', '/home/khazg/amr_ws/src/my_nav/my_nav.rviz'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/maps/my_map.yaml')),
        DeclareLaunchArgument('lidar', default_value='rplidar'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('esp_port', default_value='/dev/esp32'),
        DeclareLaunchArgument('lidar_model', default_value='LDLiDAR_LD19'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        serial_bridge,
        rplidar,
        ldlidar,
        laser_transform,
        *nav_nodes,
        lifecycle_manager,
        rviz,
    ])
