import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='vcan1',
            description='Default can interface name'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='127',
            description='Default dronecan node id'
        ),
        DeclareLaunchArgument(
            'bitrate',
            default_value='1000000',
            description='Default can interface bitrate'
        ),
        DeclareLaunchArgument(
            'enable_nodeid_server',
            default_value='true',
            description='Enable dronecan node id server'
        ),
        DeclareLaunchArgument(
            'battery_current_offset',
            default_value='0.0',
            description='Default battery current offset'
        ),
        DeclareLaunchArgument(
            'battery_voltage_offset',
            default_value='0.0',
            description='Default battery voltage offset'
        ),
        DeclareLaunchArgument(
            'battery_calculate_percentage',
            default_value='true',
            description='Calculate percentage for battery'
        ),
        DeclareLaunchArgument(
            'battery_negative_charge',
            default_value='true',
            description='Does negative current mean charging'
        ),
        DeclareLaunchArgument(
            'battery_cell_empty',
            default_value='3.3',
            description='Default battery cell empty voltage'
        ),
        DeclareLaunchArgument(
            'battery_cell_full',
            default_value='4.2',
            description='Default battery cell full voltage'
        ),
        DeclareLaunchArgument(
            'battery_cell_num',
            default_value='6',
            description='Default cells number in battery'
        ),
        DeclareLaunchArgument(
            'battery_report_temperature',
            default_value='false',
            description='Report battery temperature'
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='ros2_dronecan',
            executable='ros2_dronecan_node',
            name='ros2_dronecan_node',
            parameters=[
                {'can_interface': LaunchConfiguration('can_interface')},
                {'node_id': LaunchConfiguration('node_id')},
                {'bitrate': LaunchConfiguration('bitrate')},
                {'enable_nodeid_server': LaunchConfiguration('enable_nodeid_server')},
                {'battery_current_offset': LaunchConfiguration('battery_current_offset')},
                {'battery_voltage_offset': LaunchConfiguration('battery_voltage_offset')},
                {'battery_calculate_percentage': LaunchConfiguration('battery_calculate_percentage')},
                {'battery_negative_charge': LaunchConfiguration('battery_negative_charge')},
                {'battery_cell_empty': LaunchConfiguration('battery_cell_empty')},
                {'battery_cell_full': LaunchConfiguration('battery_cell_full')},
                {'battery_cell_num': LaunchConfiguration('battery_cell_num')},
                {'battery_report_temperature': LaunchConfiguration('battery_report_temperature')},
            ]
        ),
    ])
