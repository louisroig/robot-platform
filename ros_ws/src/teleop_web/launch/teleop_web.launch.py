"""Teleop launch: rosbridge WebSocket + static HTTP server for the joystick page."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    web_dir = os.path.join(get_package_share_directory('teleop_web'), 'web')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_port',
            default_value='9090',
            description='rosbridge WebSocket port',
        ),
        DeclareLaunchArgument(
            'http_port',
            default_value='8000',
            description='Static HTTP server port for the joystick page',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': LaunchConfiguration('ws_port')}],
        ),
        ExecuteProcess(
            cmd=[
                sys.executable, '-m', 'http.server',
                LaunchConfiguration('http_port'),
                '--bind', '0.0.0.0',
                '--directory', web_dir,
            ],
            output='screen',
        ),
    ])
