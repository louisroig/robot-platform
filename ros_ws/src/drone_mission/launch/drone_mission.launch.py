"""drone_mission_coordinator launch — single Node, mock-backend by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _coordinator_action(context, *args, **kwargs):
    mavros_backend = LaunchConfiguration('mavros_backend').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    # YAML first, dict overrides last so CLI launch args win — same order
    # as platform_hal.launch.py.
    parameters = []
    if params_file:
        parameters.append(params_file)
    parameters.append({'mavros_backend': mavros_backend})

    return [
        Node(
            package='drone_mission',
            executable='drone_mission_coordinator',
            name='drone_mission_coordinator',
            output='screen',
            parameters=parameters,
            on_exit=Shutdown(reason='drone_mission_coordinator exited'),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mavros_backend',
            default_value='mock',
            description=(
                "MAVROS backend: 'mock' (in-process drone simulator), "
                "'real' (talks to actual mavros — scaffold only at M3)."
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional YAML params file for the coordinator.',
        ),
        OpaqueFunction(function=_coordinator_action),
    ])
