"""M1 HAL stack launch: safety_monitor + motor_driver + imu_driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _motor_driver_action(context, *args, **kwargs):
    gpio_backend = LaunchConfiguration('gpio_backend').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    parameters = [{'gpio_backend': gpio_backend}]
    if params_file:
        parameters.append(params_file)

    return [
        Node(
            package='platform_hal',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=parameters,
        ),
    ]


def _imu_driver_action(context, *args, **kwargs):
    imu_backend = LaunchConfiguration('imu_backend').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    parameters = [{'imu_backend': imu_backend}]
    if params_file:
        parameters.append(params_file)

    return [
        Node(
            package='platform_hal',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpio_backend',
            default_value='lgpio',
            description="motor_driver GPIO backend: 'lgpio' on hardware, 'mock' on dev",
        ),
        DeclareLaunchArgument(
            'imu_backend',
            default_value='ism330',
            description="imu_driver backend: 'ism330' on hardware, 'mock' on dev",
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional YAML params file shared by motor_driver and imu_driver',
        ),
        Node(
            package='platform_hal',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
        ),
        OpaqueFunction(function=_motor_driver_action),
        OpaqueFunction(function=_imu_driver_action),
    ])
