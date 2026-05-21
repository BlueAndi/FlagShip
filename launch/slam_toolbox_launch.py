"""Launch SLAM Toolbox online async mode for Webots simulation."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Create launch description for slam_toolbox online async mode."""

    slam_params_file = os.path.join(
        get_package_share_directory('FlagShip'),
        'config',
        'slam_toolbox_params.yaml'
    )

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated time (e.g., from Webots Ros2Supervisor).'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Path to the slam_toolbox YAML parameter file.'
        ),
    ]

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription(declare_args + [slam_launch])
