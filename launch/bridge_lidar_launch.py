#!/usr/bin/env python3
"""Launch only TCP bridge and standalone LiDAR Webots controller."""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def _arg_to_bool(arg: str) -> bool:
    """Convert common string representations of booleans to actual bools."""
    if isinstance(arg, bool):
        return arg
    return arg.lower() in ('true', 'yes', '1', 'ok')


def _launch_setup(context) -> List[LaunchDescriptionEntity]:
    """Resolve launch configurations and build the list of launch actions."""
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    lidar_robot_name = LaunchConfiguration('lidar_robot_name').perform(context)
    webots_port = LaunchConfiguration('webots_port').perform(context)
    lidar_description = LaunchConfiguration('lidar_description').perform(context)
    launch_supervisor = _arg_to_bool(LaunchConfiguration('launch_supervisor').perform(context))
    launch_ekf = _arg_to_bool(LaunchConfiguration('launch_ekf').perform(context))
    publish_zumo_tf = _arg_to_bool(LaunchConfiguration('publish_zumo_tf').perform(context))

    actions = []

    tcp_bridge = Node(
        package='flagship',
        executable='tcp_bridge.py',
        name='tcp_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    actions.append(tcp_bridge)

    lidar_controller = WebotsController(
        robot_name=lidar_robot_name,
        port=webots_port,
        parameters=[
            {'robot_description': lidar_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    actions.append(lidar_controller)

    if launch_supervisor:
        actions.append(
            Ros2SupervisorLauncher(port=webots_port)
        )

    if launch_ekf:
        ekf_config = LaunchConfiguration('ekf_config').perform(context)
        actions.append(
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {'use_sim_time': use_sim_time}
                ]
            )
        )

    if publish_zumo_tf:
        zumo_urdf = os.path.join(
            get_package_share_directory('flagship'),
            'resource',
            'Zumo32U4.urdf'
        )
        try:
            with open(zumo_urdf, 'r') as f:
                robot_desc = f.read()
        except Exception:
            robot_desc = ''

        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {'robot_description': robot_desc},
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Create launch description exposing user-configurable arguments."""
    pkg_share = get_package_share_directory('flagship')
    
    # Define default file paths
    zumo_urdf_default = os.path.join(pkg_share, 'resource', 'Zumo32U4.urdf')
    ekf_config_default = os.path.join(pkg_share, 'config', 'ekf.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Flag to enable use_sim_time.'
        ),
        DeclareLaunchArgument(
            'lidar_robot_name',
            default_value='FlagShip',
            description='Name of standalone Webots Robot node for native ROS 2 LiDAR.'
        ),
        DeclareLaunchArgument(
            'webots_port',
            default_value='1234',
            description='Webots controller port used by the standalone LiDAR robot.'
        ),
        DeclareLaunchArgument(
            'launch_supervisor',
            default_value='true',
            description='Enable/disable the Ros2Supervisor node that publishes /clock.'
        ),
        DeclareLaunchArgument(
            'launch_ekf',
            default_value='true',
            description='Enable/disable robot_localization EKF node.'
        ),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=ekf_config_default,
            description='Path to robot_localization EKF parameter file.'
        ),
        DeclareLaunchArgument(
            'publish_zumo_tf',
            default_value='true',
            description='Enable/disable TF publishing for Zumo tree via robot_state_publisher.'
        ),
        DeclareLaunchArgument(
            'lidar_description',
            default_value=zumo_urdf_default,
            description='Path to URDF defining ROS 2 LiDAR device mapping.'
        ),
    ]

    lds = LaunchDescription(launch_args)
    lds.add_action(OpaqueFunction(function=_launch_setup))

    return lds