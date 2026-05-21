"""Launch only TCP bridge and standalone LiDAR Webots controller."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def _arg_to_bool(arg):
    if isinstance(arg, bool):
        return arg

    if arg.lower() in ('true', 'yes', '1', 'ok'):
        return True

    return False


def _launch_setup(context) -> list[LaunchDescriptionEntity]:
    """Create runtime launch actions with resolved arguments."""
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    lidar_robot_name = LaunchConfiguration('lidar_robot_name').perform(context)
    webots_port = LaunchConfiguration('webots_port').perform(context)
    webots_ip = LaunchConfiguration('webots_ip').perform(context) or None
    launch_supervisor = _arg_to_bool(LaunchConfiguration('launch_supervisor').perform(context))
    launch_ekf = _arg_to_bool(LaunchConfiguration('launch_ekf').perform(context))
    publish_zumo_tf = _arg_to_bool(LaunchConfiguration('publish_zumo_tf').perform(context))

    tcp_bridge = Node(
        package='FlagShip',
        executable='tcp_bridge.py',
        name='tcp_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    lidar_description = LaunchConfiguration('lidar_description').perform(context)
    lidar_controller = WebotsController(
        robot_name=lidar_robot_name,
        port=webots_port,
        ip_address=webots_ip,
        parameters=[
            {'robot_description': lidar_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    actions = [tcp_bridge]

    if launch_supervisor:
        actions.append(
            Ros2SupervisorLauncher(
                port=webots_port,
                ip_address=webots_ip
            )
        )

    actions.append(lidar_controller)

    if launch_ekf:
        actions.append(
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('ekf_config').perform(context),
                    {'use_sim_time': use_sim_time}
                ]
            )
        )

    if publish_zumo_tf:
        zumo_urdf = os.path.join(
            get_package_share_directory('FlagShip'),
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
    """Create launch description for bridge + LiDAR controller only."""
    lidar_description_default = os.path.join(
        get_package_share_directory('FlagShip'),
        'resource',
        'lidar_webots.urdf'
    )

    ekf_config_default = os.path.join(
        get_package_share_directory('FlagShip'),
        'config',
        'ekf.yaml'
    )

    launch_args = [
        DeclareLaunchArgument(
            'lidar_robot_name',
            default_value='FlagShip',
            description='Name of standalone Webots Robot node for native ROS2 LiDAR.'
        ),
        DeclareLaunchArgument(
            'lidar_description',
            default_value=lidar_description_default,
            description='Path to URDF defining ROS2 LiDAR device mapping.'
        ),
        DeclareLaunchArgument(
            'publish_zumo_tf',
            default_value='true',
            description='Enable/disable TF publishing for Zumo tree via robot_state_publisher.'
        ),
        DeclareLaunchArgument(
            'launch_supervisor',
            default_value='true',
            description='Enable/disable the Ros2Supervisor node that publishes /clock.'
        ),
        DeclareLaunchArgument(
            'webots_port',
            default_value='1234',
            description='Webots controller port used by the standalone LiDAR robot.'
        ),
        DeclareLaunchArgument(
            'webots_ip',
            default_value='',
            description='Webots controller IP address used by the standalone LiDAR robot.'
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
            'use_sim_time',
            default_value='true',
            description='Flag to enable use_sim_time.'
        ),
    ]

    lds = LaunchDescription(launch_args)
    lds.add_action(OpaqueFunction(function=_launch_setup))

    return lds
