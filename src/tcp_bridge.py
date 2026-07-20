#!/usr/bin/env python3

import json
import math
import socket
import threading
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan


LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
GYRO_SENSITIVITY_FACTOR = (17.5 * math.pi / 180.0)  # mrad/s per digit (LSM6DS33 at ±500 dps range → 17.5 mrad/s per digit)
ACCELEROMETER_SENSITIVITY_FACTOR = (0.061 * 9.81)  # mm/s² per digit

@dataclass
class VehicleData:
    stamp_sec: float
    x: float
    y: float
    yaw: float
    center_velocity: float
    left_velocity: float
    right_velocity: float
    imu_angular_velocity_z: float
    imu_linear_acceleration_x: float


class TcpBridge(Node):

    def __init__(self) -> None:
        super().__init__("tcp_bridge")

        # Parameters
        self.host: str = self.declare_parameter("host", "0.0.0.0").value
        self.port: int = self.declare_parameter("port", 8888).value
        self.wheel_radius: float = self.declare_parameter("wheel_radius", 0.018).value
        self.wheel_separation: float = self.declare_parameter("wheel_separation", 0.075).value
        self.use_source_timestamp: bool = self.declare_parameter("use_source_timestamp", True).value
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            "odom",
            10
        )
        self.imu_pub = self.create_publisher(
            Imu,
            "imu",
            10
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            "joint_states",
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            "scan",
            10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan_raw",
            self.scan_callback,
            10
        )

        # State
        self.client_socket: Optional[socket.socket] = None
        self.socket_lock = threading.Lock()

        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_joint_state_stamp_sec: Optional[float] = None

        # TCP server thread
        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()

    # TCP server loop
    def server_loop(self) -> None:

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)

        self.get_logger().info(f"TCP server listening on {self.port}")

        while rclpy.ok():
            client, addr = server.accept()

            with self.socket_lock:
                self.client_socket = client

            self.get_logger().info(f"Client connected: {addr}")

            buffer = ""

            try:
                while True:

                    data = client.recv(1024)
                    if not data:
                        break

                    buffer += data.decode()
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n",1)
                        line = line.strip()
                        if line:
                            self.process_vehicle_line(line)

            except OSError as exc:
                self.get_logger().error(f"Socket error: {exc}")
            finally:
                self.get_logger().info("Client disconnected")
                client.close()
                with self.socket_lock:
                    self.client_socket = None

    # Scan callback
    def scan_callback(self,msg: LaserScan) -> None:

        if msg.angle_increment < 0.0:
            out = LaserScan()
            out.header = msg.header

            # Reverse the angles
            out.angle_min = msg.angle_max
            out.angle_max = msg.angle_min
            out.angle_increment = -msg.angle_increment

            # Metadata
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max

            out.ranges = msg.ranges[::-1]

            if msg.intensities:
                out.intensities = msg.intensities[::-1]
        else:
            out = msg  

        self.scan_pub.publish(out)

    # cmd_vel callback
    def cmd_vel_callback(self, msg: TwistStamped) -> None:

        with self.socket_lock:
            sock = self.client_socket

        if sock is None:
            return

        payload = json.dumps(
            {
                "linear": msg.twist.linear.x * 1000.0,
                "angular": msg.twist.angular.z * 1000.0
            }
        ) + "\n"

        try:
            sock.sendall(
                payload.encode()
            )
        except OSError as exc:
            self.get_logger().error(f"Send failed: {exc}")

    # Packet processing
    def process_vehicle_line(self, line: str) -> None:

        data = self.parse_vehicle_payload(line)
        if data is None:
            return

        self.publish_odom(data)
        self.publish_imu(data)
        self.publish_joint_states(data)

    def parse_vehicle_payload(self, line: str) -> Optional[VehicleData]:

        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return None

        if payload.get("type") != "vehicle_data":
            return None

        try:
            # Source packets use millimeters and milliradians.
            # All values are converted to SI units.
            return VehicleData(
                stamp_sec=float(payload["t"]) / 1000.0,
                x=float(payload["x"]) / 1000.0,
                y=float(payload["y"]) / 1000.0,
                yaw=float(payload["h"]) / 1000.0,
                center_velocity=float(payload["c"]) / 1000.0,
                left_velocity=float(payload["l"]) / 1000.0,
                right_velocity=float(payload["r"]) / 1000.0,
                imu_angular_velocity_z=(float(payload.get("tz", 0.0)) * GYRO_SENSITIVITY_FACTOR / 1000.0),
                imu_linear_acceleration_x=(float(payload.get("ax", 0.0)) * ACCELEROMETER_SENSITIVITY_FACTOR / 1000.0)
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Ignoring malformed packet: {exc}")
            return None

    # Publishers
    def publish_odom(self, data: VehicleData) -> None:

        odom = Odometry()
        odom.header.stamp = (rclpy.time.Time(seconds=data.stamp_sec).to_msg())
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = data.x
        odom.pose.pose.position.y = data.y

        yaw = data.yaw
        odom.pose.pose.orientation.z = (math.sin(yaw * 0.5))

        odom.pose.pose.orientation.w = (math.cos(yaw * 0.5))

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 4e-2
        odom.pose.covariance[7] = 4e-2
        odom.pose.covariance[35] = 1e4

        odom.twist.twist.linear.x = (data.center_velocity)

        odom.twist.twist.angular.z = ((data.right_velocity - data.left_velocity) / self.wheel_separation)

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 1e-2
        odom.twist.covariance[7] = 10000000
        odom.twist.covariance[35] = 10

        self.odom_pub.publish(odom)

    def publish_imu(self, data: VehicleData) -> None:

        imu = Imu()
        imu.header.stamp = (rclpy.time.Time(seconds=data.stamp_sec).to_msg())
        imu.header.frame_id = "base_imu"

        imu.angular_velocity.z = (data.imu_angular_velocity_z)

        imu.linear_acceleration.x = (data.imu_linear_acceleration_x)

        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e-4
        ]

        imu.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e-4
        ]

        imu.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1
        ]

        self.imu_pub.publish(imu)

    def publish_joint_states(self, data: VehicleData) -> None:

        if self.last_joint_state_stamp_sec is None:
            dt = 0.0
        else:
            dt = (data.stamp_sec - self.last_joint_state_stamp_sec)

            dt = max(dt,0.0)

        self.last_joint_state_stamp_sec = (data.stamp_sec)

        left_wheel_angular_velocity = (data.left_velocity / self.wheel_radius)
        right_wheel_angular_velocity = (data.right_velocity / self.wheel_radius)

        self.left_wheel_position += (left_wheel_angular_velocity * dt)

        self.right_wheel_position += (right_wheel_angular_velocity * dt)

        joint_state = JointState()

        joint_state.header.stamp = (self.get_clock().now().to_msg())

        joint_state.name = [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT]

        joint_state.position = [self.left_wheel_position, self.right_wheel_position]

        joint_state.velocity = [left_wheel_angular_velocity, right_wheel_angular_velocity]

        self.joint_state_pub.publish(joint_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()