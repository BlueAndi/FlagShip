#!/usr/bin/env python3

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState


HOST = "0.0.0.0"
PORT = 8888


LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
WHEEL_RADIUS = 0.018


class TcpBridge(Node):

    def __init__(self):

        super().__init__("tcp_bridge")

        # The incoming heading is defined in the simulator's convention, which
        # is rotated relative to ROS's frame.
        self.declare_parameter("heading_offset_rad", -math.pi / 2.0)
        self.heading_offset_rad = float(
            self.get_parameter("heading_offset_rad").value
        )
        self._heading_offset_cos = math.cos(self.heading_offset_rad)
        self._heading_offset_sin = math.sin(self.heading_offset_rad)

        self.client_socket = None
        self.declare_parameter("use_source_timestamp", True)


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

        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_joint_state_stamp_sec = None

        # wheel separation (meters) used to compute angular velocity from
        # left/right wheel linear speeds.
        self.wheel_separation = 0.075

        # ROS subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Start TCP server thread
        self.server_thread = threading.Thread(
            target=self.server_loop,
            daemon=True
        )

        self.server_thread.start()

    def server_loop(self):

        server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server.bind((HOST, PORT))
        server.listen(1)

        self.get_logger().info(
            f"TCP server listening on {PORT}"
        )

        while rclpy.ok():

            client, addr = server.accept()

            self.client_socket = client

            self.get_logger().info(
                f"Client connected: {addr}"
            )

            try:
                buffer = ""

                while True:
                    data = client.recv(1024)

                    if not data:
                        break

                    buffer += data.decode()

                    # newline framed packets
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)

                        line = line.strip()

                        if not line:
                            continue

                        # self.get_logger().info(
                        #     f"RX: {line}"
                        # )

                        self.publish_vehicle_odometry(line)

            except Exception as e:

                self.get_logger().error(str(e))

            finally:

                self.get_logger().info(
                    "Client disconnected"
                )

                client.close()
                self.client_socket = None

    def cmd_vel_callback(self, msg):

        if self.client_socket is None:
            return

        try:
            # Extract linear and angular velocities for 2D scenario
            linear_vel = msg.linear.x * 400
            angular_vel = msg.angular.z * 400

            # Format as JSON
            payload_dict = {
                "linear": linear_vel,
                "angular": angular_vel
            }
            payload = json.dumps(payload_dict) + "\n"

            self.client_socket.sendall(
                payload.encode()
            )

            self.get_logger().info(
                f"TX: linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )

        except Exception as e:

            self.get_logger().error(str(e))

    def publish_vehicle_odometry(self, line):

        try:

            payload = json.loads(line)

        except json.JSONDecodeError:

            return

        if payload.get("type") != "vehicle_data":

            return

        try:
            x_pos = float(payload["x"]) / 1000.0
            y_pos = float(payload["y"]) / 1000.0
            stamp_sec = float(payload["t"]) / 1000.0
            yaw = float(payload["h"])
            center_velocity = float(payload["c"]) / 1000.0
            left_velocity = float(payload["l"]) / 1000.0
            right_velocity = float(payload["r"]) / 1000.0
            imu_angular_velocity_z = float(payload.get("tz", 0.0)) / 1000.0
            imu_linear_acceleration_x = float(payload.get("ax", 0.0)) / 1000.0

        except (KeyError, TypeError, ValueError) as exc:

            self.get_logger().warn(
                f"Ignoring malformed vehicle_data payload: {exc}"
            )
            return

        odom = Odometry()
        if self.get_parameter("use_source_timestamp").value:
            odom.header.stamp = rclpy.time.Time(seconds=stamp_sec).to_msg()
        else:
            odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Rotate source coordinates into the ROS odom frame.
        odom.pose.pose.position.x = (
            x_pos * self._heading_offset_cos - y_pos * self._heading_offset_sin
        )
        odom.pose.pose.position.y = (
            x_pos * self._heading_offset_sin + y_pos * self._heading_offset_cos
        )
        odom.pose.pose.position.z = 0.0

        # Convert heading from milliradians to radians and align it with ROS.
        yaw = (yaw / 1000.0) + self.heading_offset_rad

        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 1e-3  # variance on x (m^2)
        odom.pose.covariance[7] = 1e-3  # variance on y (m^2)
        odom.pose.covariance[35] = 1e-2  # variance on yaw (rad^2)

        odom.twist.twist.linear.x = center_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = (
            (right_velocity - left_velocity) / float(self.wheel_separation)
        )

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 1e-3
        odom.twist.covariance[7] = -1.0
        odom.twist.covariance[35] = 1e-2

        self.odom_pub.publish(odom)

        self.publish_wheel_joint_states(
            stamp_sec,
            left_velocity,
            right_velocity
        )

        try:
            imu = Imu()
            imu.header.stamp = odom.header.stamp
            imu.header.frame_id = "base_imu"
            imu.angular_velocity.x = 0.0
            imu.angular_velocity.y = 0.0
            imu.angular_velocity.z = imu_angular_velocity_z
            imu.linear_acceleration.x = imu_linear_acceleration_x
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 0.0
            imu.orientation_covariance = [ -1.0, 0.0, 0.0,
                                           0.0, -1.0, 0.0,
                                           0.0, 0.0, -1.0 ]
            imu.angular_velocity_covariance = [1e-3, 0.0, 0.0,
                                               0.0, 1e-3, 0.0,
                                               0.0, 0.0, 1e-3]
            imu.linear_acceleration_covariance = [1e-2, 0.0, 0.0,
                                                 0.0, 1e-2, 0.0,
                                                 0.0, 0.0, 1e-2]
            self.imu_pub.publish(imu)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish IMU: {e}")

    def publish_wheel_joint_states(
        self,
        stamp_sec,
        left_wheel_linear_velocity,
        right_wheel_linear_velocity
    ):

        if self.last_joint_state_stamp_sec is None:
            dt = 0.0
        else:
            dt = stamp_sec - self.last_joint_state_stamp_sec
            if dt < 0.0:
                dt = 0.0

        self.last_joint_state_stamp_sec = stamp_sec

        left_wheel_angular_velocity = left_wheel_linear_velocity / WHEEL_RADIUS
        right_wheel_angular_velocity = right_wheel_linear_velocity / WHEEL_RADIUS

        self.left_wheel_position += left_wheel_angular_velocity * dt
        self.right_wheel_position += right_wheel_angular_velocity * dt

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT]
        joint_state.position = [
            self.left_wheel_position,
            self.right_wheel_position,
        ]
        joint_state.velocity = [
            left_wheel_angular_velocity,
            right_wheel_angular_velocity,
        ]

        self.joint_state_pub.publish(joint_state)


def main(args=None):

    rclpy.init(args=args)

    node = TcpBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
