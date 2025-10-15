#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from transforms3d.euler import euler2quat
import math
from std_msgs.msg import Int32MultiArray
from encoders_interfaces.msg import Counter

class Odometer(Node):
    def __init__(self):
        super().__init__("position_calculator")

        # Parameters
        self.declare_parameter("cpr", 508.8)  # counts per revolution
        self.declare_parameter("wheel_radius", 0.036)
        self.declare_parameter("wheel_separation", 0.35)
        self.declare_parameter("encoder_topic", "/encoders_ticks")
        self.declare_parameter("odom_topic", "/my_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        # Read params
        self.cpr = self.get_parameter("cpr").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.encoder_topic = self.get_parameter("encoder_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # Robot state
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.sub_encoders = self.create_subscription(
            Counter,
            # self._encoder_msg_type(),
            self.encoder_topic,
            self.encoder_callback,
            10
        )

        self.get_logger().info(
            f"Odometry node started.\n"
            f"  Subscribing to: {self.encoder_topic}\n"
            f"  Publishing to: {self.odom_topic}"
        )

    def _encoder_msg_type(self):
        """Dynamically load message type for /encoders_ticks."""
        from rosidl_runtime_py.utilities import get_message
        try:
            return get_message("encoders_pkg/msg/Encoders")
        except Exception:
            self.get_logger().warn("⚠️ Using fallback Int32MultiArray for encoders.")
            from std_msgs.msg import Int32MultiArray
            return Int32MultiArray

    def encoder_callback(self, msg):
        """Process new encoder ticks and publish odometry."""
        
        try:
            left_ticks = msg.count_left
            right_ticks = msg.count_right
        except AttributeError:
            left_ticks, right_ticks = msg.data[0], msg.data[1]

        now = self.get_clock().now()

        # Initialize on first message
        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = now
            return

        # Calculate differences
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Handle encoder wrap-around
        max_ticks = self.cpr
        if abs(delta_left) > max_ticks / 2:
            delta_left -= math.copysign(max_ticks, delta_left)
        if abs(delta_right) > max_ticks / 2:
            delta_right -= math.copysign(max_ticks, delta_right)

        # Convert to distance (m)
        dist_left = 2 * math.pi * self.wheel_radius * (delta_left / self.cpr)
        dist_right = 2 * math.pi * self.wheel_radius * (delta_right / self.cpr)
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_separation

        # Compute elapsed time
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt == 0:
            return

        # Integrate pose
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose (position + quaternion orientation)
        q = euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        # Velocities
        odom_msg.twist.twist.linear.x = delta_s / dt
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt

        # Publish and log occasionally
        self.odom_pub.publish(odom_msg)
        if int(now.nanoseconds / 1e8) % 10 == 0:  # occasional log
            self.get_logger().info(
                f"x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Odometer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Vector3, Pose
# from std_msgs.msg import String


# class Odometer(Node):
#     def __init__(self):
#         super().__init__("position_calculator")
#         self.sub_vel = self.create_subscription(Vector3, "velocity", self.vel_callback, 10)
#         self.sub_name_time = self.create_subscription(String, "name_and_time", self.name_time_callback, 10)

#         self.pose = Pose()
#         self.dt = 0.5
#         self.get_logger().info("Odometer node started, listening to walker...")

#     def vel_callback(self, msg: Vector3):
#         self.pose.position.x += msg.x * self.dt
#         self.pose.position.y += msg.y * self.dt
#         self.pose.position.z = 0.0

#         self.get_logger().info(
#             f"The new position of the walker is :\n"
#             f"  x = {self.pose.position.x}\n"
#             f"  y = {self.pose.position.y}\n"
#             f"  z = {self.pose.position.z}"
#         )

#     def name_time_callback(self, msg: String):
#         try:
#             student_id, timestamp = msg.data.split(",")
#             self.get_logger().info(
#                 f"Student {student_id} contacted me, and told me that current time is: {timestamp}"
#             )
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse message: {msg.data} ({e})")


# def main(args=None):
#     rclpy.init(args=args)
#     node = Odometer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
