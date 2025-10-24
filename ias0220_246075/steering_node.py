#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler
import math

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.sub_distance = self.create_subscription(Range, '/distance', self.distance_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.distance = 0.0
        self.distance_threshold_max = 0.05
        self.distance_threshold_min = 0.001

        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info("Steering node started. Controlling robot via IMU + Distance sensor.")

    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])
        self.current_roll = roll
        self.current_pitch = pitch

    def distance_callback(self, msg: Range):
        self.distance = msg.range
        self.get_logger().info(f"New Distance: ${self.distance}")

    def publish_cmd(self):
        twist = Twist()
        
        if self.distance < self.distance_threshold_max and self.distance > self.distance_threshold_min:
            linear_speed = max(min(self.current_pitch * 0.5, 0.5), -0.5)
            angular_speed = max(min(self.current_roll * 1.0, 1.0), -1.0)

            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
