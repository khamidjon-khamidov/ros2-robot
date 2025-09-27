import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import random
import time

STUDENT_ID = "246075"


class RandomWalker(Node):
    def __init__(self):
        super().__init__("walker")
        self.publisher_vel = self.create_publisher(Vector3, "velocity", 10)
        self.publisher_name_time = self.create_publisher(String, "name_and_time", 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Walker node started, publishing at 2Hz...")

    def timer_callback(self):
        vx = random.choice([-1.0, 0.0, 1.0])
        vy = random.choice([-1.0, 0.0, 1.0])
        vz = 0.0

        vel_msg = Vector3(x=vx, y=vy, z=vz)
        self.publisher_vel.publish(vel_msg)
        self.get_logger().info(
            f"I will move with this velocity for 0.5 seconds:\n  x: {vx}\n  y: {vy}\n  z: {vz}"
        )

        current_time = time.time()
        msg = String()
        msg.data = f"{STUDENT_ID},{current_time}"
        self.publisher_name_time.publish(msg)
        self.get_logger().info(
            f"Hello, these are my ID and the current time: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
