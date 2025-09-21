import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String


class Odometer(Node):
    def __init__(self):
        super().__init__("position_calculator")
        self.sub_vel = self.create_subscription(Vector3, "velocity", self.vel_callback, 10)
        self.sub_name_time = self.create_subscription(String, "name_and_time", self.name_time_callback, 10)

        self.pose = Pose()
        self.dt = 0.5
        self.get_logger().info("Odometer node started, listening to walker...")

    def vel_callback(self, msg: Vector3):
        self.pose.position.x += msg.x * self.dt
        self.pose.position.y += msg.y * self.dt
        self.pose.position.z = 0.0

        self.get_logger().info(
            f"The new position of the walker is :\n"
            f"  x = {self.pose.position.x}\n"
            f"  y = {self.pose.position.y}\n"
            f"  z = {self.pose.position.z}"
        )

    def name_time_callback(self, msg: String):
        try:
            student_id, timestamp = msg.data.split(",")
            self.get_logger().info(
                f"Student {student_id} contacted me, and told me that current time is: {timestamp}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse message: {msg.data} ({e})")


def main(args=None):
    rclpy.init(args=args)
    node = Odometer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
