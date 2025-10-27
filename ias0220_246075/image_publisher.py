import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_packages_with_prefixes


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.publisher_ = self.create_publisher(Image, "/image_raw", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()

        pkg_in = {get_packages_with_prefixes()["ias0220_246075"]}

        pkg_path = get_packages_with_prefixes()["ias0220_246075"]
        self.get_logger().info(f"Package directory: {pkg_path}")

        pkg_ = pkg_in.pop().split("/")[:-2]
        self.image_dir = "/".join(pkg_) + "/src/ias0220_246075/data/images"

        self.get_logger().info(f"Computed images directory: {self.image_dir}")

        self.index = 0

        self.image_files = []
        if os.path.exists(self.image_dir):
            self.image_files = sorted(
            [
                    os.path.join(self.image_dir, f)
                    for f in os.listdir(self.image_dir)
                    if f.lower().endswith(".png")
            ]
        )
            self.get_logger().info(f"Found {len(self.image_files)} images to publish.")
        else:
            self.get_logger().error(f"Image directory does not exist: {self.image_dir}")

        self.index = 0

    def timer_callback(self):
        if not self.image_files:
            self.get_logger().error("No images found in directory!")
            return

        img_path = self.image_files[self.index]
        frame = cv2.imread(img_path)
        if frame is None:
            self.get_logger().error(f"Failed to load image: {img_path}")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {os.path.basename(img_path)}")

        self.index = (self.index + 1) % len(self.image_files)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
