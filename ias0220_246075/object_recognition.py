import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__("object_recognition")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, "/my_robot/camera1/image_raw", self.image_callback, 10
        )

        self.pub = self.create_publisher(Image, "/object_recognition/image", 10)

        self.lower_red1 = np.array([0, 150, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 150, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            large_contours = [c for c in contours if cv2.contourArea(c) > 200]

            if large_contours:
                c = max(large_contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 3)
                self.get_logger().info(f"Detected robot at x={x:.1f}, y={y:.1f}, r={radius:.1f}")
            else:
                self.get_logger().info("No large contours detected")
        else:
            self.get_logger().info("No contours detected")


        out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
