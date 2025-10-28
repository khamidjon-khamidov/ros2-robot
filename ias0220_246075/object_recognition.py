import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__("object_recognition")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            "/my_robot/camera1/image_raw",
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(Image, "/object_recognition/image", 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define HSV range for target object
        lower = (50, 100, 100)
        upper = (70, 255, 255)
        
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
