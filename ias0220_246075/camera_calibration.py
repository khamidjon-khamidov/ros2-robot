import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')

        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub_processed = self.create_publisher(Image, '/image_processed', 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.bridge = CvBridge()
        self.objpoints = []  # 3D points in real world
        self.imgpoints = []  # 2D points in image plane
        self.state = 'collecting'
        self.image_count = 0

        self.pattern_size = (7, 6)  # columns x rows
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.objp = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)

        self.mtx = None
        self.dist = None

        self.get_logger().info("Camera calibration node started. Waiting for /image_raw...")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.state == 'collecting':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

            if ret:
                self.get_logger().warn(f"Corners detected in this image (image {self.image_count + 1})")
            else:
                self.get_logger().warn(f"Corners NOT detected in this frame (image {self.image_count + 1})")

            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                self.image_count += 1

                cv2.drawChessboardCorners(frame, self.pattern_size, corners2, ret)
                self.get_logger().warn(f"Collected {self.image_count}/37 calibration images")

                processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                processed_msg.header = msg.header
                self.pub_processed.publish(processed_msg)

            if self.image_count >= 37:
                self.state = 'calibrating'
                self.get_logger().warn("Enough images collected. Starting calibration...")

        elif self.state == 'calibrating':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, gray.shape[::-1], None, None
            )
            self.mtx = mtx
            self.dist = dist

            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error

            mean_error /= len(self.objpoints)
            self.get_logger().warn(f"✅ Calibration complete. Mean error: {mean_error:.6f}")

            self.state = 'publishing'

        elif self.state == 'publishing':
            if self.mtx is None or self.dist is None:
                self.get_logger().warn("Calibration not yet done. Skipping /camera_info publish.")
                return

            cam_info = CameraInfo()
            cam_info.header = msg.header
            cam_info.height = frame.shape[0]
            cam_info.width = frame.shape[1]

            cam_info.distortion_model = "plumb_bob"

            cam_info.k = self.mtx.flatten().tolist()
            cam_info.d = self.dist.flatten().tolist()

            cam_info.r = np.eye(3).flatten().tolist()

            P = np.hstack((self.mtx, np.zeros((3, 1))))
            cam_info.p = P.flatten().tolist()

            self.pub_camera_info.publish(cam_info)
            self.get_logger().warn_once("Publishing /camera_info ...")

        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        processed_msg.header = msg.header
        self.pub_processed.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
