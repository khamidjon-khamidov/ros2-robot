#!/usr/bin/env python3

"""
Solution to home assignment 7 (Robot Control). Node to take a set of
waypoints and to drive a differential drive robot through those waypoints
using a simple PD controller and provided odometry data.
"""

import math
import rclpy
import time
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class PDController(Node):
    def __init__(self):
        super().__init__('simple_control')

        # Wait for other nodes to start (gazebo, tf, etc.)
        time.sleep(5)

        # variables for error change rate calculation
        self.start_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        # subscriptions and publishers
        self.sub_odom = self.create_subscription(
            Odometry, '/diff_cont/odom', self.onOdom, 10)
        # optional goal subscriber (bonus) - safe to keep but not used
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.onGoal, 10)

        self.vel_cmd_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel', 10)
        self.pub_viz = self.create_publisher(Marker, "waypoints", 10)

        self.marker_frame = "odom"

        self.vel_cmd_msg = Twist()
        self.vel_cmd = np.array([0.0, 0.0])

        self.pos = np.array([0.0, 0.0])
        self.pos_diff = np.array([0.0, 0.0])
        self.theta = 0.0
        self.th_diff = 0.0

        self.error = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])

        self.error_change_rate = np.array([0.0, 0.0])

        # Load params from parameter server
        self.waypoints = []
        read_waypoints_x = self.declare_parameter('waypoints_x', [0.0]).value
        read_waypoints_y = self.declare_parameter('waypoints_y', [0.0]).value
        self.Kp = np.array(self.declare_parameter('Kp', [0.0, 0.0]).value)
        self.Kd = np.array(self.declare_parameter('Kd', [0.0, 0.0]).value)

        # Identify the waypoints parameter is correctly loaded or not
        if (read_waypoints_x == [0.0]) and (read_waypoints_y == [0.0]):
            self.get_logger().error("!!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!!")
            self.get_logger().error("Parameters not loaded correctly")
            self.get_logger().error("Please check your launch file to load yaml file correctly")
            self.get_logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        if (read_waypoints_x) and (read_waypoints_y):
            for i in range(len(read_waypoints_x)):
                self.waypoints.append([read_waypoints_x[i], read_waypoints_y[i]])
        self.waypoints = np.array(self.waypoints)

        self.distance_margin = self.declare_parameter(
            'distance_margin', 0.05).value
        self.target_angle = self.wrapAngle(self.declare_parameter(
            'target_angle', 0.0).value)

        # Safety caps for velocity (adjust if necessary)
        self.max_linear = self.declare_parameter('max_linear', 1.0).value
        self.max_angular = self.declare_parameter('max_angular', 2.0).value

        # Check Params
        self.get_logger().info(f'Waypoints: \n {self.waypoints}')
        self.get_logger().info(f'Kp: {self.Kp}')
        self.get_logger().info(f'Kd: {self.Kd}')
        self.get_logger().info(f'Start Time: {self.start_time}')

    def wrapAngle(self, angle):
        """
        Helper function that returns angle wrapped between +- Pi.
        """
        # Normalize to [-pi, pi]
        a = float(angle)
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def control(self):
        """
        Takes the errors and calculates velocities from it, according to
        PD control algorithm. Sets self.vel_cmd = [linear, angular]
        """
        # If there are no waypoints left, stop the robot
        if self.waypoints.size == 0:
            self.vel_cmd = np.array([0.0, 0.0])
            return

        # Proportional terms
        linear_p = self.Kp[0] * self.error[0]
        angular_p = self.Kp[1] * self.th_diff

        # Derivative terms (error change rates)
        linear_d = self.Kd[0] * self.error_change_rate[0]
        angular_d = self.Kd[1] * self.error_change_rate[1]

        v = linear_p + linear_d
        w = angular_p + angular_d

        # If angular error is large, optionally reduce forward speed to avoid cutting corners
        angle_abs = abs(self.th_diff)
        if angle_abs > (math.pi / 6.0):  # 30 degrees
            v *= max(0.0, 1.0 - (angle_abs - math.pi/6.0))

        # Saturate velocities
        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))

        self.vel_cmd = np.array([v, w])

    def publishWaypoints(self):
        marker = Marker()
        marker.header.frame_id = self.marker_frame

        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD

        marker.scale.x = self.distance_margin
        marker.scale.y = self.distance_margin
        marker.scale.z = self.distance_margin
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0

        marker.points = [Point(x=waypoint[0], y=waypoint[1])
                         for waypoint in self.waypoints]

        self.pub_viz.publish(marker)

    def calculateError(self):
        """
        Calculate lateral error to first waypoint and heading error between
        line to waypoint and robot heading. Updates self.error, self.error_change_rate, self.th_diff and self.pos_diff
        """
        if self.waypoints.size == 0:
            # no target
            self.error = np.array([0.0, 0.0])
            self.error_change_rate = np.array([0.0, 0.0])
            self.th_diff = 0.0
            self.pos_diff = np.array([0.0, 0.0])
            return

        # Current target is the first waypoint in the list
        target = self.waypoints[0]
        # position difference vector (target - current)
        self.pos_diff = np.array([target[0] - self.pos[0], target[1] - self.pos[1]])

        # distance (Euclidean)
        distance_error = float(np.linalg.norm(self.pos_diff))

        # desired heading to target
        desired_heading = math.atan2(self.pos_diff[1], self.pos_diff[0])

        # angular difference between desired heading and current heading
        self.th_diff = self.wrapAngle(desired_heading - self.theta)

        # update errors
        self.last_error = np.copy(self.error)
        self.error = np.array([distance_error, abs(self.th_diff)])

        # compute rate of change of error (derivative)
        # dt available from onOdom callback as self.dt (guard against zero)
        dt = getattr(self, 'dt', 1e-6)
        if dt <= 0.0:
            dt = 1e-6

        self.error_change_rate = (self.error - self.last_error) / float(dt)

    def isWaypointReached(self):
        if self.waypoints.size != 0:
            if (self.error[0] < self.distance_margin):
                self.waypoints = np.delete(self.waypoints, 0, axis=0)
                return True

        return False

    def onOdom(self, odom_msg):
        self.pos[0] = odom_msg.pose.pose.position.x
        self.pos[1] = odom_msg.pose.pose.position.y

        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.theta = euler_from_quaternion(orientation_list)[2]

        now_odom_time = Time.from_msg(odom_msg.header.stamp)

        dt_tmp = (now_odom_time - self.last_odom_time).to_msg()
        self.dt = float(dt_tmp.sec + dt_tmp.nanosec/1e9)
        # guard dt
        if self.dt <= 0.0:
            self.dt = 1e-6
        self.last_odom_time = now_odom_time

        # Calculate error between current pose and next waypoint position
        self.calculateError()

        # Check reaching waypoint or not
        if self.isWaypointReached():
            self.get_logger().info(
                "Reached waypoint!\nFuture waypoint list: "
                + str(self.waypoints))
            self.calculateError()  # Update error with new target waypoint

        # Calculate velocity command using PD control
        self.control()

        # publish velocity commands
        self.vel_cmd_msg.linear.x = float(self.vel_cmd[0])
        self.vel_cmd_msg.angular.z = float(self.vel_cmd[1])
        self.vel_cmd_pub.publish(self.vel_cmd_msg)

        # Publish waypoints visualization
        self.publishWaypoints()

    def onGoal(self, goal_msg):
        # Bonus: user-provided goal; simply append to waypoint list
        try:
            x = goal_msg.pose.position.x
            y = goal_msg.pose.position.y
            self.waypoints = np.vstack([self.waypoints, [x, y]])
            self.get_logger().info(f"Added external goal: {[x,y]}")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    controller = PDController()
    rclpy.spin(controller)

if __name__ == "__main__":
    main()