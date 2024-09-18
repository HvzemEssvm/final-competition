#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu  
from std_msgs.msg import Bool
import math

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ball_detected_sub = rospy.Subscriber('/ball_detected', Bool, self.ball_detected_callback) #subscribe to camera results
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.move_ball=rospy.Publisher('/move', Bool,queue_size=10)
        self.robot_x = 0
        self.robot_y = 0.22  # initial position of the robot
        self.ball_detected = False
        self.current_orientation = 0  # for the orientation from the IMU

    def odom_callback(self, msg):
        # Get robot's position from odom
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        rospy.loginfo("Current position: x={}, y={}".format(self.robot_x, self.robot_y))
        self.check_zone()

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles to get yaw (rotation around z-axis)
        orientation_q = msg.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def rotate_by_angle(self, target_angle_rad):
        # Generic rotation function
        twist = Twist()
        target_angle = (self.current_orientation + target_angle_rad) % (2 * math.pi)
        while abs(self.current_orientation - target_angle) > 0.1:  # Rotate until the target is reached
            twist.angular.z = 0.5  # Adjust rotation speed
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def rotate_180(self):
        # Rotate 180 degrees using the generic function
        rospy.loginfo("Rotating 180 degrees")
        self.rotate_by_angle(math.pi)

    def search_ball(self):
        # Rotate incrementally to search for a ball
        rospy.loginfo("Searching for the ball")
        twist = Twist()
        while not self.ball_detected:
            self.rotate_by_angle(math.radians(30))  # Rotate by 30 degrees
            rospy.sleep(0.5)  # Allow time for the camera to check
            if self.ball_detected:
                rospy.loginfo("Ball detected! Stopping search.")
                break  # Stop if the ball is detected
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def return_to_start(self):
        # Move back to the original starting position (0, 0.22)
        rospy.loginfo("Returning to start position")
        twist = Twist()
        while abs(self.robot_x - 0) > 0.1 or abs(self.robot_y - 0.22) > 0.1:
            twist.linear.x = -0.2  # Move backward
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def ball_detected_callback(self, msg):
        self.ball_detected = msg.data

    def check_zone(self):
        # Check if the robot is in restricted zones
        if self.robot_y > 1.1:  # The red line
            rospy.loginfo("Reached red line, rotating 180 degrees")
            self.rotate_180()
        elif abs(self.robot_x) >= 0.95:  # Boundaries
            rospy.loginfo("Reached boundary, rotating 180 degrees")
            self.rotate_180()
        elif 0.8 < self.robot_x < 1 and 0.2 < self.robot_y < 0.4:  # Positive penalty zone
            rospy.loginfo("Reached positive penalty zone, returning to start position")
            self.return_to_start()
        elif -1 < self.robot_x < -0.8 and 0.2 < self.robot_y < 0.4:  # Negative penalty zone
            rospy.loginfo("Reached negative penalty zone, returning to start position")
            self.return_to_start()
        elif -0.4 < self.robot_x < 0.4 and 0 < self.robot_y < 0.2:  # Green zone
            rospy.loginfo("Reached green zone, returning to start position")
            self.return_to_start()
        else:
            if not self.ball_detected:
                self.search_ball()  # Search for the ball if not detected
            else:
                rospy.loginfo("Ball detected, performing action!")  
                self.move_ball.publish(True)

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
