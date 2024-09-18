#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow
import tf

class PIDController:
    def __init__(self):
        # PID control parameters
        self.Kp_linear = 1.0
        self.Kp_strafe = 1.0  # New PID for strafing (x direction)
        self.Kp_angular = 4.0
        self.distance_tolerance = 0.05  # Threshold to stop when close enough to target

        # Target position (initialized to 0,0,0)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Initialize velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to odometry to get current position and orientation
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        # Extract the current position from odometry data
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        # Extract the current orientation (theta) from the quaternion
        orientation_q = data.pose.pose.orientation
        (_, _, self.current_theta) = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def set_target(self, x, y, theta):
        """Set the target position (x, y, theta) for the robot."""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta

    def calculate_distance_error(self):
        """Calculate the distance between current position and target position."""
        return sqrt(pow((self.target_x - self.current_x), 2) + pow((self.target_y - self.current_y), 2))

    def calculate_angle_to_target(self):
        """Calculate the angle between the robot's current heading and the target position."""
        return atan2(self.target_y - self.current_y, self.target_x - self.current_x)

    def calculate_angle_error(self):
        """Calculate the orientation error (difference between target and current theta)."""
        return self.target_theta - self.current_theta

    def calculate_strafe_velocity(self, angle_to_goal):
        """
        Calculate the strafe velocity (velocity in the x direction) for Mecanum wheels.
        The robot should strafe to align with the target direction.
        """
        return self.Kp_strafe * angle_to_goal

    def move_to_target(self):
        """Move the robot towards the target using PID control."""
        rate = rospy.Rate(10)  # 10 Hz update rate

        while not rospy.is_shutdown():
            # Calculate errors
            distance_error = self.calculate_distance_error()
            angle_to_goal = self.calculate_angle_to_target()
            angle_error = angle_to_goal - self.current_theta

            # Control signals
            linear_velocity = self.Kp_linear * distance_error if distance_error > self.distance_tolerance else 0
            angular_velocity = self.Kp_angular * angle_error
            strafe_velocity = self.calculate_strafe_velocity(angle_to_goal)

            # Stop the robot if it's within tolerance
            if distance_error < self.distance_tolerance:
                rospy.loginfo("Target reached!")
                self.stop_robot()
                break

            # Create and publish velocity command
            vel_msg = Twist()
            vel_msg.linear.x = linear_velocity  # Forward/backward
            vel_msg.linear.y = strafe_velocity  # Strafe left/right
            vel_msg.angular.z = angular_velocity  # Rotation

            self.velocity_publisher.publish(vel_msg)

            rate.sleep()

    def stop_robot(self):
        """Stop the robot by sending zero velocities."""
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('mecanum_robot_pid_controller', anonymous=True)

        # Create the controller object
        controller = PIDController()

        # Set the target for the controller
        controller.set_target(2.0, 2.0, 0.0) #(x, y, theta "in radian")

        # Start the robot movement
        controller.move_to_target()

    except rospy.ROSInterruptException:
        pass
