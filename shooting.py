#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool  
from geometry_msgs.msg import Twist  


class Shooting:
    def __init__(self):
        # Initialize the ROS node named 'ball_shooter'
        rospy.init_node('ball_shooter', anonymous=True)

        # Subscribe to the '/ball_detected' topic to get ball detection status (True or False)
        self.ball_detected_sub = rospy.Subscriber('/ball_detected', Bool, self.ball_callback)

        # Publisher to send velocity commands to control the Shato movement
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Store the ball detection status (initially False)
        self.ball_detected = False

    def ball_callback(self, data):
        """Callback function executed when a message is received on the '/ball_detected' topic"""

        # Update the ball detection status
        self.ball_detected = data.data
        rospy.loginfo(f"Ball detected: {self.ball_detected}")

        # If ball is detected, move Shato back and then forward
        if self.ball_detected:
            self.move_back_and_forward()

        # If no ball is detected, log the information (no movement)
        else:
            rospy.loginfo("No ball detected, no action taken.")

    def move_back_and_forward(self):
        """Move Shato back, then forward, and stop"""

        rospy.loginfo("Ball detected, Shoot")

        # Create a Twist message for backward movement
        move_back = Twist()
        move_back.linear.x = -0.2  # Set backward speed
        self.vel_pub.publish(move_back)
        rospy.sleep(2)  # Move backward for 2 seconds

        # Create a Twist message for forward movement
        move_forward = Twist()
        move_forward.linear.x = 0.6  # Set forward speed
        self.vel_pub.publish(move_forward)
        rospy.sleep(2)  # Move forward for 2 seconds

        # Create a Twist message to stop Shato
        stop = Twist()
        self.vel_pub.publish(stop)  # Stop Shato

if __name__ == '__main__':
    try:
        # Create an instance of the Shooting class to initialize everything
        Shooting()

        # Keep the program alive, waiting for incoming messages on the subscribed topics
        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
