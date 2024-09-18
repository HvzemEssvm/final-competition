#!/usr/bin/env python

import rospy
from std_msgs.msgs import Bool
from geomtry_msgs.msgs import Twist


class Shooting:
    def__init__(self):
        rospy.init_node('ball_shooter', anonymous=True)
    
        self.ball_detected_sub = rospy.Subscriber('/ball_detected', Bool, self.ball_callback)
    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
        self.ball_detected = False
    
    def ball_callback(self, data):

        self.ball_detected = data.data
        rospy.loginfo(f"Ball detected: {self.ball_detected}")

        while self.ball_detected:
            self.move_back_and_forward()

    def move_back_and_forward(self):
        rospy.loginfo("Ball detected, moving bach and foward")

        move_back = Twist()
        move_back.linear.x = -0.2
        self.vel_pub.publish(move_back)
        rospy.sleep(2)

        move_forward = Twist()
        move_back.linear.x = 0.6
        self.vel_pub.publish(move_forward)
        rospy.sleep(2)

        stop = Twist()
        self.vel_pub.publish(stop)    

if __name__ == '__main__':
    try:
        ball_follower = Shooting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass            