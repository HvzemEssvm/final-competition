#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class BallDetector:
    def __init__(self, model_path):
        # Initialize ROS node
        rospy.init_node('ball_detector', anonymous=True)

        # Load the YOLO model using ultralytics
        self.model = YOLO(model_path)

        # Publisher to indicate ball detection
        self.ball_detected_pub = rospy.Publisher('/ball_detected', Bool, queue_size=10)

        # Subscribers for RGB and depth images
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        # Used to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Placeholder for RGB and depth images
        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Call the ball detection function
        self.detect_ball()

    def depth_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def detect_ball(self):
        if self.rgb_image is None:
            return

        # Flip the frame for consistency with the second script
        flipped_frame = cv2.flip(self.rgb_image, 1)

        # Run the YOLOv8 model for ball detection
        results = self.model.track(source=flipped_frame, conf=0.8, iou=0.5, show=False)

        # Assuming YOLO detects the ball as a specific class (e.g., "ball" or class 0)
        ball_detected = False
        for result in results:
            for box in result.boxes:
                if box.cls == 0:  # Adjust based on the class index of the ball in your model
                    ball_detected = True
                    break

        # Publish the detection result
        self.ball_detected_pub.publish(ball_detected)

        # Log for debugging
        rospy.loginfo(f"Ball detected: {ball_detected}")

        # Optional: Show the annotated frame with detections
        annotated_frame = results[0].plot()  # Draw boxes on the frame
        cv2.imshow("Ball Detection", annotated_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        model_path = "full code/best.pt"  # Update with the path to your YOLO model file
        ball_detector = BallDetector(model_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
