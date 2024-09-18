#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import torch  # Use this if your model is in PyTorch, change if you're using a different framework
import numpy as np

class BallDetector:
    def __init__(self, model_path):
        # Initialize ROS node
        rospy.init_node('ball_detector', anonymous=True)

        # Load your pre-trained model
        self.model = self.load_model(model_path)

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

    def load_model(self, model_path):
        # Load the model based on the framework you're using
        # Example: If using PyTorch, this will load the model
        model = torch.load(model_path)
        model.eval()  # Set the model to evaluation mode for inference
        return model

    def rgb_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Call the ball detection function
        self.detect_ball()

    def depth_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def preprocess_image(self, image):
        # Preprocess the image for your model (resize, normalize, etc.)
        # Example preprocessing (modify this according to your model's input requirements):
        image_resized = cv2.resize(image, (224, 224))  # Assuming 224x224 input for the model
        image_transposed = np.transpose(image_resized, (2, 0, 1))  # Change to channel-first format
        image_normalized = image_transposed / 255.0  # Normalize to [0, 1]
        image_tensor = torch.tensor(image_normalized, dtype=torch.float32).unsqueeze(0)  # Add batch dimension
        return image_tensor

    def detect_ball(self):
        if self.rgb_image is None:
            return

        # Preprocess the RGB image for your model
        input_tensor = self.preprocess_image(self.rgb_image)

        # Run the model for ball detection
        with torch.no_grad():  # Disable gradient computation for inference
            output = self.model(input_tensor)

        # Process the model's output to check if the ball is detected
        # Assuming the model returns a probability or binary prediction
        ball_detected = output.item() > 0.5  # Example threshold, modify based on your model

        # Publish the detection result
        self.ball_detected_pub.publish(ball_detected)

        # Log for debugging
        rospy.loginfo(f"Ball detected: {ball_detected}")

        # Optional: Show the image (for debugging)
        cv2.imshow("Ball Detection", self.rgb_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        model_path = "/path/to/your/model.pth"  # Update with the path to your model file
        ball_detector = BallDetector(model_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass