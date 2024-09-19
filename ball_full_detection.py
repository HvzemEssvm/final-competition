#!/usr/bin/env python3
#------------------------------------------------------
#   Edited By Hazem Essam
#   Please Please Read All the notes to fully understand and integrate the code
#------------------------------------------------------
#   I didn't want to publish on any specific project channel to make free for you
#   either to make this as individual script or integrate it to a main code,
#   but do not worry.. reeding notes will help you alot
#   contact me if you are confused with anything written down here...
#------------------------------------------------------
#   In your ROS package.xml, include the following dependencies:
#   <depend>rospy</depend>
#   <depend>sensor_msgs</depend>
#   <depend>std_msgs</depend>
#   <depend>cv_bridge</depend>
#   <depend>ultralytics</depend>
#   <depend>numpy</depend>
#   <depend>opencv-python</depend>
#------------------------------------------------------


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from cv_bridge import CvBridge #CvBridge is a ROS utility that helps bridge the gap between ROS image messages and OpenCV images.
from ultralytics import YOLO   
import numpy
import cv2

model = YOLO("best.pt")
rgbCV_img = None
depthCV_img = None
bridge = CvBridge()


# The following values could be changed later
fx = 606  # Focal length x-axis
fy = 589  # Focal length y-axis
cx = 319.5  # Principal points
cy = 239.5


ball_depth_pub = rospy.Publisher('/ball_depth', Float32, queue_size=10)  # Publishes the depth of the nearest ball
ball_detected_pub = rospy.Publisher('/ball_detected', Bool, queue_size=10)  # Publishes if a ball is detected



def rgbCV_converter(rgb_img):
    global rgbCV_img
    rgbCV_img = bridge.imgmsg_to_cv2(rgb_img, "bgr8")

def depthCV_converter(depth_img):
    global depthCV_img
    depthCV_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding="passthrough")

# The following function <ball_tracker()> should be called in the main loop!
def ball_tracker():
    global rgbCV_img, depthCV_img
    if rgbCV_img is None or depthCV_img is None:
        return
    # rgbCv_img = cv2.flip(self.rgb_image, 1) # remove the 1st (#) if the image is passed to the robot as a flipped image and flippedd coordinates
    results = model.track(source=rgbCV_img, save=False, show=False,conf=0.8, iou=0.5)
    boxes_num = len(results[0].boxes)
    ball_detected = False
    if boxes_num != 0 :
        ball_detected = True
        # (done)Add here the channel to publish on the boolean ball_detected True
        ball_detected_pub.publish(ball_detected)  # Publish ball_detected as True
        i=0
        min_depth = float('inf')
        for r in results:
            while i < boxes_num :
                coordinates = r.boxes.xywh.cpu().numpy()
                classes_id = r.boxes.id.cpu().numpy()
                Xcenter = int(coordinates[i][0])
                Ycenter = int(coordinates[i][1])
                z = depthCV_img[Ycenter,Xcenter] # in mm

                if z == 0 or not numpy.isfinite(z):  # Handle invalid depth
                    continue
                
                rospy.loginfo(f"Ball Detected, id : {classes_id[i]}, detected at 3D coordinates: ({Xcamera}, {Ycamera}, {z} mm)")

                # Attention Teammates! The following values (X, Y, Zworld<distance from robot>) are all local frames
                # To get global coordinates we need to use equations to convert these values using further mathematical equations
                Xcamera =(Xcenter - cx) * z / fx
                Ycamera = (Ycenter - cy) * z / fy
                # z is the same
                
                ## Here I added a logic to return the coordinates and id of the nearest ball
                if z<min_depth :
                    min_depth = z
                    near_ball_id = classes_id[i]
                    Xnear = Xcenter ## or You could Use Xcenter and Ycenter instead of camera coordinates
                    Ynear = Ycenter
                    Znear = z # in mm
                    # these info above could be used to return the nearest ball at the current frame (POV)
                    ball_depth_pub.publish(Znear) # publish the Znear
                if i == boxes_num-1: 
                    rospy.loginfo(f"Nearest Ball id {near_ball_id}=detected at 3D coordinates: ({Xnear}, {Ynear}, {Znear} mm)")
                i+=1
        return
    else:
        ball_detected = False
        rospy.loginfo("No ball detected")
        # Add here the channel to publish on the boolean ball_detected True
        ball_detected_pub.publish(ball_detected)
        return
    
rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, rgbCV_converter)
depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depthCV_converter)


rospy.init_node('ball_tracker_node', anonymous=True)
while True:
    ball_tracker()