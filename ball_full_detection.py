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
#------------------------------------------------------


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge #CvBridge is a ROS utility that helps bridge the gap between ROS image messages and OpenCV images.
from ultralytics import YOLO   
import numpy

model = YOLO("best.pt")
rgbCV_img = None
depthCV_img = None
bridge = CvBridge()


# The following values could be changed later
fx = 606  # Focal length x-axis
fy = 589  # Focal length y-axis
cx = 319.5  # Principal points
cy = 239.5

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
    results = model.track(source=rgbCV_img, save=False, show=False,conf=0.8, iou=0.5)
    boxes_num = len(results[0].boxes)
    ball_detected = False
    if boxes_num != 0 :
        ball_detected = True
        # Add here the channel to publish on the boolean ball_detected True
        i=0
        min_depth = float('inf')
        for r in results:
            while i < boxes_num :
                coordinates = r.boxes.xywh.cpu().numpy()
                classes_id = r.boxes.id.cpu().numpy()

                id 
                Xcenter = coordinates[i][0]
                Ycenter = coordinates[i][1]
                z = depthCV_img[Ycenter,Xcenter] # in mm

                if z == 0 or not numpy.isfinite(z):  # Handle invalid depth
                    continue

                # Attention Teammates! The following values (Xworld, Yworld, Zworld<distance from robot>) are all local frames
                # To get global coordinates we need to use equations to convert these values using further mathematical equations
                Xcamera =(Xcenter - cx) * z / fx
                Ycamera = (Ycenter - cy) * z / fy
                # z is the same
                
                ## Here I added I logic to return the coordinates and id of the nearest ball
                if z<min_depth :
                    min_depth = z
                    near_ball_id = classes_id[i]
                    Xnear = Xcamera ## or You could Use Xcenter and Ycenter instead of camera coordinates
                    Ynear = Ycamera
                    Znear = z # in mm
                    # these info above could be used to return the nearest ball at the current frame (POV)
                rospy.loginfo(f"Ball id {classes_id}=detected at 3D coordinates: ({Xcamera}, {Ycamera}, {z} mm)") #for debugging
                i+=1
        return
    else:
        ball_detected = False
        rospy.loginfo("No ball detected")
        # Add here the channel to publish on the boolean ball_detected True
        return
    
rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, rgbCV_converter)
depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depthCV_converter)

