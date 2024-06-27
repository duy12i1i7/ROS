#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def image_callback(ros_image):
    rospy.loginfo("Compressed image received")
    bridge = CvBridge()
    try:
        # Convert ROS CompressedImage message to OpenCV2
        np_arr = np.fromstring(ros_image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except CvBridgeError as e:
        rospy.logerr("CvBridgeError: %s", e)
    
    # Display image
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('image_reader', anonymous=True)
    rospy.Subscriber("/team613/camera/rgb/compressed", CompressedImage, image_callback)
    rospy.loginfo("Image reader node started")
    rospy.spin()

if __name__ == '__main__':
    main()

