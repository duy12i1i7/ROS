#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import datetime


def save_image(cv_image):
    # Get the current script directory
    script_dir = os.path.dirname(os.path.realpath(__file__))
    images_folder_path = os.path.join(script_dir, "images")

    # Create the folder if it does not exist
    if not os.path.exists(images_folder_path):
        os.makedirs(images_folder_path)
    
    # Create a timestamped filename for the image
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    filename = f"{timestamp}.png"
    file_path = os.path.join(images_folder_path, filename)

    # Save the image
    cv2.imwrite(file_path, cv_image)
    rospy.loginfo(f"Image saved as {file_path}")
    
    
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
    save_image(cv_image)


def main():
    rospy.init_node('collect', anonymous=True)
    rospy.Subscriber("/team613/camera/rgb/compressed", CompressedImage, image_callback)
    rospy.loginfo("Image reader node started")
    rospy.spin()

if __name__ == '__main__':
    main()

