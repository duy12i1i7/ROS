#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLOv10
import supervision as sv


def image_callback(ros_image, model):
    rospy.loginfo("Compressed image received")
    bridge = CvBridge()
    try:
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except CvBridgeError as e:
        rospy.logerr("CvBridgeError: %s", e)
        return

    try:
        # Predict and get the results
        results = model(source=cv_image, conf=0.3, save=False)[0]
        detections = sv.Detections.from_ultralytics(results)
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        label_annotator = sv.LabelAnnotator()

        annotated_image = bounding_box_annotator.annotate(scene=cv_image, detections=detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)
        #sv.plot_image(annotated_image)
    except Exception as e:
        rospy.logerr("Detection or drawing error: %s", e)
        return

    cv2.imshow("Image Window", annotated_image)
    cv2.waitKey(1)


def main():
    rospy.init_node('detect', anonymous=True)
    model_path = "./src/detect/model/yolov10x.pt"
    try:
        model = YOLOv10(model_path)
    except Exception as e:
        rospy.logerr("Model loading error: %s", e)
        return

    rospy.loginfo("Image reader node started with YOLOv10 model loaded")
    rospy.Subscriber("/team613/camera/rgb/compressed", CompressedImage, lambda msg: image_callback(msg, model))
    rospy.spin()

if __name__ == '__main__':
    main()

