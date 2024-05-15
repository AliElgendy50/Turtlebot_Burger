#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ultralytics import YOLO

import requests
import imutils


class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detection_publisher')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publisher for annotated image
        self.image_pub = rospy.Publisher('/object_detection/image', Image, queue_size=10)

        # Publisher for  position
        self.object_position_pub = rospy.Publisher('/object_position', Point, queue_size=10)

        # Load the YOLOv8 model
        self.model = YOLO('/home/ali/catkin_ws/src/Turtlebot_Burger/turtlebot3_laptop_code/scripts/best.pt')
        
        self.url = "http://192.168.100.7:8080/shot.jpg"
        

        # Loop for processing frames
        self.rate = rospy.Rate(30)  # Adjust as needed

    def detect_object(self):
        
        # Fetch image from the URL
        img_resp = requests.get(self.url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        img = imutils.resize(img, width=1000, height=1800) 

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = self.model.track(img, persist=True, conf=0.6)
        object_position = Point()
        object_position.x = 0
        object_position.y = 0

        if len(results[0].boxes) > 0:
            # Extract position and publish object positions for all detections
            for cls, _, xyxy in zip(results[0].boxes.cls, results[0].boxes.conf, results[0].boxes.xyxy):
                # Extract bounding box coordinates
                object_bbox = [int(coord) for coord in xyxy.tolist()]

                # Publish object position
                
                object_position.x = (object_bbox[0] + object_bbox[2]) / 2
                object_position.y = (object_bbox[1] + object_bbox[3]) / 2
        self.object_position_pub.publish(object_position)
            

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert the annotated frame from BGR to RGB
        annotated_frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

        # Rotate the frame by 90 degrees counterclockwise
        #annotated_frame_rgb = cv2.rotate(annotated_frame_rgb, cv2.ROTATE_90_CLOCKWISE)

        # Convert the frame to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame_rgb, encoding="rgb8")

        # Publish the annotated frame
        self.image_pub.publish(image_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.detect_object()
            self.rate.sleep()

        # Release the video capture
        self.video_capture.release()

if __name__ == '__main__':
    try:
        ObjectDetector().run()
    except rospy.ROSInterruptException:
        pass
