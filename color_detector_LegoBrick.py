#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def detect_color(hsv, color):
    if color == 'red':
        lower = np.array([0, 200, 0]) #[0, 120, 70])
        upper = np.array([10, 255, 99]) #[10, 255, 255])
    elif color == 'blue':
        lower = np.array([90,0,0]) #[110,50,50])
        upper = np.array([130,255,100]) #[130,255,255])
    elif color == 'green':
        lower = np.array([60, 0, 0]) #[36, 0, 0])
        upper = np.array([80, 255, 90]) #[86, 255, 255])
    elif color == 'yellow':
        lower = np.array([17, 70, 0]) #[21, 39, 64])
        upper = np.array([23, 255, 255]) #[40, 255, 255])
    #elif color == 'white':
        #lower = np.array([0, 0, 0])
        #upper = np.array([0, 0, 255])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

bridge = CvBridge()

def image_callback(img_msg):
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

    colors = ['red', 'blue', 'green', 'yellow'] #, 'white'
    for color in colors:
        mask = detect_color(hsv, color)
        # Apply morphological operations to get rid of noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations = 1)
        mask = cv2.dilate(mask, kernel, iterations = 1)

        # Finding contours for the mask
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Make the contour size relative to the image size
            rel_area = area / (cv2_img.shape[0] * cv2_img.shape[1])
            if rel_area > 0.001: # Adjust this value based on your requirement
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv2_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv2_img, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

    cv2.imshow('frame', cv2_img)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('color_detector_node', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

