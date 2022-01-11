#!/usr/bin/env python

# -*- coding: utf-8 -*-
import sys
import pickle
import socket
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cap = cv2.VideoCapture(0)

# Main loop receiving images from temi to calibrate

def imagecallback():
    global cammat,cap

    ret, frame = cap.read()

    if not ret:
        return

    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    
def main():
    global image_pub

    rospy.init_node('webcam', anonymous=False)
    image_pub = rospy.Publisher("/camera/raw",Image, queue_size = 10)
    while not rospy.is_shutdown():
        imagecallback()

if __name__ == '__main__':
    main()
