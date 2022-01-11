#!/usr/bin/env python

# -*- coding: utf-8 -*-
import os
import pickle
import socket
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dir_path = os.path.dirname(os.path.realpath(__file__))

bridge = CvBridge()

imgsize=(0,0)

# Read calibration matrices form pickked file
# REF:https://stackoverflow.com/questions/11218477/how-can-i-use-pickle-to-save-a-dict

cammat = None
print('Load calibration matrices from cam.pickle...')
with open(os.path.join(dir_path,'calibration.pickle'), 'rb') as handle:
    cammat = pickle.load(handle)

# Existence check

if cammat is None:
    print("Dame! You have to save to the calibration matrices to cam.pickle file first!")


# Main loop receiving images from temi to calibrate

def imagecallback(msg):
    global cammat
    image = bridge.imgmsg_to_cv2(msg)

    imgsize = image.shape[:-1]
    
    # Undistort this shit
    #map1, map2 = cv2.fisheye.initUndistortRectifyMap(cammat['mtx'], cammat['dist'], np.eye(3), cammat['mtx'], (imgsize[1],imgsize[0]), cv2.CV_16SC2)
    #dst = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cammat['mtx'], cammat['dist'], (imgsize[1],imgsize[0]), 1, (imgsize[1],imgsize[0]))
    dst = cv2.undistort(image, cammat['mtx'], cammat['dist'], None, newcameramtx)

    # Cropping
    
    #x, y, w, h = roi
    #x, y, w, h = 50,50,530,450
    #dst = dst[y:y+h, x:x+w]
    
    image_pub.publish(bridge.cv2_to_imgmsg(dst, "bgr8"))
    
def main():
    global image_pub

    rospy.init_node('undistort', anonymous=False)
    rospy.Subscriber('/camera/raw', Image, imagecallback, queue_size=1)
    image_pub = rospy.Publisher("/camera/undistorted",Image, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    main()
