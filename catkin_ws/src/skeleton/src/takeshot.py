#!/usr/bin/env python
import os
import pickle
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)
os.chdir(dir_path + '/../poses')
bridge = CvBridge()
# module for camera calibration

imgsize = (0,0)
id=0
# Main loop receiving images from temi to calibrate
def imagecallback(msg):
    global imgsize,id
    # Decode the jpeg image    
    image = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('QQ', image)
    key = cv2.waitKey(1)

    if key == 10:
        id+=1
        cv2.imwrite('{0}.jpg'.format(id), image)

def main():
    rospy.Subscriber('/camera/undistorted', Image, imagecallback, queue_size=1)
    rospy.init_node('takeshoot', anonymous=False)
    rospy.spin()

if __name__ == '__main__':
    main()
