#!/usr/bin/env python
import os
import pickle
import numpy as np
import cv2
import rospy

# msg
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players
from geometry_msgs.msg import Pose,PoseArray,PoseStamped

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

PlayerNum=3

names = ['brain','ariel', 'jj'] # add a name into this list
    
def main():
    global id, goalpub, tfbuf, playerpub

    rospy.init_node('gamecore', anonymous=False)

    rospy.spin()

if __name__ == '__main__':
    main()
