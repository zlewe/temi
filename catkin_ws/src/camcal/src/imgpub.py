#!/usr/bin/env python

# -*- coding: utf-8 -*-
import os
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dir_path = os.path.dirname(os.path.realpath(__file__))

bridge = CvBridge()

op_publishing = False
fd_publishing = False

def command_cb(msg):
    global op_publishing, fd_publishing

    msg = msg.data
    if msg=='START_OPENPOSE':
        print('Start openpose')
        op_publishing=True
    elif msg=='STOP_OPENPOSE':
        print('Stop openpose')
        op_publishing=False
    elif msg=='START_FACEDETECTION':
        print('Start face detection')
        fd_publishing=True
    elif msg=='STOP_FACEDETECTION':
        print('Stop face detection')
        fd_publishing=False
    
    cmd = msg.split(',')
    print(cmd)
    if len(cmd)>1:
        if cmd[0]=='Register':
            name_pub.publish(cmd[1])

def imagecallback(msg):
    global op_publishing, fd_publishing

    if op_publishing:
        opimg_pub.publish(msg)

    if fd_publishing:
        fdimg_pub.publish(msg)
    
def main():
    global opimg_pub, fdimg_pub, name_pub

    rospy.init_node('ImagePort', anonymous=False)
    rospy.Subscriber('/game', String,  callback=command_cb, queue_size=10)
    rospy.Subscriber('/camera/raw', Image, imagecallback, queue_size=1)
    opimg_pub = rospy.Publisher("/camera/openpose",Image, queue_size = 1)
    fdimg_pub = rospy.Publisher("/camera/facedetection",Image, queue_size = 1)
    name_pub = rospy.Publisher('/register/name', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
