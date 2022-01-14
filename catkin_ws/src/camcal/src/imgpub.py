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


publishing = False
def command_cb(msg):
    global publishing
    msg = msg.data
    if msg=='START_OPENPOSE':
        publishing=True
    elif msg=='STOP_OPENPOSE':
        publishing=False

pub_count=0
def imagecallback(msg):
    global publishing, pub_count

    pub_count -=1
    if publishing and pub_count<=0:
        image_pub.publish(msg)
        pub_count=0
        
    
def main():
    global image_pub

    rospy.init_node('ImagePort', anonymous=False)
    rospy.Subscriber('/game', String,  callback=command_cb, queue_size=10)
    rospy.Subscriber('/camera/raw', Image, imagecallback, queue_size=1)
    image_pub = rospy.Publisher("/camera/port",Image, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    main()
