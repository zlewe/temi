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
from game.msg import GameStatus
from temi_driver.msg import TemiCMD

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()
    

def image_cb(msg):
    img = bridge.imgmsg_to_cv2(msg)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]
    result, encimg = cv2.imencode('.jpg', img, encode_param)

    bytesarr =  str(bytearray(np.squeeze(encimg, axis=1)))
    cmd = TemiCMD('imagestream', bytesarr)
    cmd_pub.publish(cmd)

def main():
    global cmd_pub

    rospy.init_node('GameCore', anonymous=False)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=10)
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        '''path = '/home/rtu/temi/catkin_ws/src/skeleton/poses/raw/i5.jpg'
        img = cv2.imread(path)
        cv2.imshow('img', img)
        cv2.waitKey(0)'''
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
