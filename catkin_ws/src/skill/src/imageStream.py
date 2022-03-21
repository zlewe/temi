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
from std_msgs.msg import String
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from game.msg import GameStatus
from temi_driver.msg import TemiCMD

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

img=None   
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]
last_taken=None

def image_cb(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg)

def game_cb(msg):
    global encode_param, last_taken
    if msg.data == 'take_one_picture' and (not img is None):
        print('Take a pic')
        result, encimg = cv2.imencode('.jpg', img, encode_param)
        bytesarr =  str(bytearray(np.squeeze(encimg, axis=1)))
        cmd = TemiCMD('imagestream', bytesarr)
        cmd_pub.publish(cmd)

        last_taken = rospy.Time.now()

    elif msg.data == 'ready_to_take_one_picture' and (not img is None):

        cmd = TemiCMD('cmd', 'already_to_take_one_picture')
        cmd_pub.publish(cmd)

def main():
    global cmd_pub, last_taken

    rospy.init_node('imageStream', anonymous=False)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
    rospy.Subscriber('/game', String, game_cb)
    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=10)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        if last_taken is not None and (rospy.Time.now()-last_taken).to_sec()>10:
            last_taken=None
            cmd = TemiCMD('cmd', 'return_initial')
            cmd_pub.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
    main()
