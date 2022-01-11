#!/usr/bin/env python

# -*- coding: utf-8 -*-
from ctypes import pointer
import os
import pickle
import numpy as np
import cv2
import rospy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

bridge = CvBridge()

# Main loop receiving images from temi to calibrate

cammat = None
print('Load calibration matrices from cam.pickle...')
dir_path = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(dir_path,'calibration.pickle'), 'rb') as handle:
    cammat = pickle.load(handle)

print('Load the intrinsic matrix...')
print(cammat['mtx'])

newcameramtx, _ = cv2.getOptimalNewCameraMatrix(cammat['mtx'], cammat['dist'], (1920,1080), 1, (1920,1080))

def toGround(x, y):
    pts = np.asarray((((x,y),),), dtype=np.float32)
    x,y = cv2.undistortPoints(pts,cammat['mtx'], cammat['dist'], P=newcameramtx)[0,0,:]

    ox = newcameramtx[0,2]
    oy = newcameramtx[1,2]
    fx = newcameramtx[0,0]
    fy = newcameramtx[1,1]

    y_ = (x-ox)/fx
    z_ = (y-oy)/fy
    x_ = 1.0

    vec = np.asarray((x_,y_,z_))
    vec /= np.linalg.norm(vec)

    height = 93.0
    vec *= height/vec[2]

    # reverse the horizontal axis
    vec[1]*=-1.0

    return vec[:2]*0.01

def mycallback(msg):
    global img

    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('pick_point', img)
    cv2.waitKey(1)
    
def mouse(event, x, y, flag, parms):

    # if mouse pressed
    if event==1:
        pos = toGround(x, y)
        fakepoint = PoseStamped()
        fakepoint.header.frame_id = 'temi'
        fakepoint.header.stamp = rospy.Time.now()
        fakepoint.pose.position.x = pos[0]
        fakepoint.pose.position.y = pos[1]
        fppub.publish(fakepoint)
        print(x,y)
        print(fakepoint.pose.position)
        transform = tfbuf.lookup_transform('map', 'temi', rospy.Time.now(), rospy.Duration(4.0))
        print(tf2_geometry_msgs.do_transform_pose(fakepoint, transform))
    
def main():
    global image_pub, fppub, tflistener, tfbuf

    cv2.namedWindow('pick_point', cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow('pick_point', 640, 480)

    rospy.init_node('fake_point_puber', anonymous=False)

    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    rospy.Subscriber("/camera/raw", Image, callback=mycallback, queue_size = 1)
    fppub = rospy.Publisher('/fakePoint', PoseStamped, queue_size=1)

    cv2.namedWindow('pick_point', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('pick_point', mouse)
    rospy.spin()

if __name__ == '__main__':
    main()
