#!/usr/bin/env python

# -*- coding: utf-8 -*-
import os
import pickle
import numpy as np
import cv2
import rospy
from ros_openpose.msg import Frame
from skeleton.msg import Player, Players
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

# tf
import tf2_ros
import tf2_geometry_msgs

bridge = CvBridge()

PlayerNum=3

# Main loop receiving images from temi to calibrate
cammat = None
print('Load calibration matrices from cam.pickle...')
dir_path = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(dir_path,'calibration.pickle'), 'rb') as handle:
    cammat = pickle.load(handle)

print('Load the intrinsic matrix...')
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
def imgcallback(msg):
    global camera
    camera = bridge.imgmsg_to_cv2(msg)

def mycallback(msg):
    global camera, tfbuf

    poses = PoseArray()
    stamp_pos = PoseStamped()
    
    poses.header.stamp = rospy.Time.now()
    poses.header.frame_id = 'temi'
    stamp_pos.header = poses.header

    # Add frame transformation to get the correct position in the map frame
    transform = tfbuf.lookup_transform('map', 'temi', rospy.Time.now(), rospy.Duration(4.0))

    for player in msg.players:
        skeleton = player.posture.skeleton
        feet = skeleton.bodyParts[11]
        if skeleton.bodyParts[11].pixel.y < skeleton.bodyParts[14].pixel.y:
            feet = skeleton.bodyParts[14].pixel
        else:
            feet = skeleton.bodyParts[11].pixel

        pos = toGround(feet.x, feet.y)

        new_pose = Pose()
        new_pose.position.x = pos[0]
        new_pose.position.y = pos[1]
        new_pose.position.z = 0.0
        poses.poses.append(new_pose)

        # Transform the player poses to map frame for later usage
        stamp_pos.pose = new_pose
        posonmap = tf2_geometry_msgs.do_transform_pose(stamp_pos, transform)

        player.position = posonmap.pose
        
        if player.id >-1 and player.id<PlayerNum:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

        
    pospub.publish(poses)
    playerpub.publish(msg)
    
def main():
    global image_pub, pospub, playerpub, transform, tfbuf

    rospy.init_node('PositionEstimator', anonymous=False)
    rospy.Subscriber('/players/withid', Players, callback=mycallback, queue_size = 10)
    pospub = rospy.Publisher('/players', PoseArray, queue_size=10)
    playerpub = rospy.Publisher('/players/withpos', Players, queue_size=1)

    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    rospy.spin()

if __name__ == '__main__':
    main()
