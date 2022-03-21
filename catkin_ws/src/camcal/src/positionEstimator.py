#!/usr/bin/env python

# -*- coding: utf-8 -*-
import os
import pickle
import numpy as np
import cv2
import rospy

# msgs
from skeleton.msg import Players
from geometry_msgs.msg import PoseStamped
from game.msg import GameStatus
game_status = GameStatus()

# tf
import tf2_ros
import tf2_geometry_msgs

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

cammat = None
print('Load calibration matrices from cam.pickle...')
dir_path = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(dir_path,'calibration.pickle'), 'rb') as handle:
    cammat = pickle.load(handle)

print('Load the intrinsic matrix...')
newcameramtx,_ = cv2.getOptimalNewCameraMatrix(cammat['mtx'], cammat['dist'], (1920,1080), 1, (1920,1080))

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

def player_cb(msg):
    global camera, tfbuf, namepub

    if not game_status.status=='DETECT_STARTED':
        return

    stamp_pos = PoseStamped()
    stamp_pos.header.stamp = rospy.Time.now()
    stamp_pos.header.frame_id = 'temi'

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

        # Transform the player poses to map frame for later usage
        stamp_pos.pose.position.x = pos[0]
        stamp_pos.pose.position.y = pos[1]
        posonmap = tf2_geometry_msgs.do_transform_pose(stamp_pos, transform)

        player.position = posonmap.pose
        
        if player.id >-1 and player.id<game_status.player_num:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

    playerpub.publish(msg)
    
def gamestatus_cb(msg):
    global game_status
    game_status = msg
    
def main():
    global image_pub, playerpub, tfbuf

    rospy.init_node('PositionEstimator', anonymous=False)
    rospy.Subscriber('/players/withid', Players, callback=player_cb, queue_size = 10)
    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    playerpub = rospy.Publisher('/players/withpos', Players, queue_size=1)

    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    rospy.spin()

if __name__ == '__main__':
    main()
