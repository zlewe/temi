#!/usr/bin/env python
import os
import pickle
import numpy as np
import math
import cv2
import rospy
import Skesim as ss
import tf2_ros
import tf2_geometry_msgs

# msg
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players
from geometry_msgs.msg import Pose,PoseArray,PoseStamped

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

names = ['brain','ariel', 'jj'] # add a name into this list
print("Re-directing the folder to poses...")
dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../poses'
os.chdir(dir_path)

print("Loading the poses...")
poses = None
with open(os.path.join(dir_path,'poses.pickle'), 'rb') as handle:
    poses = pickle.load(handle)
id = 0

moving = False
goal = None

def compare(players):
    global id, poses,goal,moving

    if moving:
        return

    image = bridge.imgmsg_to_cv2(poses[id][1])

    for msg in players.players:
        mypose = ss.toNumpyArray(msg.posture.skeleton.bodyParts)
        target = ss.toNumpyArray(poses[id][2].bodyParts)

        mja = ss.jointAngles(mypose)
        tja = ss.jointAngles(target)
        score, mins, count = ss.similarity(mja, tja) 
        new_score = (score+mins)/2.0*count/16.0
        msg.score = new_score

    playerpub.publish(players)
    
def main():
    global id, goalpub, tfbuf, playerpub

    rospy.init_node('findSkeleton', anonymous=False)

    rospy.Subscriber('/players/withpos', Players, callback=compare, queue_size = 10)
    playerpub = rospy.Publisher('players/withscore', Players, queue_size=10)

    image = bridge.imgmsg_to_cv2(poses[id][1])
    cv2.imshow('Target pose', image)
    cv2.waitKey(1)

    rospy.spin()

if __name__ == '__main__':
    main()
