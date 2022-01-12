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
    global id, poses, goalpub,goal,moving

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
        cv2.imshow('Target pose', image)
        cv2.waitKey(1)

        if msg.id>-1:
            print('%s\'s Pose score: %.0f'%(names[msg.id], new_score*100.0))

        '''if new_score>0.85:
            id+=1
            goal = PoseStamped()
            goal.header.frame_id='map'
            goal.header.stamp = rospy.Time.now()
            goal.pose = msg.position
            transform = tfbuf.lookup_transform('map', 'temi', rospy.Time.now(), rospy.Duration(4.0))
            goal = tf2_geometry_msgs.do_transform_pose(goal, transform)

            moving = True
            goalpub.publish(goal)'''
        

def pose_cb(msg):
    global goal,moving

    if goal is None:
        goal = msg
        return

    error=999.0
    if moving:
        error = (goal.pose.position.x - msg.pose.position.x)**2+\
        (goal.pose.position.y - msg.pose.position.y)**2+\
        (goal.pose.position.z - msg.pose.position.z)**2+\
        0.1*(goal.pose.orientation.x - msg.pose.orientation.x)**2+\
        0.1*(goal.pose.orientation.y - msg.pose.orientation.y)**2+\
        0.1*(goal.pose.orientation.z - msg.pose.orientation.z)**2+\
        0.1*(goal.pose.orientation.w - msg.pose.orientation.w)**2
        print('Target position error: %f'%error)

    if moving and error <2.0:
        moving=False
        print("Target reached!")

def goal_cb(msg):
    global goal
    moving = True
    goal = msg
    

def main():
    global id, goalpub,tfbuf

    rospy.init_node('findSkeleton', anonymous=False)


    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    rospy.Subscriber('/pose', PoseStamped, callback=pose_cb, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback=goal_cb, queue_size=10)
    goalpub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.Subscriber('/players/withpos', Players, callback=compare, queue_size = 10)

    image = bridge.imgmsg_to_cv2(poses[id][1])
    cv2.imshow('Target pose', image)
    cv2.waitKey(1)

    rospy.spin()

if __name__ == '__main__':
    main()
