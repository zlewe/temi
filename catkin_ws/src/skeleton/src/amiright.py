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
from temi_driver.msg import TemiCMD

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

PlayerNum=3

names = ['brian','ariel', 'jj'] # add a name into this list
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

def pickPoseture():
    global id, cmd_pub
    id = np.random.randint(0,9, size=1, dtype=int)[0]

    cmd = TemiCMD()
    cmd.type = 'cmd'
    cmd.arg = 'imitate,%s'%(poses[id][0])
    print(poses[id][0], id)

    cmd_pub.publish(cmd)

    print('Publish')
    print(cmd)

    target_pos = ss.jointAngles(ss.toNumpyArray(poses[id][2].bodyParts))
    return target_pos


def compare(players):
    global id, poses,goal,moving,target_pos

    if moving:
        return

    for player in players.players:
        mypose = ss.toNumpyArray(player.posture.skeleton.bodyParts)

        mja = ss.jointAngles(mypose)
        score, mins, count = ss.similarity(mja, target_pos) 
        new_score = (score+mins)/2.0*count/16.0
        player.score = new_score

        if player.id >-1 and player.id<PlayerNum:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))
        
        if player.id == -1:
            print('Unknown playe with score %f at (%f, %f).'%(player.score, player.position.position.x, player.position.position.y))


    player_pub.publish(players)
    
def main():
    global id, goalpub, tfbuf, player_pub, cmd_pub, target_pos

    rospy.init_node('PostureCheck', anonymous=False)

    rospy.Subscriber('/players/withpos', Players, callback=compare, queue_size = 10)
    player_pub = rospy.Publisher('players/withscore', Players, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)

    rospy.sleep(1)
    target_pos = pickPoseture()
    rospy.spin()

if __name__ == '__main__':
    main()
