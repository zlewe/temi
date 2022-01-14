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

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

game_status = GameStatus()
PlayerNum=3
    
def main():
    global id, goalpub, tfbuf, playerpub, game_status

    rospy.init_node('GameCore', anonymous=False)

    status_pub = rospy.Publisher('/game/status', GameStatus, queue_size=10)
    
    rate = rospy.Rate(10)

    game_status.player_num = 3
    game_status.status = 'READY'
    game_status.last_status = 'READY'
    game_status.names = ['Brian', 'Ariel', 'James']
    game_status.alive = [1, 1, 1]
    
    while not rospy.is_shutdown():

        status_pub.publish(game_status)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
