#!/usr/bin/env python3

from numpy.random.mtrand import randint
import paho.mqtt.client as mqtt
import numpy as np
import rospy
from enum import Enum
from game.msg import GameStatus
from rospy.core import rospyinfo
from temi_driver.msg import TemiCMD
from geometry_msgs.msg import Pose
from tf.transformations import random_quaternion
from std_msgs.msg import String

stat = ["REGISTER","START_GAME","DISPLAY_POSE","READY_TO_DETECT","DETECT_STARTED","DETECT_END",""]

pub_status = rospy.Publisher('game_status',GameStatus, queue_size=10)
rospy.init_node('game_test', anonymous=False)

game_status = GameStatus()
game_status.last_status = stat[4]
game_status.status = stat[5]
game_status.player_num = 3
game_status.alive = [1, 0 ,0]
game_status.names = ['a','b','c']
pub_status.publish(game_status)

rospy.spin()