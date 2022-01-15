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

START_POSITION = "1.26,-3.57,0"

game_status = GameStatus()
game_status.status = "REGISTER"
temi_status = 'NO_STATUS'
move_status = 'STOP'

def status_cb(msg):
    global temi_status, move_status
    status = msg.data.split(',')
    if status[0] == 'goToStatus':
        temi_status = status[1].upper()

        if move_status == 'MOVING':
            if temi_status == 'MOVING':
                move_status = 'MOVING'
            elif temi_status == 'COMPLETE':
                move_status = 'STOP'
        elif move_status == 'STOP':
            if temi_status == 'MOVING':
                move_status = 'MOVING'
            if temi_status == 'COMPLETE':
                move_status = 'STOP'

def game_cb(msg):
    global game_status, start_time
    game_status = msg

    if game_status.status == "START_GAME":
        if game_status.last_status == "REGISTER": 
            start_time = rospy.Time.now()
    
def start_the_round():
    #go to start position
    pub_cmd.publish(TemiCMD("cmd","goToPosition,"+START_POSITION))
    print("goToPosition")
    move_status = 'MOVING'

    #check if done moving
    while not (move_status == 'STOP' and temi_status == 'COMPLETE'):
        continue

    for i in range(3,0,-1):
        cmd = TemiCMD("cmd", "countdown,"+str(i))
        pub_cmd.publish(cmd)
        rospy.sleep(1)

    #display pose
    game_status.last_status = game_status.status
    game_status.status = 'DISPLAY_POSE'
    pub_status.publish(game_status)
    timelast = rospy.Time.now()

    #delay
    DELAY = 4
    while ((rospy.Time.now() - timelast) < DELAY):
        continue

    #turn
    pub_cmd.publish(TemiCMD("cmd", "turn"))
    timelast = rospy.Time.now()

    #delay
    DELAY = randint(3-8)
    while ((rospy.Time.now() - timelast) < DELAY):
        continue

    #turn back
    pub_cmd.publish(TemiCMD("cmd", "turnback"))

    #ready to detect pose
    game_status.last_status = game_status.status
    game_status.status = 'READY_TO_DETECT'
    pub_status.publish(game_status)
    
    while not (game_status.status == 'DETECT_END'):
        continue

    return

def main():
    global pub_status
    global pub_cmd
    global start_time
    global game_status

    # rospy.Subscriber('game', String, callback=game_cb)
    # pub = rospy.Publisher('game',String)
    rospy.Subscriber('game_status', GameStatus, callback=game_cb)
    rospy.Subscriber('status', String, callback=status_cb)
    pub_status = rospy.Publisher('game_status',GameStatus)
    pub_cmd = rospy.Publisher('temi_cmd', TemiCMD)
    rospy.init_node('game_loop', anonymous=False)
    rospy.loginfo("Start game loop")
    
    while not rospy.is_shutdown():
        if game_status.status == "START_GAME":
        #if 5 min or if not dead
            if (len(game_status.alive) > 1) and ((rospy.Time.now() - start_time) < 300):
                start_the_round()
            else:
                game_status.last_status = game_status.status
                game_status.status = 'END_GAME'
                pub_status.publish(game_status)

                #TODO:show winner

                #TODO: 
                # if one more game = pub 'START_GAME'
                # else pub 'REGISTER'

if __name__ == '__main__':
    main()

