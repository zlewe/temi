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
    print('status_cb')
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

        print(move_status, temi_status)

def game_cb(msg):
    global game_status
    game_status = msg
    print(game_status)
    
def reset_time():
    global start_time

    start_time = rospy.Time.now()

def start_the_round():
    global move_status, temi_status
    #go to start position
    print('start_the_round')
    pub_cmd.publish(TemiCMD("cmd","goToPosition,"+START_POSITION))
    print("goToPosition")
    move_status = 'MOVING'

    #check if done moving
    while not (move_status == 'STOP' and temi_status == 'COMPLETE'):
        #print('sleeping')
        rate.sleep()
    
    print('countdown')
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
    while ((rospy.Time.now() - timelast).to_sec() < DELAY):
        rate.sleep()

    #turn
    print('turn '+str((rospy.Time.now() - timelast).to_sec())+' sec')
    pub_cmd.publish(TemiCMD("cmd", "turn"))
    timelast = rospy.Time.now()

    #delay
    DELAY = randint(3,8)
    while ((rospy.Time.now() - timelast).to_sec() < DELAY):
        rate.sleep()

    #turn back
    print('turnback after delay '+str((rospy.Time.now() - timelast).to_sec())+' sec')
    pub_cmd.publish(TemiCMD("cmd", "turnback"))

    #ready to detect pose
    game_status.last_status = game_status.status
    game_status.status = 'READY_TO_DETECT'
    pub_status.publish(game_status)
    
    while not (game_status.status == 'DETECT_END'):
        rate.sleep()

    game_status.last_status = game_status.status
    game_status.status = 'START_GAME'
    pub_status.publish(game_status)

    return

def main():
    global pub_status
    global pub_cmd
    global start_time
    global game_status
    global rate
    global start_time

    # rospy.Subscriber('game', String, callback=game_cb)
    # pub = rospy.Publisher('game',String)
    rospy.Subscriber('game_status', GameStatus, callback=game_cb)
    rospy.Subscriber('status', String, callback=status_cb)
    pub_status = rospy.Publisher('game_status',GameStatus, queue_size=10)
    pub_cmd = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
    rospy.init_node('game_loop', anonymous=False)
    rospy.loginfo("Start game loop")
    
    
    rate = rospy.Rate(5)
    start_time = None
    while not rospy.is_shutdown():
        if game_status.status == "START_GAME":
        #if 5 min or if not dead
            if start_time is None:
                reset_time()

            if (sum(game_status.alive) > 1) and ((rospy.Time.now() - start_time).to_sec() < 300):
                print('elapsed_time = '+str((rospy.Time.now() - start_time).to_sec()))
                start_the_round()
            else:
                game_status.last_status = game_status.status
                game_status.status = 'END_GAME'
                pub_status.publish(game_status)
        
        elif game_status.status == "END_GAME":
                #TODO:show winner
                print('show_winner')
                #TODO: 
                # if one more game = pub 'START_GAME'
                game_status.last_status = game_status.status
                game_status.status = 'START_GAME'
                game_status.alive = np.ones((game_status.player_num,),dtype=np.int16).tolist()
                pub_status.publish(game_status)
                reset_time()
                # else pub 'REGISTER'

        rate.sleep()

if __name__ == '__main__':
    main()

