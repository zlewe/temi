#!/usr/bin/env python3

from math import pi
from numpy.random.mtrand import randint
import paho.mqtt.client as mqtt
import numpy as np
import rospy
from enum import Enum
from game.msg import GameStatus
from rospy.core import rospyinfo
from temi_driver.msg import TemiCMD
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import random_quaternion, euler_from_quaternion
from std_msgs.msg import String

START_POSITION = rospy.get_param("/start_position","2, -3.6 ,0")

game_status = GameStatus()
game_status.status = "REGISTER"
temi_status = 'NO_STATUS' 
move_status = 'STOP'

def status_cb(msg):
    global temi_status, move_status
    #print('status_cb')
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
    global move_status, temi_status, robotpose
    #go to start position
    print('start_the_round')
    cmd = TemiCMD("cmd", "origin_position")
    timelast = rospy.Time.now()
    pub_cmd.publish(cmd)
    pub_cmd.publish(TemiCMD("cmd","goToPosition,"+START_POSITION))
    print("goToPosition")
    move_status = 'MOVING'

    #check if done moving
    DELAY = 13.1
    while not (move_status == 'STOP' and temi_status == 'COMPLETE' and (rospy.Time.now() - timelast).to_sec() > DELAY):
        #print('sleeping')
        rate.sleep()

    if game_status.last_status == "DETECT_END":
        rospy.sleep(10)
    
    print('countdown')
    for i in range(3,0,-1):
        cmd = TemiCMD("cmd", "countdown,"+str(i))
        pub_cmd.publish(cmd)
        rospy.sleep(1.05)

    #display pose
    game_status.last_status = game_status.status
    game_status.status = 'DISPLAY_POSE'
    pub_status.publish(game_status)
    timelast = rospy.Time.now()

    #delay
    DELAY = 10
    while ((rospy.Time.now() - timelast).to_sec() < DELAY):
        rate.sleep()

    #turn back
    q = [robotpose.orientation.x,robotpose.orientation.y,robotpose.orientation.z,robotpose.orientation.w]
    (_, _, before_angle) = euler_from_quaternion(q)
    print('turn '+str((rospy.Time.now() - timelast).to_sec())+' sec')
    pub_cmd.publish(TemiCMD("cmd", "turnback"))
    timelast = rospy.Time.now()

    #delay
    DELAY = randint(10,15)
    while ((rospy.Time.now() - timelast).to_sec() < DELAY):
        rate.sleep()

    #turn 
    print('turnback after delay '+str((rospy.Time.now() - timelast).to_sec())+' sec')
    pub_cmd.publish(TemiCMD("cmd", "turn"))

    q = [robotpose.orientation.x,robotpose.orientation.y,robotpose.orientation.z,robotpose.orientation.w]
    (_, _, current_angle)= euler_from_quaternion(q)
    while (abs(current_angle-before_angle) > 5.0*pi/180.0):
        q = [robotpose.orientation.x,robotpose.orientation.y,robotpose.orientation.z,robotpose.orientation.w]
        (_, _, current_angle)= euler_from_quaternion(q)
        print(current_angle, before_angle)
        rate.sleep()
    #ready to detect pose
    game_status.last_status = game_status.status
    game_status.status = 'DETECT_STARTED'
    pub_status.publish(game_status)
    
    while not (game_status.status == 'DETECT_END'):
        rate.sleep()
 
    game_status.last_status = game_status.status
    game_status.status = 'START_GAME'
    pub_status.publish(game_status)

    return

def pose_cb(msg):
    global robotpose

    robotpose = msg.pose

def main():
    global pub_status
    global pub_cmd
    global start_time
    global game_status
    global rate
    global start_time
    global move_status, temi_status

    # rospy.Subscriber('game', String, callback=game_cb)
    # pub = rospy.Publisher('game',String)
    rospy.Subscriber('game_status', GameStatus, callback=game_cb)
    rospy.Subscriber('status', String, callback=status_cb)
    rospy.Subscriber('pose', PoseStamped, callback=pose_cb)
    pub_status = rospy.Publisher('game_status',GameStatus, queue_size=10)
    pub_cmd = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
    rospy.init_node('game_loop', anonymous=False)
    rospy.loginfo("start game loop")
    
    
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
            if sum(game_status.alive) == 0:
                pub_cmd.publish(TemiCMD("cmd","goToPosition,"+START_POSITION))
                print("goToPosition")
                move_status = 'MOVING'
                #check if done moving
                while not (move_status == 'STOP' and temi_status == 'COMPLETE'):
                    rate.sleep()
                pub_cmd.publish(TemiCMD("cmd", "end,Nobody"))
                rospy.sleep(6)
                game_status.last_status = game_status.status
                game_status.status = 'SHOW_RESULT'
                pub_status.publish(game_status)
            else:
                winner = ''
                winner_index = 1
                for i, name in enumerate(game_status.names):
                    if game_status.alive[i] == 1:
                        winner += str(name)
                        if winner_index >= 1 and winner_index < sum(game_status.alive):
                            winner += ' and '
                        winner_index+=1
                pub_cmd.publish(TemiCMD("cmd","goToPosition,"+START_POSITION))
                print("goToPosition")
                move_status = 'MOVING'
                #check if done moving
                while not (move_status == 'STOP' and temi_status == 'COMPLETE'):
                    rate.sleep()
                print('show_winner/s')
                pub_cmd.publish(TemiCMD("cmd", "end,"+ winner))
                rospy.sleep(6)
                game_status.last_status = game_status.status
                game_status.status = 'SHOW_RESULT'
                pub_status.publish(game_status)
        
        elif game_status.status == "SHOW_RESULT":
            pub_cmd.publish(TemiCMD("cmd", "play_again"))
            start_time = None
            while not (game_status.status == "START_GAME"):
                rate.sleep()
            
            # reset_time()
            #TODO: 
            # if one more game = pub 'START_GAME'
            
            # else pub 'REGISTER'

        rate.sleep()

if __name__ == '__main__':
    main()

