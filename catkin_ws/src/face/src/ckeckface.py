#!/usr/bin/env python3

import cv2
import numpy as np
from numpy.core.defchararray import asarray
from numpy.lib.type_check import imag
import rospy
import os

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from temi_driver.msg import TemiCMD
from skeleton.msg import Players
from game.msg import GameStatus
game_status = GameStatus()


from scipy.optimize import linear_sum_assignment
import math

from cv_bridge import CvBridge
bridge = CvBridge()

dir_path = os.path.dirname(os.path.realpath(__file__))

temi_status = 'COMPLETE'   # MOVING, COMPELETE 
detect_status = 'FINISH'    # READY, MOVING, DETECTING, FINISH 

players=Players()
target_player=None
recoreded=np.zeros(game_status.player_num, dtype=float)
matrix=np.zeros((game_status.player_num, game_status.player_num)) #define empty matrix 
person = 0
row=[]


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x,y,z,w)

# initialize the face recognition module
def initFaceRec():
    global recognizer, faceCascade, font
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(os.path.join(dir_path,'trainer.yml'))

    face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
    faceCascade = cv2.CascadeClassifier(face_cascade_Path)

    font = cv2.FONT_HERSHEY_SIMPLEX

# callback function for image
def image_cb(msg):
    global target_image, recognizer, recoreded, alive, detect_status, names, matrix

    if (detect_status == "DETECTING"):
        target_image = bridge.imgmsg_to_cv2(msg)

        gray = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(int(gray.shape[1]*0.1), int(gray.shape[0]*0.1)),)

        id = -1
        confidence = 0.0
        for (x, y, w, h) in faces:
            new_id, conf = recognizer.predict(gray[y:y + h, x:x + w])
            if conf < 100 and conf>confidence:
                id = new_id
                confidence = conf

        if id<0 or id>=game_status.player_num or game_status.alive[id]<0:
            print('No face detected!')
            return

        if id>-1:
            recoreded[id] += confidence
            print('Player %d is detected with %f confidence'%(id, confidence))

        
        if np.max(recoreded)>500.0:
            target_player.id = np.argmax(recoreded)
            print('I think this is player %d'%(target_player.id))
            sum = 0           
            #allplayer.append(target_player.id)  
            for i in range(game_status.player_num):
                sum += recoreded[i]
            for i in range(game_status.player_num):
                recoreded[i] = (recoreded[i]/sum)
            matrix[person] = recoreded 

            if target_player.score<0.8:
                cmd = TemiCMD('cmd','out, YOU!')
                cmd_pub.publish(cmd)

            detect_status = 'READY'
        


# receive processed player data
def player_cb(msg):
    global players, detect_status, matrix, recoreded, row, person
    players = msg
    detect_status = 'READY'

    recoreded=np.zeros(game_status.player_num, dtype=float)
    matrix=np.zeros((game_status.player_num, game_status.player_num)) #define empty matrix 
    row=[]
    person = 0

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Start one new checkout stage
def GoToCheckout(player):
    global goal_pub, detect_status, target_player, recoreded, target_pos

    if detect_status=='READY':
        print('Unknown person founded! Go checkout! => (%f, %f)'%(player.position.position.x, player.position.position.y))

        # Set the target pos on the map
        target_pos = PoseStamped()
        pos = Vector3()
        dir = [1,0]
        pos.x = player.position.position.x - dir[0]*1
        pos.y = player.position.position.y - dir[1]*1
        target_pos.pose.position = pos
        target_pos.pose.orientation = Quaternion(0,0,0,1)
        target_pos.header.frame_id = 'map'
        target_pos.header.stamp = rospy.Time.now()

        # pub goal
        goal_pub.publish(target_pos)

        print('Go to target palyer at (%f, %f)'%(target_pos.pose.position.x,target_pos.pose.position.y))

        detect_status = "MOVING"
        target_player = player

        # Reset recoreded confidence
        recoreded=np.zeros(game_status.player_num, dtype=float)

# receiving temi status
def status_cb(msg):
    global temi_status, detect_status, cmd_pub

    status = msg.data.split(',')
    #print(status)

    if status[0] == 'goToStatus':

        temi_status = status[1].upper()
        #print('temi_status: '+temi_status+' detect_status: '+detect_status)

        if detect_status=='READY':
            if temi_status=='COMPLETE':
                detect_status = 'READY'
            elif temi_status=='MOVING':
                detect_status = 'MOVING'
        elif detect_status=='MOVING':
            if temi_status=='COMPLETE':
                # position and orientation
                error = pow((target_player.position.position.x-target_pos.pose.position.x), 2)+pow((target_player.position.position.y-target_pos.pose.position.y),2)
                print('I want ot go to there, but end up with error %f'%error)
                
                #if(error>0.25):

                theta = math.atan2((target_player.position.position.y-temi_position.pose.position.y), (target_player.position.position.x-temi_position.pose.position.x))
                # target_pos.pose.orientation = theta
                orientation = quaternion_from_euler(0, 0, theta)

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = temi_position.pose.position.x
                pose.pose.position.y = temi_position.pose.position.y
                pose.pose.orientation.x = orientation[0]
                pose.pose.orientation.y = orientation[1]
                pose.pose.orientation.z = orientation[2]
                pose.pose.orientation.w = orientation[3]

                goal_pub.publish(pose) 
                detect_status = 'TURNING'
                temi_status = 'MOVING'
                #print('Fail to go to target pose, try turn another direction.......')
            elif temi_status=='MOVING':
                detect_status = 'MOVING'

        elif detect_status=='TURNING':
            if temi_status=='COMPLETE':

                # temi look up
                command = TemiCMD('cmd','tiltAngle,30')
                cmd_pub.publish(command)
                rospy.sleep(3)

                detect_status = 'DETECTING'
            elif temi_status=='MOVING':
                detect_status = 'TURNING'
        elif detect_status=='DETECTING':
            if temi_status=='COMPLETE':
                detect_status = 'FINISH'
            elif temi_status=='MOVING':
                detect_status = 'DETECTING'
        elif detect_status=='FINISH':
            if temi_status=='COMPLETE':
                detect_status = 'FINISH'
            elif temi_status=='MOVING':
                detect_status = 'STOP'
        elif detect_status=='STOP':
            if temi_status=='COMPLETE':
                detect_status = 'READY'
            elif temi_status=='MOVING':
                detect_status = 'STOP'

        print('=>> temi_status: '+temi_status+' detect_status: '+detect_status)

def temi_posi(msg):
    global temi_position
    temi_position = msg

def gamestatus_cb(msg):
    global game_status, gamestatud_pub
    game_status = msg

def main():
    global target_image, goal_pub, players, detect_status, cmd_pub, temi_position, person, matrix, game_status, gamestatud_pub

    temi_position = PoseStamped()
    target_image = None
    rospy.init_node('FaceCheckout', anonymous=False)

    initFaceRec()

    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=1)
    rospy.Subscriber('/players/final', Players, callback=player_cb, queue_size=10)
    rospy.Subscriber('/status', String, callback=status_cb, queue_size=10)
    rospy.Subscriber('/pose', PoseStamped, callback=temi_posi, queue_size=10)
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=5)
    gamestatud_pub = rospy.Publisher('/game_status', GameStatus, queue_size=10)
    game_pub = rospy.Publisher('/game', String, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (temi_status=='COMPLETE') and (detect_status=='READY') and game_status.status=='CHECKOUT_STARTED':
            finish=True
            person=0
            for player in players.players:
                if player.id<0:
                    finish=False
                    GoToCheckout(player)
                    break
                else:
                    matrix[person, player.id] = 1 if game_status.alive[player.id]==1 else 0
                person += 1
            if finish:
                matrix = 1 - matrix
                row_ind, col_ind = linear_sum_assignment(matrix)

                print(matrix)
                print(col_ind)

                die_guy = ''

                # Re-assign ids
                for i in range(min(len(players.players), game_status.player_num)):
                    players.players[i].id = col_ind[i]
                    if players.players[i].score < 0.8:
                        new_alive = list(game_status.alive)
                        new_alive[players.players[i].id] = 0
                        game_status.alive = tuple(new_alive)
                        print('Player %s is dead!'%(game_status.names[players.players[i].id]))

                        die_guy += game_status.names[players.players[i].id]+'  ' if game_status.alive[players.players[i].id]==0 else ''

                game_status.last_status = game_status.status
                game_status.status = 'DETECT_END'
                gamestatud_pub.publish(game_status)

                game_pub.publish('STOP_FACEDETECTION')
                game_pub.publish('STOP_OPENPOSE')
                
                rospy.sleep(1)
                print('Send msg: %s'%(die_guy))
                if len(die_guy)==0:
                    cmd_pub.publish(TemiCMD('cmd','out,Nobody die!'))
                else:
                    cmd_pub.publish(TemiCMD('cmd','out,'+die_guy))

                # finish checkout progress, go back to front
                # goal_pub.publish()

        rate.sleep()

if __name__ == '__main__':
    main()
