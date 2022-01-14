#!/usr/bin/env python3

import cv2
import numpy as np
from numpy.core.defchararray import asarray
from numpy.lib.type_check import imag
import rospy
import os
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from temi_driver.msg import TemiCMD
#from ros_openpose.msg import Frame
from skeleton.msg import Player, Players
bridge = CvBridge()

dir_path = os.path.dirname(os.path.realpath(__file__))

temi_status = 'COMPLETE'   # MOVING, COMPELETE 
detect_status = 'FINISH'    # READY, MOVING, DETECTING, FINISH 

names = ['brian','ariel', 'jj'] # add a name into this list
PlayerNum = 3
alive = [1,1,1]
players=Players()
target_player=None
recoreded=np.zeros(PlayerNum, dtype=float)

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
    global target_image, recognizer, recoreded, alive, detect_status, names
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

        if id<0 or id>=PlayerNum or alive[id]<0:
            print('No face detected!')
            return

        if id>-1:
            recoreded[id] += confidence
            print('Player %d is detected with %f confidence'%(id, confidence))

        if np.max(recoreded)>500.0:
            target_player.id = np.argmax(recoreded)
            print('I think this is player %d'%(target_player.id))

            if target_player.score<0.8:
                cmd = TemiCMD('cmd','out,%s'%(names[target_player.id]))
                cmd_pub.publish(cmd)

            detect_status = 'READY'

# receive processed player data
def player_cb(msg):
    global players, detect_status
    players = msg
    if (temi_status=='COMPLETE') and (detect_status=='FINISH'):
            detect_status = 'READY'

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Start one new checkout stage
def GoToCheckout(player):
    global goal_pub, detect_status, target_player, recoreded

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
        recoreded=np.zeros(PlayerNum, dtype=float)

# receiving temi status
def status_cb(msg):
    global temi_status, detect_status, cmd_pub

    status = msg.data.split(',')
    print(status)

    if status[0] == 'goToStatus':

        temi_status = status[1].upper()
        print('temi_status: '+temi_status+' detect_status: '+detect_status, end='')

        if detect_status=='READY':
            if temi_status=='COMPLETE':
                detect_status = 'READY'
            elif temi_status=='MOVING':
                detect_status = 'MOVING'
        elif detect_status=='MOVING':
            if temi_status=='COMPLETE':
                command = TemiCMD('cmd','tiltAngle,30')
                cmd_pub.publish(command)
                rospy.sleep(3)
                detect_status = 'DETECTING'
            elif temi_status=='MOVING':
                detect_status = 'MOVING'
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

def main():
    global target_image, goal_pub, players, detect_status, cmd_pub

    target_image = None
    rospy.init_node('FaceCheckout', anonymous=False)

    initFaceRec()

    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=1)
    rospy.Subscriber('/players/final', Players, callback=player_cb, queue_size=10)
    rospy.Subscriber('/status', String, callback=status_cb, queue_size=10)
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (temi_status=='COMPLETE') and (detect_status=='READY'):
            finish=True
            for player in players.players:
                if player.id<0 or player.score<0.8:
                    finish=False
                    GoToCheckout(player)
                    break
            if finish:
                pass
                # finish checkout progress, go back to front
                # goal_pub.publish()

        rate.sleep()

if __name__ == '__main__':
    main()
