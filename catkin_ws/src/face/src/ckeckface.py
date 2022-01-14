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
from geometry_msgs.msg import PoseStamped
from ros_openpose.msg import Frame
from skeleton.msg import Player, Players
bridge = CvBridge()

dir_path = os.path.dirname(os.path.realpath(__file__))

temi_status = 'COMPELETE'   # MOVING, COMPELETE 
detect_status = 'FINISH'    # READY, MOVING, DETECTING, FINISH 

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
    global target_image, recognizer, recoreded, alive
    target_image = bridge.imgmsg_to_cv2(msg)

    gray = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(int(gray.shape[1]*0.5), int(gray.shape[0]*0.5)),)

    id = -1
    confidence = 0.0
    for (x, y, w, h) in faces:
        new_id, conf = recognizer.predict(gray[y:y + h, x:x + w])
        if conf < 100 and conf>confidence:
            id = new_id
            confidence = conf

    if id<0 or id>=PlayerNum or alive[id]<0:
        id=-1

    if id>-1:
        recoreded[id] += confidence

# receive precessed player data
def player_cb(msg):
    global players
    players = msg

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# checkout stage
def GoToCheckout(player):
    global detect_status, target_player

    if detect_status=='READY':
        print('Unknown person founded! Go checkout! => (%f, %f)'%(player.position.position.x, player.position.position.y))
        target_pos = PoseStamped()
        target_pos.pose.position = player.position.position
        target_pos.header.frame_id = 'map'
        target_pos.header.stamp = rospy.Time.now()
        goal_pub.publish(target_pos)
        detect_status = "MOVING"
        target_player = player

# receiving temi status
def status_cb(msg):
    global temi_status, detect_status

    temi_status = msg.data

    if detect_status=='READY':
        if temi_status=='COMPELETE':
            detect_status = 'READY'
        elif temi_status=='MOVING':
            detect_status = 'MOVING'
    elif detect_status=='MOVING':
        if temi_status=='COMPELETE':
            detect_status = 'DETECTING'
        elif temi_status=='MOVING':
            detect_status = 'MOVING'
    elif detect_status=='DETECTING':
        if temi_status=='COMPELETE':
            detect_status = 'DETECTING'
        elif temi_status=='MOVING':
            detect_status = 'DETECTING'
    elif detect_status=='FINISH':
        if temi_status=='COMPELETE':
            detect_status = 'FINISH'
        elif temi_status=='MOVING':
            detect_status = 'STOP'

def main():
    global image_pub, img, playerpub, target_image, goal_pub, players, detect_status

    target_image = None
    rospy.init_node('FaceCheckout', anonymous=False)

    initFaceRec()

    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=1)
    rospy.Subscriber('/players/final', Players, callback=player_cb, queue_size=10)
    rospy.Subscriber('/temi/status', String, callback=status_cb, queue_size=10)
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if detect_status=='COMPELETE':
            finish=True
            for player in players:
                if player.id<0:
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
