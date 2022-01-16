#!/usr/bin/env python3

import os
import cv2
import json
import rospy
import numpy as np
import pickle

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from temi_driver.msg import TemiCMD
from game.msg import GameStatus

from cv_bridge import CvBridge
bridge = CvBridge()

# Game status
game_status = GameStatus()

src_path = os.path.dirname(os.path.realpath(__file__))
imgs_path = os.path.join(src_path, 'images')
players_path = os.path.join(src_path, 'players.json')

# player list 
players_data = {'names':[]}
new_resgister = {}
count = 0

# read the players.json file to know player numbers and names
def getPlayerList():
    global players_data

    if os.path.exists(players_path):
        print('Found players data, ready to load...')
        with open(players_path) as f:
            players_data = json.load(f)
    else:
        players_data = {'names':[]}

# initialize the face recognition moudle
def initFaceRec():
    global faceCascade, font

    print('Initialize the face recognition module...')
    face_cascade_Path = os.path.join(src_path,'haarcascade_frontalface_default.xml')
    faceCascade = cv2.CascadeClassifier(face_cascade_Path)

    font = cv2.FONT_HERSHEY_SIMPLEX

# check if the joint is detected
def isJointValid(j):
    return j.pixel.x>0.0 or j.pixel.y>0.0

# crop a picture around the head
def getHeadPic(img, s):
    
    bbx = np.asarray((0,0,img.shape[1], img.shape[0]), dtype=float)
    
    for i in [0, 15, 16, 17, 18]:
        if isJointValid(s.bodyParts[i]):
            bbx[0] = max(bbx[0], s.bodyParts[i].pixel.x)
            bbx[1] = max(bbx[1], s.bodyParts[i].pixel.y)
            bbx[2] = min(bbx[2], s.bodyParts[i].pixel.x)
            bbx[3] = min(bbx[3], s.bodyParts[i].pixel.y)

    size = (2.2*(bbx[1]-bbx[3])/2, 1.8*(bbx[0]-bbx[2])/2)

    # inflate the range
    bbx[1] = min(img.shape[0], bbx[1]+size[1])
    bbx[3] = max(0, bbx[3]-size[1])

    bbx[0] = min(img.shape[1], bbx[0]+size[0])
    bbx[2] = max(0, bbx[2]-size[0])

    bbx = np.int0(bbx)

    ret = img[bbx[3]:bbx[1], bbx[2]:bbx[0], :]
    if len(ret)==0:
        return None
    return ret

# image callback to retrieve the currebnt image
def image_cb(msg):
    global target_image
    target_image = bridge.imgmsg_to_cv2(msg)

# get skeleton
def frame_cb(msg):
    global target_image, count, player_name

    if target_image is None or \
        not game_status.status == 'REGISTER':
        return

    for skeleton in msg.persons:

        # get head pic
        img = getHeadPic(target_image, skeleton)
        if img is None:
            print('I found your head, but sorry.')
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(gray, 1.3, 5, minSize = (int(img.shape[1]*0.7), int(img.shape[0]*0.7)))

        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
            count += 1
            print(count)

            # Save the captured image into the images directory
            cv2.imwrite(os.path.join(src_path,'images/%s.'%(player_name)) + str(players_data['names'].index(player_name)) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
            cv2.imshow('image', img)
            cv2.waitKey(1)

            if count>100:
                stopDetect()
                return


def getAllRegisterPlayer():
    os.listdir()

# called when the numbers of images are enough
def stopDetect():
    global game_pub, temi_pub

    game_pub.publish('STOP_FACEDETECTION')
    game_pub.publish('STOP_OPENPOSE')
    
    player_list = print(str(players_data['names'])[1:-1].replace('\'','').replace(' ',''))
    temi_pub.publish(TemiCMD('cmd', 'register_done,'+player_list))

    with open(players_path, 'w') as f:
        json.dump(players_data, f)

    print('Enough data received, stop detection...')

# called whenever a player name is passed
def name_cb(msg):
    global game_pub, player_name, players_data, count

    # normalize the player name
    player_name = msg.data.replace(' ','_')
    playerfilepath = os.path.join(src_path, '%d.json'%(player_name))
    
    if os.path.exists(playerfilepath):
    #if player_name in players_data['names']:
        print('This player is already exist.')
        temi_pub.publish(TemiCMD('cmd', 'register_done'))
    else:
        print('Start register player, %s, reset image count.'%(player_name))
        count = 0
        players_data['names'].append(player_name)
        game_pub.publish('START_FACEDETECTION')
        game_pub.publish('START_OPENPOSE')

# update the game status 
def gamestatus_cb(msg):
    global game_status, temi_pub
    game_status = msg

    if game_status.status == 'REGISTER':
        temi_pub.publish(TemiCMD('cmd', 'tiltAngle,30'))

# main loop
def main():
    global image_pub, img, target_image, game_pub, temi_pub

    target_image = None
    rospy.init_node('face_recognition', anonymous=False)

    getPlayerList()
    initFaceRec()

    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    rospy.Subscriber("/frame", Frame, callback=frame_cb, queue_size = 10)
    rospy.Subscriber('/camera/facedetection', Image, callback=image_cb, queue_size=1)
    rospy.Subscriber('/register/name', String, callback=name_cb, queue_size=1)
    game_pub = rospy.Publisher('/game', String, queue_size=10)
    temi_pub = rospy.Publisher('/temi_cmd', TemiCMD, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
