#!/usr/bin/env python3
import os
import cv2
import rospy
import json
import numpy as np
from PIL import Image

# msgs
from std_msgs.msg import String
from temi_driver.msg import TemiCMD
from game.msg import GameStatus
game_status = GameStatus()

dir_path = os.path.dirname(os.path.realpath(__file__))
players_path = os.path.join(dir_path, 'players.json')
img_path = os.path.join(dir_path, 'images')

# player list 
players_data = {'names':[]}

recognizer = cv2.face.LBPHFaceRecognizer_create()

#Haar cascade file
face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
detector = cv2.CascadeClassifier(face_cascade_Path)

# read the players.json file to know player numbers and names
def getPlayerList():
    global players_data, players_path

    if os.path.exists(players_path):
        print('Found players data, ready to load...')

        with open(players_path) as f:
            players_data = json.load(f)
    else:
        players_data = {'names':[]}

def getImagesAndLabels(path):
    imagePaths = [os.path.join(path,f) for f in os.listdir(path)]
    faceSamples=[]
    ids = []
    for imagePath in imagePaths:
        # convert it to grayscale
        PIL_img = Image.open(imagePath).convert('L')
        img_numpy = np.array(PIL_img,'uint8')
        id = int(os.path.split(imagePath)[-1].split(".")[1])
        faces = detector.detectMultiScale(img_numpy)
        for (x,y,w,h) in faces:
            faceSamples.append(img_numpy[y:y+h,x:x+w])
            ids.append(id)

    return faceSamples,ids

def game_cb(msg):
    global game_status

    cmd=msg.data.split(',')

    if len(cmd)<=0:
        return

    if cmd[0] == 'start game':

        print ("Start training faces...")
        faces,ids = getImagesAndLabels(img_path)
        recognizer.train(faces, np.array(ids))

        getPlayerList()

        # Save the model into the current directory.
        recognizer.write(os.path.join(dir_path, 'trainer.yml'))
        print("Totally {0} faces trained.".format(len(np.unique(ids))))

        game_status = GameStatus()
        game_status.last_status = 'REGISTER'
        game_status.status = 'START_GAME'
        game_status.player_num = len(cmd)-1
        game_status.alive = [1 for i in range(game_status.player_num)]
        game_status.names = cmd[:-1]
        gamestatus_pub.publish(game_status)
        print('Game Start!')

def main():
    global image_pub, img, target_image, game_pub, temi_pub, gamestatus_pub

    target_image = None
    rospy.init_node('initializeGame', anonymous=False)

    rospy.Subscriber('/game', String, callback=game_cb, queue_size=10)
    #rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    gamestatus_pub = rospy.Publisher('/game_status', GameStatus, queue_size=10)
    temi_pub = rospy.Publisher('/temi_cmd', TemiCMD, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()