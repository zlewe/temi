#!/usr/bin/env python3

from glob import glob
import cv2
import numpy as np
from numpy.core.defchararray import asarray
from numpy.lib.type_check import imag
import rospy
import os

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Player, Players
from game.msg import GameStatus
game_status = GameStatus()

# cv bridge import
from cv_bridge import CvBridge
bridge = CvBridge()

dir_path = os.path.dirname(os.path.realpath(__file__))

def initFaceRec():
    global recognizer, faceCascade, font
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(os.path.join(dir_path,'trainer.yml'))

    face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
    faceCascade = cv2.CascadeClassifier(face_cascade_Path)

    font = cv2.FONT_HERSHEY_SIMPLEX

def image_cb(msg):
    global target_image
    target_image = bridge.imgmsg_to_cv2(msg)


def imagecallback(msg):
    img = bridge.imgmsg_to_cv2(msg)

def isJointValid(j):
    return j.pixel.x>0.0 or j.pixel.y>0.0

def getHeadPic(img, s):
    
    bbx = np.asarray((0,0,img.shape[1], img.shape[0]), dtype=float)
    
    for i in [0, 15, 16, 17, 18]:
        if isJointValid(s.bodyParts[i]):
            bbx[0] = max(bbx[0], s.bodyParts[i].pixel.x)
            bbx[1] = max(bbx[1], s.bodyParts[i].pixel.y)
            bbx[2] = min(bbx[2], s.bodyParts[i].pixel.x)
            bbx[3] = min(bbx[3], s.bodyParts[i].pixel.y)
    bbx = np.int0(bbx)

    # inflate the range
    bbx[1] = min(img.shape[0], bbx[1]+75)
    bbx[3] = max(0, bbx[3]-75)

    bbx[0] = min(img.shape[1], bbx[0]+50)
    bbx[2] = max(0, bbx[2]-50)

    ret = img[bbx[3]:bbx[1], bbx[2]:bbx[0], :]
    if len(ret)==0:
        return None
    return ret
    

def frame_cb(msg):
    global target_image, jj_pub, brian_pub, ariel_pub

    if (target_image is None) or (not game_status.status=='DETECT_STARTED'):
        return

    players = Players()

    for skeleton in msg.persons:
        img = getHeadPic(target_image, skeleton)
        if img is None:
            continue
        cv2.imshow('123',img)
        cv2.waitKey(1)
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except:
            pass

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(int(gray.shape[1]*0.5), int(gray.shape[0]*0.5)),)

        id = -1
        confidence = 0.0
        myname = "nope"

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            new_id, conf = recognizer.predict(gray[y:y + h, x:x + w])
            if conf < 100 and conf>confidence:
                id = new_id
                myname = game_status.names[id]
                confidence = conf

        if id >= 0:
            cv2.putText(img, myname, (x + 5, y - 5), font, 1, (255, 255, 255), 2)
            cv2.putText(img, "  {0}%".format(round(100 - confidence)), (x + 5, y + h - 5), font, 1, (255, 255, 0), 1)
            '''cv2.imshow('{0}'.format(myname),img)
            cv2.waitKey(1)'''

            if id==0:
                brian_pub.publish(bridge.cv2_to_imgmsg(img))
            elif id==1:
                ariel_pub.publish(bridge.cv2_to_imgmsg(img))
            elif id==2:
                jj_pub.publish(bridge.cv2_to_imgmsg(img))
            else:
                unknown_pub.publish(bridge.cv2_to_imgmsg(img))
            

        player = Player()
        player.id = id
        player.id_score = confidence
        player.posture.skeleton = skeleton
        players.players.append(player)

        if player.id >-1 and player.id<game_status.player_num:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

    playerpub.publish(players)

def gamestatus_cb(msg):
    global game_status, game_pub
    game_status = msg
    
    # Start Detection stage!!!!!!!!!!!!!!
    if game_status.status=='DETECT_STARTED' and game_status.last_status=='DISPLAY_POSE':
        rospy.sleep(2)
        game_pub.publish('START_FACEDETECTION')
        game_pub.publish('START_OPENPOSE')

def main():
    global image_pub, img, playerpub, target_image, game_pub, jj_pub, brian_pub, ariel_pub, unknown_pub

    target_image = None
    rospy.init_node('FaceRecognition', anonymous=False)

    initFaceRec()

    rospy.Subscriber("/frame", Frame, callback=frame_cb, queue_size = 10)
    rospy.Subscriber('/camera/facedetection', Image, callback=image_cb, queue_size=1)
    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    playerpub = rospy.Publisher('/players/withid', Players, queue_size=10)
    game_pub = rospy.Publisher('/game', String, queue_size=10)

    jj_pub = rospy.Publisher('playerimg/jj', Image, queue_size=1)
    brian_pub = rospy.Publisher('playerimg/brian', Image, queue_size=1)
    ariel_pub = rospy.Publisher('playerimg/ariel', Image, queue_size=1)
    unknown_pub = rospy.Publisher('playerimg/ariel', Image, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
