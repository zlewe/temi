#!/usr/bin/env python3

import cv2
import numpy as np
from numpy.core.defchararray import asarray
from numpy.lib.type_check import imag
import rospy
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Player, Players
bridge = CvBridge()


dir_path = os.path.dirname(os.path.realpath(__file__))

def initFaceRec():
    global recognizer, faceCascade, font
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(os.path.join(dir_path,'trainer.yml'))

    face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
    faceCascade = cv2.CascadeClassifier(face_cascade_Path)

    font = cv2.FONT_HERSHEY_SIMPLEX

id = 0
# names related to ids: The names associated to the ids: 1 for Mohamed, 2 for Jack, etc...
names = ['brain','ariel', 'jj'] # add a name into this list
#Video Capture
cam = cv2.VideoCapture(0)
cam.set(3, 640)
cam.set(4, 480)
# Min Height and Width for the  window size to be recognized as a face
minW = 0.1 * cam.get(3)
minH = 0.1 * cam.get(4)

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
    global target_image

    if target_image is None:
        return

    players = Players()

    for skeleton in msg.persons:
        img = getHeadPic(target_image, skeleton)
        if img is None:
            continue

        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        except:
            print(img)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(int(minW), int(minH)),
        )
        id = -1
        myname = "nope"
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            id, confidence = recognizer.predict(gray[y:y + h, x:x + w])
            if (confidence < 100):
                myname = names[id]
                confidence = "  {0}%".format(round(100 - confidence))
            else:
                # Unknown Face
                id=-1
                myname = "Who are you ?"
                confidence = "  {0}%".format(round(100 - confidence))

            cv2.putText(img, myname, (x + 5, y - 5), font, 1, (255, 255, 255), 2)
            cv2.putText(img, str(confidence), (x + 5, y + h - 5), font, 1, (255, 255, 0), 1)
        if not myname == 'nope':
            cv2.imshow('{0}'.format(myname),img)
            cv2.waitKey(1)

        player = Player()
        player.id = id
        player.posture.skeleton = skeleton
        players.players.append(player)

    playerpub.publish(players)


def main():
    global image_pub, img, playerpub, target_image

    target_image = None
    rospy.init_node('face_recognition', anonymous=False)

    initFaceRec()

    rospy.Subscriber("/frame", Frame, callback=frame_cb, queue_size = 10)
    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=1)
    playerpub = rospy.Publisher('/players/withid', Players, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
