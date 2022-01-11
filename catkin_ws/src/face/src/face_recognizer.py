#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
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
names = ['brain','ariel'] # add a name into this list
#Video Capture
cam = cv2.VideoCapture(0)
cam.set(3, 640)
cam.set(4, 480)
# Min Height and Width for the  window size to be recognized as a face
minW = 0.1 * cam.get(3)
minH = 0.1 * cam.get(4)
    

def imagecallback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(int(minW), int(minH)),
    )
    id = 0
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        id, confidence = recognizer.predict(gray[y:y + h, x:x + w])
        if (confidence < 100):
            print(id)
            id = names[id-1]
            confidence = "  {0}%".format(round(100 - confidence))
        else:
            # Unknown Face
            id = "Who are you ?"
            confidence = "  {0}%".format(round(100 - confidence))

        cv2.putText(img, str(id), (x + 5, y - 5), font, 1, (255, 255, 255), 2)
        cv2.putText(img, str(confidence), (x + 5, y + h - 5), font, 1, (255, 255, 0), 1)

    cv2.imshow('camera', img)
    if id=='ariel':
        cv2.waitKey(0)
    else:
        cv2.waitKey(1)

def main():
    global image_pub
    rospy.init_node('face_recognition', anonymous=False)

    initFaceRec()

    rospy.Subscriber('/camera/raw', Image, imagecallback, queue_size=1)
    image_pub = rospy.Publisher("/camera/undistorted",Image, queue_size = 1)

    rospy.spin()

if __name__ == '__main__':
    main()
