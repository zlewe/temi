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

face_id = input('\n enter user id (MUST be an integer) and press <return> -->  ')
print("\n [INFO] Initializing face capture. Look the camera and wait ...")

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
    bbx[1] = min(img.shape[0], bbx[1]+100)
    bbx[3] = max(0, bbx[3]-100)

    bbx[0] = min(img.shape[1], bbx[0]+75)
    bbx[2] = max(0, bbx[2]-75)

    return img[bbx[3]:bbx[1], bbx[2]:bbx[0], :]
    

count = 0
def frame_cb(msg):
    global target_image, count

    if target_image is None:
        return

    for skeleton in msg.persons:
        img = getHeadPic(target_image, skeleton)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = faceCascade.detectMultiScale(gray, 1.3, 5)

        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
            count += 1
            # Save the captured image into the images directory
            cv2.imwrite(os.path.join(dir_path,'images/Users.') + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
            cv2.imshow('image', img)
            cv2.waitKey(1)
            if count>100:
                break
        if count>100:
            break


def main():
    global image_pub, img, target_image

    target_image = None
    rospy.init_node('face_recognition', anonymous=False)

    initFaceRec()

    rospy.Subscriber("/frame", Frame, callback=frame_cb, queue_size = 10)
    rospy.Subscriber('/camera/raw', Image, callback=image_cb, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
