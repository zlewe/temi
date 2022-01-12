#!/usr/bin/env python3
import numpy as np
import cv2
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
faceCascade = cv2.CascadeClassifier(face_cascade_Path)
if (faceCascade):
    print("First true!")
cam = cv2.VideoCapture(0)
cam.set(3,640)
cam.set(4,480)
count = 0

if not cam.isOpened():
    raise IOError("Cannot open webcam")
# For each person, enter one unique numeric face id
face_id = input('\n enter user id (MUST be an integer) and press <return> -->  ')
print("\n [INFO] Initializing face capture. Look the camera and wait ...")


while(True):
    ret, img = cam.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = faceCascade.detectMultiScale(gray, 1.3, 5) #error!!
    # if(faces):
    #     print("Second true!")
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
        count += 1
        # Save the captured image into the images directory
        cv2.imwrite(os.path.join(dir_path,'images/Users.') + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
        cv2.imshow('image', img)
    # Press Escape to end the program.
    k = cv2.waitKey(100) & 0xff
    if k < 30:
        break
    # Take 30 face samples and stop video. You may increase or decrease the number of
    # images. The more the better while training the model.
    elif count >= 30:
         break

print("\n [INFO] Exiting Program.")
cam.release()
cv2.destroyAllWindows()
