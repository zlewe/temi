#!/usr/bin/env python3
import cv2
import numpy as np
from PIL import Image
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
#Directory path name where the face images are stored.
path = os.path.join(dir_path, 'images')
recognizer = cv2.face.LBPHFaceRecognizer_create()
#Haar cascade file
face_cascade_Path = os.path.join(dir_path,'haarcascade_frontalface_default.xml')
detector = cv2.CascadeClassifier(face_cascade_Path)

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

    print(ids)
    return faceSamples,ids
print ("\n[INFO] Training faces...")
faces,ids = getImagesAndLabels(path)
recognizer.train(faces, np.array(ids))
# Save the model into the current directory.
recognizer.write(os.path.join(dir_path, 'trainer.yml'))
print("\n[INFO] {0} faces trained. Exiting Program".format(len(np.unique(ids))))