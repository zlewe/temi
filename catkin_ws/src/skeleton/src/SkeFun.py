"""
The script parsing the skeleton and estimate the joint angle of a human
 """
import sys
import os
import numpy as np
from sys import platform
import argparse

def mag(a, b):
    return np.linalg.norm(a-b)

def normalize(vec):
    return vec/np.linalg.norm(vec)

def valid(a):
    return (not a[0]==0) or (not a[1]==0.0)

def normalizeSkeleton(sk):
    sk=sk[:,0:2]
    neck = sk[1]
    button = sk[8]
    body = mag(neck, button)
    sk /= body
    return sk

def getHeadYaw(nsk):
    r=mag(nsk[15], nsk[0])
    l=mag(nsk[16], nsk[0])

    return 200.0*(r-l)/(r+l) 

def showVectors(vecs, name, scale, res=(512,512,3)):
    img=np.zeros(res, dtype=np.uint8)
    print(vecs)
    for v in vecs:
        cv2.line(img, (res[0], res[1]), (int(res[0]+v[0]*scale), int(res[1]+v[1]*scale)), (255,255,255), 2)
    cv2.imshow(name, img)
    
def getHeadPitch(nsk):

    # calculate the eye vector
    test = nsk[16]-nsk[15]

    # prepare the 90 deg rotation matrix
    c,s = np.cos(np.pi/2), np.sin(np.pi/2)
    r = np.asarray([[c,s],[-s,c]])
    
    # rotate the eye vector by 90 deg
    test = np.matmul(r,test)
    eye_mid = normalize(test)
    print(test, eye_mid)
    # find nose-to-ears vectors
    ear_r = normalize(nsk[17]-nsk[0])
    ear_l = normalize(nsk[18]-nsk[0])
    #showVectors(np.asarray((eye_mid, ear_l, ear_r)), 'test', 10.0)
        
    # use only valid nose-to-ear vectors to estimate
    if not valid(nsk[18]):
        ang = np.dot(eye_mid, ear_r)
    elif not valid(nsk[17]):
        ang = np.dot(eye_mid, ear_l)
    else:
        ang = np.dot(eye_mid, (ear_r+ear_l)/2)

    ang = 100*(ang-0.1)
    return ang

def leftarm(nsk):
    shoulder = normalize(nsk[5]-nsk[1])
    elbow = normalize(nsk[6]-nsk[5])
    return -np.arctan2(np.cross(shoulder,elbow), np.dot(shoulder,elbow))/np.pi*180.0+90.0

def leftelbow(nsk):
    shoulder = normalize(nsk[6]-nsk[5])
    elbow = normalize(nsk[7]-nsk[6])
    return np.arctan2(np.cross(shoulder,elbow), np.dot(shoulder,elbow))/np.pi*180.0
    
def leftshoulder(nsk):
    suogu = mag(nsk[1],nsk[5])
    arm = mag(nsk[6],nsk[5])
    return -120+(arm/suogu/1.5)*120.0


def rightarm(nsk):
    shoulder = normalize(nsk[2]-nsk[1])
    elbow = normalize(nsk[3]-nsk[2])
    return np.arctan2(np.cross(shoulder,elbow), np.dot(shoulder,elbow))/np.pi*180.0+90.0

def rightelbow(nsk):
    shoulder = normalize(nsk[3]-nsk[2])
    elbow = normalize(nsk[4]-nsk[3])
    return -np.arctan2(np.cross(shoulder,elbow), np.dot(shoulder,elbow))/np.pi*180.0
    
def rightshoulder(nsk):
    suogu = mag(nsk[1],nsk[2])
    arm = mag(nsk[3],nsk[2])
    return -120+(arm/suogu/1.5)*120.0

def getJointRot(nsk):
    pitch=getHeadPitch(nsk)
    yaw=getHeadYaw(nsk)
    la=leftarm(nsk)
    le=leftelbow(nsk)
    ls=leftshoulder(nsk)

    ra=rightarm(nsk)
    re=rightelbow(nsk)
    rs=rightshoulder(nsk)

    return pitch, yaw, la, le, ls, ra, re, rs


if __name__ == '__main__':
    import cv2
    import Opps as op

    cap = cv2.VideoCapture(0)
    cv2.namedWindow("QQ",cv2.WINDOW_NORMAL)

    norm=None
    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        cvoutput, keypoint = op.getpose(frame)
        '''if norm is None:
            norm = normalizeSkeleton(keypoint[0])
        norm = 0.5*norm + 0.5*normalizeSkeleton(keypoint[0])'''

        pitch, yaw, la, le, ls, ra, re, rs = getJointRot(normalizeSkeleton(keypoint[0]))
        if not pitch:
            continue

        cvoutput=cv2.putText(cvoutput, "%f"%pitch, (50,150), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0))
        cvoutput=cv2.putText(cvoutput, "%f"%yaw, (50,250), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0))
        cvoutput=cv2.putText(cvoutput, "%f"%ls, (50,350), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0))
        cv2.imshow("QQ", cvoutput)
        if cv2.waitKey(1)>0:
            break
