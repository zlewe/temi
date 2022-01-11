"""
Runtime program of skeleton similarity node
"""

import os
import cv2
import Opps as op
import numpy as np
import Skesim


print("Init cv2 stuff...")
cap = cv2.VideoCapture(0)
cv2.namedWindow("QQ",cv2.WINDOW_NORMAL)
cv2.namedWindow("Target",cv2.WINDOW_FREERATIO)

class PoseSet:

    set=[]  
    number=0
    last=None

    def __init__(self, path):

        files = os.listdir(path)

        print('Loading target frames...')
        #Pre-loading
        for filename in files:
            frame = cv2.imread(os.path.join(path,filename))

            cvoutput, keypoint = op.getpose(frame)
            
            if (keypoint is None) or (not len(keypoint)==1):
                continue

            target = Skesim.jointAngles(keypoint[0])
            
            self.set.append((frame, cvoutput, target))

            cv2.imshow("Target", cvoutput)
            cv2.waitKey(1)

        self.number = len(self.set)

    def pick(self, id): 
        if id<0 or id >= self.number:
            print("[ERROR] Index %d out of range (0-%d)!"%(id, self.number))
            return
        return self.set[id]

    def pickDiffRand(self):
        
        id = np.random.randint(0, len(self.set))
        while self.last is not None and id==self.last:
            id = np.random.randint(0, len(self.set))

        return self.pick(id)

pose = PoseSet('pose')

#Main loop for task
while True:
    target = pose.pickDiffRand()

    #Show the target pose
    cv2.imshow("Target", target[0])
    cv2.waitKey(1)

    #loop for detection
    while True:    
        ret, frame = cap.read()
        if not ret:
            continue

        cvoutput, keypoint = op.getpose(frame)
        if keypoint is None:
            continue

        #Parse all the skeletons in the scene
        min_score = 1.0
        for sk in keypoint:
            jangle = Skesim.jointAngles(sk)

            score, mins = Skesim.similarity(jangle, target[2])
            new_score = (score+mins)/2.0

            color = (0,0,255) if new_score<0.85 else (0,255,0)
            cvoutput=cv2.putText(cvoutput, "%.1f"%(new_score*100.0), np.int0(sk[0,:2])+np.asarray((50,50)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, thickness=6)

            min_score = new_score if new_score < min_score else min_score
        
        cv2.imshow("QQ", cvoutput)
        cv2.waitKey(1)

        if min_score>0.85:
            break
            

