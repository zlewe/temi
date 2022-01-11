#!/usr/bin/env python
from ctypes import pointer
import os
import pickle
import sys
import numpy as np
import cv2
from cv2 import fisheye
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dir_path = os.path.dirname(os.path.realpath(__file__))
bridge = CvBridge()

# module for camera calibration
import caca

# set the calibration mode
if len(sys.argv)>=2 and sys.argv[1]=='post':
    topic = '/camera/undistorted'
    filename = 'calibration_after.pickle'

    print('Post-calibration mode')
else:
    topic = '/camera/raw'
    filename = 'calibration.pickle'

    print('Pre-calibration mode')

imgsize = (0,0)
# Main loop receiving images from temi to calibrate
def imagecallback(msg):
    global imgsize
    # Decode the jpeg image    
    image = bridge.imgmsg_to_cv2(msg)
    imgsize = image.shape[:-1]
    key = cv2.waitKey(1)

    if key == 10:
        print('Analyze image now...{0}'.format(len(caca.objpoints)))
        caca.AnalyzeOneImage(image, (8, 6), 1.0)

    elif key == 27:

        print('Save the img and obj points for later use...')
        # Save the img and obj points for later use
        points = {'obj':caca.objpoints, 'img':caca.imgpoints, 'size':imgsize}
        with open(os.path.join(dir_path, 'points.pickle'), 'wb') as handle:
            pickle.dump(points, handle, protocol=pickle.HIGHEST_PROTOCOL)

        print('Calibrating...')
        calibrate()

    elif key==8:

        print('Load points from points.pickle...')
        with open(os.path.join(dir_path, 'points.pickle'), 'rb') as handle:
            points = pickle.load(handle)
            imgsize = points['size']
            caca.objpoints = points['obj']
            caca.imgpoints = points['img']

        print('Calibrating...')
        calibrate()

    else:
        cv2.imshow('raw',image)

def calibrate():
    if len(caca.objpoints)==0:
        print('No chessboard founded! Exit!')
        exit()

    #Calculate the calibration matrices

    print('Calculate calibration matrices form {0} {1} frames.'.format(len(caca.objpoints), imgsize))

    # fisheye model calibration, not used now
    '''N_imm = len(caca.objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
    ret, mtx, dist, rvecs, tvecs = fisheye.calibrate(caca.objpoints, caca.imgpoints,imgsize,K,D, rvecs, tvecs, criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))'''
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(caca.objpoints, caca.imgpoints, imgsize, None, None)

    # Save the calibration result with pickle for later usage
    # REF:https://stackoverflow.com/questions/11218477/how-can-i-use-pickle-to-save-a-dict

    print('Save calibration matrices to {0}...'.format('calibration'))
    cammat={'ret':ret, 'mtx': mtx, 'dist': dist, 'rvecs': rvecs, 'tvecs': tvecs}
    with open(os.path.join(dir_path, filename), 'wb') as handle:
        pickle.dump(cammat, handle, protocol=pickle.HIGHEST_PROTOCOL)

def main():
    cv2.namedWindow('raw', cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow('raw', 480, 640)

    rospy.Subscriber(topic, Image, imagecallback, queue_size=1)
    rospy.init_node('calibrater', anonymous=False)
    rospy.spin()

if __name__ == '__main__':
    main()
