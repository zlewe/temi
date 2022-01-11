import cv2 as cv
import numpy as np

# some variables for later usage from openCV documantation
# REF:https://docs.opencv.org/3.4.15/dc/dbb/tutorial_py_calibration.html

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cv.namedWindow('corners', cv.WINDOW_KEEPRATIO)
cv.resizeWindow('corners', 500, 500)

def AnalyzeOneImage(img, config, gridsize):
    global objpoints, imgpoints
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((config[0]*config[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:config[0],0:config[1]].T.reshape(-1,2)*gridsize

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, config, None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, config, corners2, ret)
        cv.imshow('corners', img)
        cv.waitKey(1)