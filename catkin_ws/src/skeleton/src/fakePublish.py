#!/usr/bin/env python
import os
import pickle
import numpy as np
import sys
import cv2
import rospy
import Skesim as ss
import tf2_ros
import tf2_geometry_msgs

# msg
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from temi_driver.msg import TemiCMD

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

PlayerNum=3

print("Re-directing the folder to poses...")
dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../poses'
os.chdir(dir_path)

print("Loading the poses...")
poses = None
with open(os.path.join(dir_path,'poses.pickle'), 'rb') as handle:
    poses = pickle.load(handle)
    
def main():
    global id, poses

    rospy.init_node('FakeImage', anonymous=False)

    image_pub = rospy.Publisher('camera/port', Image, queue_size=10)

    # Wait for few secs to make the publishing successful!!
    rate = rospy.Rate(0.5)
    rate.sleep()

    print('publish the fake image')
    image_pub.publish(poses[int(sys.argv[1])][1])
    rospy.spin()

if __name__ == '__main__':
    main()
