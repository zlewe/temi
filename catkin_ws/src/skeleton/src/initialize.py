#!/usr/bin/env python
import os
import sys
import pickle
import numpy as np
import cv2
import cv_bridge
import rospy
import rosnode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_openpose.msg import Frame
from geometry_msgs.msg import Pose,PoseArray

bridge = CvBridge()

print("Re-directing the folder to poses...")
dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../poses/raw'
os.chdir(dir_path)

# Read target images
id = 0
images = [(name.replace('.jpg',''), cv2.imread(name)) for name in os.listdir(dir_path) if '.jpg' in name]
print("{0} target pose images were found in the folder...".format(len(images)))

dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../poses'
os.chdir(dir_path)

poses = []
def getSkeleton(msg):
    global id, images, poses

    for skeleton in msg.persons:

        data = (images[id][0], bridge.cv2_to_imgmsg(images[id][1]), skeleton)
        poses.append(data)
        print('Append the skeleton of image {0}'.format(images[id][0]))
        # Save the first skeleton only, just in case
        break

    id+=1
    if id<len(images):
        imgpub.publish(bridge.cv2_to_imgmsg(images[id][1], encoding='bgr8'))
    else:
        print('Pack up and save all target poses...')
        with open(os.path.join(dir_path,'poses.pickle'), 'wb') as handle:
            pickle.dump(poses, handle, protocol=pickle.HIGHEST_PROTOCOL)

        rospy.signal_shutdown('Process ends...exiting')

    

def main():
    global imgpub, id, getSkeleton

    rospy.init_node('findSkeleton', anonymous=False)
    rospy.Subscriber('/frame', Frame, callback=getSkeleton, queue_size=10)
    imgpub = rospy.Publisher('/camera/undistorted', Image, queue_size=1)

    # Wait for few secs to make the publishing successful!!
    rate = rospy.Rate(0.5)
    rate.sleep()
    
    if id<len(images):
        print("Publish the first pose to trigger the following process...")
        imgpub.publish(bridge.cv2_to_imgmsg(images[id][1], encoding='bgr8'))
        rospy.spin()

if __name__ == '__main__':
    main()
