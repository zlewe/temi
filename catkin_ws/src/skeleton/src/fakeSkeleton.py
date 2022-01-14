#!/usr/bin/env python
import numpy as np
import cv2
import rospy

# msg
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players,Player
from geometry_msgs.msg import Pose,PoseArray,PoseStamped

# tf
import tf2_ros
import tf2_geometry_msgs

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

targetid = 0
player_stack = Players()

def mouse(event, x, y, flag, parms):
    global player_stack, targetid

    if event==1:


        # Transform the player poses to map frame for later usage
        stamp_pos = PoseStamped()
        stamp_pos.header.frame_id='temi'
        stamp_pos.header.stamp = rospy.Time.now()
        stamp_pos.pose.position.y = (x-fakemap.shape[1]/2.0)/100.0
        stamp_pos.pose.position.x = y/100.0

        # Add frame transformation to get the correct position in the map frame
        transform = tfbuf.lookup_transform('map', 'temi', rospy.Time.now(), rospy.Duration(4.0))
        posonmap = tf2_geometry_msgs.do_transform_pose(stamp_pos, transform)

        player = Player()
        player.id = -1
        player.id_score = 0.2
        player.position = posonmap.pose

        player_stack.players.append(player)
        print('Add %d at (%f,%f) with %f score'%(targetid, stamp_pos.pose.position.x, stamp_pos.pose.position.y, player.id_score))
    return

def main():
    global id, goalpub,tfbuf, player_stack, targetid, fakemap

    rospy.init_node('findSkeleton', anonymous=False)

    fakemap = np.zeros((600,600), dtype=np.uint8)
    cv2.imshow('map', fakemap)
    cv2.setMouseCallback('map', mouse)

    playerpub = rospy.Publisher('/players/final', Players, queue_size=10)

    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        cv2.imshow('map', fakemap)
        key = cv2.waitKey(1)
        
        if key == 27:
            playerpub.publish(player_stack)
            print('Publish {0} fake persons...'.format(len(player_stack.players)))
            player_stack = Players()
        elif key>=177 and key<=185:
            targetid=key-177
            print('Switch id to  %d'%targetid)
        rate.sleep()
    
if __name__ == '__main__':
    main()
