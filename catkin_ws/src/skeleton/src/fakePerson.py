#!/usr/bin/env python
import numpy as np
import cv2
import rospy

# msg
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players,Player
from geometry_msgs.msg import Pose,PoseArray,PoseStamped

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

targetid = 0
player_stack = Players()

def mouse(event, x, y, flag, parms):
    global player_stack, targetid

    if event==1:

        player = Player()
        player.id = targetid
        player.id_score = 0.2
        player.position.position.x = x
        player.position.position.y = y

        player_stack.players.append(player)
        print('Add %d at (%d,%d) with %f score'%(targetid, x, y, player.id_score))
    return

def main():
    global id, goalpub,tfbuf, player_stack, targetid

    rospy.init_node('findSkeleton', anonymous=False)

    fakemap = np.zeros((600,600), dtype=np.uint8)
    cv2.imshow('map', fakemap)
    cv2.setMouseCallback('map', mouse)

    playerpub = rospy.Publisher('/players/withscore', Players, queue_size=10)

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
