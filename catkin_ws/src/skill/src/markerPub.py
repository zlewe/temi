#!/usr/bin/env python
import numpy as np
import cv2
import rospy

# msg
from visualization_msgs.msg import Marker,MarkerArray

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

img=None   
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]

def MakeMarker(markerarray, x, y, frame='temi', type=Marker.SPHERE, scale=0.1, text=''):
    marker = Marker()

    marker.id = len(markerarray.markers)
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.a=1.0
    marker.color.r=1.0
    marker.type = type
    marker.action = Marker.ADD
    if type == Marker.TEXT_VIEW_FACING:
        marker.text = text
    return marker  

def main():
    global cmd_pub

    rospy.init_node('Marker_Publisher', anonymous=False)
    pospub = rospy.Publisher('/locations', MarkerArray, queue_size=10)
    
    rospy.sleep(1)

    markers = MarkerArray()

    locations = [('Living Room',(0.4, -0.87)), ('Dinner Table',(0.53, -5.89)), ('Studio',(5.64, -2.29)), ('Room',(6.3, -4.0)), ('Base',(0, 0))]

    for name, position in locations:

        markers.markers.append(MakeMarker(markers, position[0], position[1], frame='map', type=Marker.SPHERE, scale=0.25))
        markers.markers.append(MakeMarker(markers, position[0], position[1]+0.2, frame='map', type=Marker.TEXT_VIEW_FACING, scale=0.35, text=name))

    pospub.publish(markers)

if __name__ == '__main__':
    main()
