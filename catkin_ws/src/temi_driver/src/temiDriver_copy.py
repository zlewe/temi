#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import cv2
import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from tf.transformations import quaternion_from_euler

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code ")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("position")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    pos = msg.payload.decode('utf-8').split(',')
    rospy.loginfo(msg.topic+" "+pos[0]+" "+pos[1]+" "+pos[2])

    # position = Point(pos[0], pos[1], 0)
    # orientation = quaternion_from_euler(0, 0, pos[2])

    # pose = PoseStamped()
    # pose.header.frame_id = 'map'
    # pose.header.stamp = rospy.Time.now()
    # pose.pose.position = position
    # pose.pose.orientation = orientation

    # pubpose.publish(pose)

def move_cb(data):
    r = rospy.Rate(10)
    #rate for move commands
    x = data.linear.x/0.7
    y = data.angular.z/0.4
    if (abs(y) >= 0.01):
        y = math.copysign(1,y)

    mclient.publish("cmd", "skidJoy,"+str(x)+","+str(y))
    r.sleep()

def main():
    global mclient
    #global pubpose

    mclient = mqtt.Client(client_id="cmd_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mclient.connect("192.168.50.64", 1883, 60)
    rospy.init_node('temi_driver', anonymous=False)

    rospy.Subscriber('cmd_vel', Twist, move_cb, queue_size=1)
    #pubpose = rospy.Publisher('pose', Pose, queue_size=5)
    rospy.loginfo("Start temi driver")
    #rospy.spin()

    mclient.loop_forever()

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()