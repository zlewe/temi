#!/usr/bin/env python

import paho.mqtt.client as mqtt
import cv2
import rospy
import math
import json
import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np
import tf
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

mypose = None

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code ")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("position")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global mypose
    rospy.loginfo(msg.topic)

    if msg.topic == 'position':
        # rospy.loginfo(pos)
        pos = msg.payload.decode('utf-8').split(',')
        orientation = quaternion_from_euler(0, 0, float(pos[2]))

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        mypose = pose

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
    global pubpose

    mclient = mqtt.Client(client_id="tf_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mserver_ip = rospy.get_param('/mqtt_ip', "192.168.50.197")
    mserver_port = rospy.get_param('/mqtt_port', 1883)
    mclient.connect(mserver_ip, mserver_port, 60)

    br = tf.TransformBroadcaster()

    rospy.Subscriber('cmd_vel', Twist, move_cb, queue_size=1)
    pubpose = rospy.Publisher('pose', PoseStamped, queue_size=5)
    rospy.init_node('temi_tf', anonymous=False)
    rospy.loginfo("Start temi driver")
    mclient.loop_start()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if mypose is not None:
            orientation = (mypose.pose.orientation.x,mypose.pose.orientation.y,mypose.pose.orientation.z,mypose.pose.orientation.w)
            br.sendTransform((mypose.pose.position.x, mypose.pose.position.y, 0.0), orientation, rospy.Time.now(), "temi", "map")

            pubpose.publish(mypose)
            rate.sleep()
        

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()