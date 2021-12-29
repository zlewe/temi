#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import cv2
import rospy
import math
import json
import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code ")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("position")
    client.subscribe("map")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
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

        pubpose.publish(pose)

    elif msg.topic == 'map':
        json_string = msg.payload.decode('utf-8')
        map = json.loads(json_string)
        # out_file = open("map.json", "w")
        # json.dump(json_dict, out_file)
        # out_file.close()
        # print(json_dict.keys())

        og = OccupancyGrid()
        og.header.frame_id = 'map'
        og.header.stamp = rospy.Time.now()
        og.data = map['Map_Image']['data']

        mtdt = MapMetaData()
        mtdt.map_load_time = rospy.Time.now()
        mtdt.resolution = map['mapInfo']['resolution']
        mtdt.width = map['Map_Image']['cols']
        print(mtdt.width)
        mtdt.height = map['Map_Image']['rows']
        print(mtdt.height)
        origin = Pose()
        origin.position.x = float(map['mapInfo']['origin_x'])
        origin.position.y = float(map['mapInfo']['origin_y'])
        origin.orientation.x = 0
        origin.orientation.y = 0
        origin.orientation.z = 0
        origin.orientation.w = 1
        mtdt.origin = origin
        og.info = mtdt

        pubmap.publish(og)

def goal_cb(msg):
    goal = msg.pose
    x = goal.position.x
    y = goal.position.y
    (roll, pitch, yaw) = euler_from_quaternion([goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w])
    mclient.publish("cmd", "goToPosition,"+str(x)+","+str(y)+","+str(yaw))
    #print(x,y,yaw)

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
    global pubmap

    mclient = mqtt.Client(client_id="cmd_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mclient.connect("192.168.50.64", 1883, 60)

    rospy.Subscriber('cmd_vel', Twist, move_cb, queue_size=1)
    pubpose = rospy.Publisher('pose', PoseStamped, queue_size=5)
    pubmap = rospy.Publisher('temi_map', OccupancyGrid, latch=True, queue_size=1)
    rospy.Subscriber('move_base_simple/goal', PoseStamped,  goal_cb)
    rospy.init_node('temi_driver', anonymous=False)
    rospy.loginfo("Start temi driver")
    mclient.loop_start()
    rospy.spin()

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()