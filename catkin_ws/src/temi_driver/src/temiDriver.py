#!/usr/bin/env python

from logging import exception
import paho.mqtt.client as mqtt
import cv2
import rospy
import math
import json
import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np

# tf
import tf2_ros
import tf2_geometry_msgs

# functions
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# msg
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from temi_driver.msg import TemiCMD

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code ")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # client.subscribe("position")
    client.subscribe("map")
    client.subscribe("status")
    client.subscribe("game")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    rospy.loginfo(msg.topic)

    if msg.topic == 'status':
        status = msg.payload.decode('utf-8')
        sm = String()
        sm.data = status
        pubstatus.publish(sm)
        

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

    elif msg.topic == 'game':
        game_pub.publish(msg.payload)

def goal_cb(msg):

    try:
        # Add frame transformation to get the correct position in the map frame
        transform = tfbuf.lookup_transform('map', msg.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        goal_map = tf2_geometry_msgs.do_transform_pose(msg, transform)

        if transform is None or goal_map is None:
            exception('Transformation failed!')

        goal = goal_map.pose
        x = goal.position.x
        y = goal.position.y
        (roll, pitch, yaw) = euler_from_quaternion([goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w])
        mclient.publish("cmd", "goToPosition,"+str(x)+","+str(y)+","+str(yaw))
    except:
        rospy.logwarn('Not transformation between map and {0} was found!'.format(msg.header.frame_id))
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

def cmd_cb(cmd):
    r = rospy.Rate(10)  
    mclient.publish(cmd.type, cmd.arg)
    r.sleep()

    
    
def main():
    global mclient
    global pubpose
    global pubmap
    global pubstatus
    global tfbuf
    global game_pub


    mclient = mqtt.Client(client_id="cmd_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mserver_ip = rospy.get_param("/mqtt_ip", "192.168.50.197")
    mserver_port = rospy.get_param("/mqtt_port", 1883)
    mclient.connect(mserver_ip, mserver_port, 60)

    rospy.init_node('temi_driver', anonymous=False)
    rospy.Subscriber('cmd_vel', Twist, move_cb, queue_size=1)
    rospy.Subscriber('temi_cmd', TemiCMD, cmd_cb, queue_size=10)
    pubpose = rospy.Publisher('pose', PoseStamped, queue_size=5)
    pubmap = rospy.Publisher('temi_map', OccupancyGrid, latch=True, queue_size=1)
    pubstatus = rospy.Publisher('status', String, queue_size=5)
    game_pub = rospy.Publisher('game', String, queue_size=5)
    rospy.Subscriber('move_base_simple/goal', PoseStamped,  goal_cb)
    rospy.loginfo("Start temi driver")

    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    mclient.publish("cmd", "tiltAngle,"+str(0))
    mclient.loop_start()
    mclient.publish("cmd", "loadMap")

    rospy.spin()

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()