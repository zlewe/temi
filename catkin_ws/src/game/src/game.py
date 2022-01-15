#!/usr/bin/env python3

from numpy.random.mtrand import randint
import paho.mqtt.client as mqtt
import numpy as np
import rospy
from enum import Enum
from geometry_msgs.msg import Pose
from tf.transformations import random_quaternion
from std_msgs.msg import String

#x,y,yaw
START_POSITION = "1.26,-3.57,0"

# class State(Enum):
#     REGISTER = 1
#     START = 2
#     DISPLAY = 3
#     VERIFY = 4
#     BARK = 5

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected to mqtt with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # client.subscribe("position")
    client.subscribe("game")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # rospy.loginfo(msg.topic)
    global state

    if msg.topic == 'game':
        message = msg.payload.decode('utf-8').split(',')
        print(message)
        if message[0] == 'start game':
            start_the_game()
    
        elif message[0] == 'pose_id':
            display_and_ping(int(message[1]))

        elif message[0] == 'detect_complete':
            eliminate_and_bark()

def start_the_game():
    #go to start position
    mclient.publish("cmd","goToPosition,"+START_POSITION)
    rospy.sleep(10)
    print("goToPosition")

    #CHECK IF DONE
    rospy.sleep(20)

    #ask for pose if
    mclient.publish("game","give_me_pose")
    rospy.sleep(5)
    display_and_ping(1)

def display_and_ping(pose_id):
    #countdown
#def eliminate():
    mclient.publish("cmd","countdown","3")
    rospy.sleep(1)
    mclient.publish("cmd","countdown","2")
    rospy.sleep(1)
    mclient.publish("cmd","countdown","1")
    rospy.sleep(1)
    mclient.publish("game","showpose,"+str(pose_id))
    rospy.sleep(3)
    mclient.publish("cmd","turnback")
    rospy.sleep(3)
    mclient.publish("cmd","scan")
    mclient.publish("game","detectpose")
    mclient.publish("cmd","turn")
    rospy.sleep(5)

def eliminate_and_bark():
    #gotoposition
    #bark
    #eliminated
    print('you out!!')


def game_cb(msg):
    msgs = msg.data.split(',')
    if msgs[0] == ''


def main():
    global mclient
    global state
    global pub

    state = None
    mclient = mqtt.Client(client_id="game_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mserver_ip = rospy.get_param("/mqtt_ip", "192.168.50.197")
    mserver_port = rospy.get_param("/mqtt_port", 1883)
    mclient.connect(mserver_ip, mserver_port, 60)

    rospy.Subscriber('game', String, callback=game_cb)
    pub = rospy.Publisher('game',String)
    rospy.init_node('game_loop', anonymous=False)
    rospy.loginfo("Start temi driver")
    
    mclient.loop_start()
    rospy.spin()

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()

