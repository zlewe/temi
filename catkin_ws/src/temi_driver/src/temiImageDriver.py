#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("image")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # print(msg.topic+" "+str(msg.payload))
    # print(bytearray(msg.payload))
    image = np.asarray(bytearray(msg.payload), dtype="uint8")
    
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    image = cv2.flip(image, -1)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
        print(e)
    # cv2.imshow('image',image)
    # cv2.waitKey(1)

def main():
    global image_pub
    global bridge
    global on_message, on_connect

    rospy.init_node('image_converter', anonymous=True)

    image_pub = rospy.Publisher("/camera/raw",Image, queue_size = 10)
    bridge = CvBridge()

    client = mqtt.Client(client_id="image_node")
    client.on_connect = on_connect
    client.on_message = on_message

    mserver_ip = rospy.get_param('/mqtt_ip', "192.168.50.197")
    print(mserver_ip)
    mserver_port = rospy.get_param('/mqtt_port', 1883)
    client.connect(mserver_ip, mserver_port, 60)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()

if __name__ == "__main__":
    main()

