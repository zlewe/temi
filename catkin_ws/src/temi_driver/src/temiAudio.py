#!/usr/bin/env python

import numpy as np
import paho.mqtt.client as mqtt
import rospy
import pyaudio
import matplotlib.pyplot as plt

plt.ion()

audioList = []
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

player = pyaudio.PyAudio()
stream = player.open(format = FORMAT,
    channels = CHANNELS,
    rate = RATE,
    output = True)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code ")
    client.subscribe("audio")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global audioList
    '''num = msg.payload[0:4]
    num = num[::-1]
    print(int(codecs.encode(num, 'hex'), 16))'''
    
    if not msg.payload==b'':
        audioList.append(msg.payload)

    signal = np.fromstring(msg.payload, "Int16")
    plt.clf()
    plt.figure(1)
    plt.title("Signal Wave...")
    plt.plot(signal)
    plt.pause(0.001)
    
def main():
    global mclient, audioList

    mclient = mqtt.Client(client_id="audio_node")
    mclient.on_connect = on_connect
    mclient.on_message = on_message
    mserver_ip = rospy.get_param("/mqtt_ip", "192.168.50.197")
    mserver_port = rospy.get_param("/mqtt_port", 1883)
    mclient.connect(mserver_ip, mserver_port, 60)

    rospy.init_node('temi_audio_driver', anonymous=False)
    rospy.loginfo("Start temi audio driver")

    mclient.loop_start()

    while not rospy.is_shutdown():
        if len(audioList)>0:
            pop = audioList.pop()
            if not pop=='':
                stream.write(pop)
        rospy.Rate(50).sleep()

if __name__ == '__main__':
    main()