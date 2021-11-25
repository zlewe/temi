import paho.mqtt.client as mqtt
import cv2
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("position")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    pos = msg.payload.decode('utf-8').split(',')
    print(msg.topic+" "+pos[0]+" "+pos[1]+" "+pos[2])

def move_cb(data):
    r = rospy.Rate(10)
    x = data.linear.x/0.7*1
    y = data.angular.z/0.4*1
    if (abs(y) >= 0.01):
        y = math.copysign(1,y)

    client.publish("cmd", "skidJoy,"+str(x)+","+str(y))
    r.sleep()

def main():
    global client
    client = mqtt.Client(client_id="cmd_node")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("192.168.50.64", 1883, 60)

    rospy.Subscriber('cmd_vel', Twist, move_cb, queue_size=1)

    rospy.init_node('temi_driver', anonymous=False)
    rospy.loginfo("Start temi driver")
    rospy.spin()



    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.

if __name__ == '__main__':
    main()