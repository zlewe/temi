#! /usr/bin/env python3
from django.forms import TextInput
from torch import true_divide
import rospy
import numpy as np
import math
import time

from temi_driver.msg import TemiCMD
from ros_openpose.msg import Frame


class Falling():
    def __init__(self):
        rospy.init_node('Falling', anonymous=True)

        self.sub = rospy.Subscriber("/frame", Frame, self.callback)
        self.pub_cmd = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
        self.EMA = 50000.0
        self.last = time.time()
        self.threshold = 100000.0

        rospy.sleep(2)

        cmd = TemiCMD()
        cmd.type = 'cmd'
        cmd.arg = 'tiltAngle,-10'
        self.pub_cmd.publish(cmd)
        
    def isnull(self, bp):
        if bp.pixel.y==0.0:
            print("Null point..")
            return True
        return False

    def callback(self, msg):
        print("body---------------------------------------------")

        for skeleton in msg.persons:

                p0 = skeleton.bodyParts[0]
                p8 = skeleton.bodyParts[8]
                p11 = skeleton.bodyParts[11]
                p14 = skeleton.bodyParts[14]


                if p0.score>0.1 and p8.score > 0.01 and (p11.score > 0.01 or p14.score > 0.01):

                    if self.isnull(p0) or self.isnull(p8) or self.isnull(p11):
                        continue

                    fall_value = abs(p0.pixel.y-p8.pixel.y) * abs(p0.pixel.y-p11.pixel.y) * abs(p8.pixel.y-p11.pixel.y)
                    new_ema = fall_value*0.5 + self.EMA*0.5

                    #print(f'{abs(p0.pixel.y-p8.pixel.y)} * {abs(p0.pixel.y-p11.pixel.y)} * {abs(p8.pixel.y-p11.pixel.y)}')
                    #print(f'total:{fall_value}')
                    #print(f'ema:{self.EMA}')

                    if new_ema<self.threshold:
                        print("Bro wake up!!!!!!!!!!!!!!!!!!")

                        if (time.time()-self.last)>5:
                            self.last = time.time()
                            cmd = TemiCMD()
                            cmd.type = 'cmd'
                            cmd.arg = 'fall'
                            self.pub_cmd.publish(cmd)

                    if new_ema>=self.threshold and self.EMA<self.threshold:
                            cmd = TemiCMD()
                            cmd.type = 'cmd'
                            cmd.arg = 'return_initial'
                            self.pub_cmd.publish(cmd)

                    self.EMA = new_ema



        # y0 = p0

        # print((y0-y8)/(y0-y11))

        # print(data.human_list[0].body_key_points_with_prob)

        # print("face")
        # print(data.human_list.face_key_points_with_prob)

        # print("right")
        # print(data.human_list.right_hand_key_points_with_prob)

        # print("left")
        # print(data.human_list.left_hand_key_points_with_prob)

if __name__ == '__main__':
    Falling()
    rospy.spin()

    