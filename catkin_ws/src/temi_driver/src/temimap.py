import rospy
import math
import json
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData

#open file
f = open('map.json')
map = json.load(f)

# print(map.keys())
rospy.init_node('map_server')

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
# og = OccupancyGrid()
# og.header.frame_id = 'map'
# og.header.stamp = rospy.Time.now()
# og.data = map['data']

# mtdt = MapMetaData()
# mtdt.map_load_time = rospy.Time.now()
# mtdt.resolution = 0.05
# mtdt.width = map['cols']
# mtdt.height = map['rows']

# origin = Pose()
# origin.position.x = -5.4
# origin.position.y = -7.4
# origin.orientation.x = 0
# origin.orientation.y = 0
# origin.orientation.z = 0
# origin.orientation.w = 1

# mtdt.origin = origin

# og.info = mtdt

mappub = rospy.Publisher('temi_map', OccupancyGrid, latch=True, queue_size=1)

mappub.publish(og)

rospy.spin()