#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sklearn.cluster import KMeans

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players,Player
from geometry_msgs.msg import Pose,PoseArray,PoseStamped

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

targetid = 0
PlayerNum = 3
alive = [1,1,1]
player_stack = Players()

def visualizationOnFakeMap(x,y,radius,color,thicc):
    global fakemap
    fakemap = cv2.circle(fakemap, (int(y*100+fakemap.shape[1]/2), int(x*100)), radius, color=color, thickness = thicc)

def player_cb(msg):
    global fakemap, PlayerNum
    for player in msg.players:
        player_stack.players.append(player)

        visualizationOnFakeMap(player.position.position.x, player.position.position.y, 3, getColor(player.id), 1)

        if player.id >-1 and player.id<PlayerNum:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

    cv2.imshow('Mymap',fakemap)
    cv2.waitKey(1)

def getColor(id):

    if id==0:
        color = (255,0,0)
    elif id==1:
        color = (0,255,0)
    elif id==2:
        color = (0,0,255)
    else:
        color = (255,255,255)
    return color 

def ClusterPlayers():
    global PlayerNum, fakemap

    # collect positions
    pos = []
    for player in player_stack.players:
        pos.append((player.position.position.x, player.position.position.y))

    cluster = KMeans(n_clusters=PlayerNum, random_state=0).fit(pos)
    
    # build list for all cluster
    clustered_players = []
    for i in range(PlayerNum):
        clustered_players.append([])

    # assign all players to corresponding cluster list
    for i in range(len(cluster.labels_)):
        clustered_players[cluster.labels_[i]].append(player_stack.players[i])

        # for visualization only
        pos = player_stack.players[i].position.position

        visualizationOnFakeMap(pos.x, pos.y, 6, getColor(player.id), 1)

    for i in range(PlayerNum):
        print('Collect %d player %d'%(len(clustered_players[i]), i))

    players = Players()
    for collection in clustered_players:
        vote = np.zeros(PlayerNum, dtype=float)
        player = Player()
        for temp in collection:
            
            if temp.id<0 or temp.id>=PlayerNum or alive[temp.id]==0:
                temp.id=-1
                
            if temp.id>-1 and temp.id_score > 0.0:
                vote[temp.id] += temp.id_score

            player.position.position.x += temp.position.position.x
            player.position.position.y += temp.position.position.y
            
            if temp.score > player.score:
                player.score = temp.score
                player.posture = temp.posture

        player.position.position.x /= len(collection)
        player.position.position.y /= len(collection)
        player.id = np.argmax(vote, axis=0)
        print(player.id, vote)
        if vote[player.id]<=0.0:
            player.id=-1
        players.players.append(player)

    print('Totally %d'%(len(players.players)))

    for p in players.players:

        visualizationOnFakeMap(p.position.position.x, p.position.position.y, 10, getColor(p.id), 3)
    cv2.imshow('Mymap',fakemap)
    cv2.waitKey(1)

    return players

def command_cb(msg):
    global player_stack
    if msg == 'DETECT':
        playerpub.publish(ClusterPlayers())
        player_stack = Players()

def main():
    global id, goalpub,tfbuf, player_stack, targetid, fakemap, playerpub

    rospy.init_node('HistoryDetection', anonymous=False)

    fakemap = np.zeros((600,600,3), dtype=np.uint8)
    cv2.imshow('Mymap', fakemap)

    playerpub = rospy.Publisher('/players/final', Players, queue_size=10)
    rospy.Subscriber('/players/withscore', Players,  callback=player_cb, queue_size=10)
    rospy.Subscriber('/game', String,  callback=command_cb, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        cv2.imshow('Mymap', fakemap)
        key = cv2.waitKey(1)
        if key == 27:
            ClusterPlayers()
        rate.sleep()
    
if __name__ == '__main__':
    main()
