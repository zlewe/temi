#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sklearn.cluster import KMeans,DBSCAN

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from skeleton.msg import Players,Player
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from temi_driver.msg import TemiCMD
from game.msg import GameStatus
game_status = GameStatus()

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

player_stack = Players()

def visualizationOnFakeMap(x,y,radius,color,thicc):
    global fakemap
    ratio = 100.0
    fakemap = cv2.circle(fakemap, (int(y*ratio+fakemap.shape[1]/2), int(x*ratio)), radius, color=color, thickness = thicc)

def player_cb(msg):
    global fakemap, game_status, start_time, player_stack, gamestatud_pub

    if not game_status.status=='DETECT_STARTED':
        return

    for player in msg.players:
        player_stack.players.append(player)

        visualizationOnFakeMap(player.position.position.x, player.position.position.y, 3, getColor(player.id), 1)

        if player.id >-1 and player.id<game_status.player_num:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

    cv2.imshow('Mymap',fakemap)
    cv2.waitKey(1)

    if game_status.status == 'DETECT_STARTED':
        if (rospy.Time.now() - start_time).to_sec()>10 and len(player_stack.players)>game_status.player_num*25:
            playerpub.publish(ClusterPlayers())
            player_stack = Players()

            game_status.last_status = game_status.status
            game_status.status = 'CHECKOUT_STARTED'
            gamestatud_pub.publish(game_status)

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

# collect positions
def CollectPositions(players):
    pos = []
    for player in players.players:
        pos.append((player.position.position.x, player.position.position.y))

    return pos

def ClusterPlayers():
    global PlayerNum, fakemap

    pos = CollectPositions(player_stack)
    cluster = DBSCAN(eps=0.6, min_samples=10).fit(pos)
    #cluster = KMeans(n_clusters=2, random_state=0).fit(pos)

    ClusterNum = -1
    for i in cluster.labels_:
        ClusterNum = max(ClusterNum, i+1)
    
    # build list for all cluster
    clustered_players = []
    for i in range(ClusterNum):
        clustered_players.append([])

    # assign all players to corresponding cluster list
    for i in range(len(cluster.labels_)):
        if cluster.labels_[i]==-1:
            continue

        clustered_players[cluster.labels_[i]].append(player_stack.players[i])

        # for visualization only
        pos = player_stack.players[i].position.position

        visualizationOnFakeMap(pos.x, pos.y, 6, getColor(cluster.labels_[i]), 1)

    for i in range(ClusterNum):
        print('Collect %d player %d'%(len(clustered_players[i]), i))

    players = Players()
    for collection in clustered_players:
        if len(collection)==0:
            continue

        vote = np.zeros(game_status.player_num, dtype=float)
        player = Player()
        for temp in collection:
            
            if temp.id<0 or temp.id>=game_status.player_num or game_status.alive[temp.id]==0:
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
    
def gamestatus_cb(msg):
    global game_status, start_time, cmd_pub
    game_status = msg
    if game_status.status == 'DETECT_STARTED' and (not game_status.last_status == 'DETECT_STARTED'):
        start_time = rospy.Time.now()
        cmd_pub.publish(TemiCMD('cmd','scan'))

def main():
    global id, goalpub, tfbuf, player_stack, fakemap, playerpub, gamestatud_pub, start_time, cmd_pub

    rospy.init_node('HistoryDetection', anonymous=False)

    start_time = rospy.Time.now()
    fakemap = np.zeros((600,600,3), dtype=np.uint8)
    cv2.imshow('Mymap', fakemap)

    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    rospy.Subscriber('/players/withscore', Players,  callback=player_cb, queue_size=10)
    playerpub = rospy.Publisher('/players/final', Players, queue_size=10)
    gamestatud_pub = rospy.Publisher('/game_status', GameStatus, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        cv2.imshow('Mymap', fakemap)
        cv2.waitKey(1)
        rate.sleep()
    
if __name__ == '__main__':
    main()
