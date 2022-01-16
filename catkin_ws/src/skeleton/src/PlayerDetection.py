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
from visualization_msgs.msg import Marker,MarkerArray
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


def MakeMarker(markerarray, x, y, frame='temi', type=Marker.SPHERE, scale=0.1, text=''):
    marker = Marker()

    marker.id = len(markerarray.markers)
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.a=1.0
    marker.color.r=1.0
    marker.type = type
    marker.action = Marker.ADD
    if type == Marker.TEXT_VIEW_FACING:
        marker.text = text
    return marker  

def ClusterPlayers():
    global PlayerNum, fakemap, pospub, namepub

    pos = CollectPositions(player_stack)
    cluster = DBSCAN(eps=0.6, min_samples=10).fit(pos)
    #cluster = KMeans(n_clusters=2, random_state=0).fit(pos)

    ClusterNum = -1
    for i in cluster.labels_:
        ClusterNum = max(ClusterNum, i+1)

    alive_num = np.sum(game_status.alive)
    wanted_list = []
    if ClusterNum > alive_num:
        print('Detected player number (%d) is larger than alive player (%d).'%(ClusterNum, alive_num))

        num_arr=np.zeros(ClusterNum, dtype=int)
        for i in range(ClusterNum):
            num_arr[i] = np.sum(np.where(cluster.labels_==i))

        for i in range(alive_num):
            max_arg = np.argmax(num_arr)
            wanted_list.append(max_arg)
            num_arr[max_arg]=-1
    else:
        wanted_list = range(ClusterNum)

    
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
    for i in wanted_list:

        collection = clustered_players[i]
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
        
        sum = np.sum(vote)
        if sum>0.0:
            player.confidence_array = vote/np.sum(vote)
        else:
            player.confidence_array = vote

        players.players.append(player)

    print('Totally %d'%(len(players.players)))

    markers = MarkerArray()
    markers_name = MarkerArray()

    for p in players.players:
        visualizationOnFakeMap(p.position.position.x, p.position.position.y, 10, getColor(p.id), 3)

        playername = 'unknown'
        if p.id>-1:
            playername = game_status.names[p.id]
        new_pose = MakeMarker(markers, p.position.position.x, p.position.position.y, frame='map', type=Marker.SPHERE, scale=0.1)
        markers.markers.append(new_pose)
        markers_name.markers.append(MakeMarker(markers_name, p.position.position.x, p.position.position.y+0.3, frame='map', type=Marker.TEXT_VIEW_FACING, scale=0.5, text=playername))

    pospub.publish(markers)
    namepub.publish(markers)

    cv2.imshow('Mymap',fakemap)
    cv2.waitKey(1)

    return players
    
def gamestatus_cb(msg):
    global game_status, start_time, cmd_pub, fakemap
    game_status = msg
    if game_status.status == 'DETECT_STARTED' and (not game_status.last_status == 'DETECT_STARTED'):
        fakemap = np.zeros((600,600,3), dtype=np.uint8)
        start_time = rospy.Time.now()
        cmd_pub.publish(TemiCMD('cmd','scan'))

def main():
    global id, goalpub, tfbuf, player_stack, fakemap, playerpub, gamestatud_pub, start_time, cmd_pub, pospub, namepub

    rospy.init_node('HistoryDetection', anonymous=False)

    start_time = rospy.Time.now()
    fakemap = np.zeros((600,600,3), dtype=np.uint8)
    cv2.imshow('Mymap', fakemap)

    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    rospy.Subscriber('/players/withscore', Players,  callback=player_cb, queue_size=10)
    playerpub = rospy.Publisher('/players/final', Players, queue_size=10)
    pospub = rospy.Publisher('/players', MarkerArray, queue_size=10)
    namepub = rospy.Publisher('/players_name', MarkerArray, queue_size=10)
    gamestatud_pub = rospy.Publisher('/game_status', GameStatus, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        cv2.imshow('Mymap', fakemap)
        cv2.waitKey(1)
        rate.sleep()
    
if __name__ == '__main__':
    main()
