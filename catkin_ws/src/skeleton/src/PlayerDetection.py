#!/usr/bin/env python
import numpy as np
import rospy
from sklearn.cluster import DBSCAN

# msg
from skeleton.msg import Players,Player
from visualization_msgs.msg import Marker,MarkerArray
from temi_driver.msg import TemiCMD
from game.msg import GameStatus
game_status = GameStatus()

# cvbridge
from cv_bridge import CvBridge
bridge = CvBridge()

# stack storing player data
player_stack = Players()

def player_cb(msg):
    global fakemap, game_status, start_time, player_stack, gamestatud_pub

    if not game_status.status=='DETECT_STARTED':
        return

    for player in msg.players:
        player_stack.players.append(player)

        if player.id >-1 and player.id<game_status.player_num:
            print('Player %d with score %f at (%f, %f).'%(player.id, player.score, player.position.position.x, player.position.position.y))

    if (rospy.Time.now() - start_time).to_sec()>10 and len(player_stack.players)>game_status.player_num*25:
        playerpub.publish(ClusterPlayers())

        player_stack = Players()

        game_status.last_status = game_status.status
        game_status.status = 'CHECKOUT_STARTED'
        gamestatud_pub.publish(game_status)

    markers = MarkerArray()
    for p in msg.players:
        playername = 'unknown' if p.id<=-1 else game_status.names[p.id]
        markers.markers.append(MakeMarker(markers, p.position.position.x, p.position.position.y, frame='map', type=Marker.SPHERE, scale=0.05))
        markers.markers.append(MakeMarker(markers, p.position.position.x, p.position.position.y+0.3, frame='map', type=Marker.TEXT_VIEW_FACING, scale=0.2, text=playername))
    markerpub.publish(markers)

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
    global PlayerNum, fakemap, markerpub, namepub

    pos = CollectPositions(player_stack)
    cluster = DBSCAN(eps=0.6, min_samples=10).fit(pos)

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
        if cluster.labels_[i]>-1:
            clustered_players[cluster.labels_[i]].append(player_stack.players[i])

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

        player.id = np.argmax(vote, axis=0) if np.max(vote)>0.0 else -1
        print('Player {0} with confidence array:{1}'.format(player.id, vote))
        
        sum = np.sum(vote)
        player.confidence_array = vote/sum if sum>0.0 else vote

        players.players.append(player)

    print('Totally %d player were detected!'%(len(players.players)))

    markers = MarkerArray()
    for p in players.players:
        playername = 'unknown' if p.id<=-1 else game_status.names[p.id]
        markers.markers.append(MakeMarker(markers, p.position.position.x, p.position.position.y, frame='map', type=Marker.SPHERE, scale=0.1))
        markers.markers.append(MakeMarker(markers, p.position.position.x, p.position.position.y+0.3, frame='map', type=Marker.TEXT_VIEW_FACING, scale=0.5, text=playername))
    markerpub.publish(markers)

    return players
    
def gamestatus_cb(msg):
    global game_status, start_time, cmd_pub, fakemap
    game_status = msg
    if game_status.status == 'DETECT_STARTED' and (not game_status.last_status == 'DETECT_STARTED'):
        fakemap = np.zeros((600,600,3), dtype=np.uint8)
        start_time = rospy.Time.now()
        cmd_pub.publish(TemiCMD('cmd','scan'))

def main():
    global id, goalpub, tfbuf, player_stack, fakemap, playerpub, gamestatud_pub, start_time, cmd_pub, markerpub, namepub

    rospy.init_node('Player_Detection', anonymous=False)

    start_time = rospy.Time.now()

    rospy.Subscriber('/game_status', GameStatus, callback=gamestatus_cb, queue_size=10)
    rospy.Subscriber('/players/withscore', Players,  callback=player_cb, queue_size=10)
    playerpub = rospy.Publisher('/players/final', Players, queue_size=10)
    markerpub = rospy.Publisher('/players', MarkerArray, queue_size=10)
    gamestatud_pub = rospy.Publisher('/game_status', GameStatus, queue_size=10)
    cmd_pub = rospy.Publisher('temi_cmd', TemiCMD, queue_size=10)
    rospy.spin()
    
if __name__ == '__main__':
    main()
