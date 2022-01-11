"""
Calculating the similarity between two skeletons.
"""

import numpy as np

#           0        1     2    3       4        5      6     7      8        9     10      11       12       13
adj_joint=((1,8), (0,1), (5,1), (2,1), (6,5), (3,2), (7,6), (4,3), (12,8), (9,8), (13,12), (10,9), (14,13), (11,10))
adj_link=((1,0), (2,1), (3,1), (2,0), (3,0), (2,3), (4,2), (5,3), (6,4), (7,5), (8,0), (9,0), (10, 8), (11,9), (12,10), (13,11))

def mag(a, b):
    return np.linalg.norm(a-b)

def normalize(vec):
    return vec/np.linalg.norm(vec)

def valid(a):
    return (not a[0]==0) or (not a[1]==0.0)

def limbvec(a,b):

    if valid(a) and valid(b):
        return a-b
    else:
        return np.zeros(2)

def against(a,b):
    a=normalize(a)
    b=normalize(b)
    return np.asarray((np.cross(a,b), np.dot(a,b)))

def jointAngles(sk):
    global adj_joint, adj_link
    sk=sk[:,0:2]
    limbs=[limbvec(sk[a],sk[b]) for a,b in adj_joint]
    relativelimbs=[against(limbs[a],limbs[b]) for a,b in adj_link]
    return relativelimbs

def jointAngles_ros(sk):
    global adj_joint, adj_link
    limbs=[limbvec(sk[a],sk[b]) for a,b in adj_joint]
    relativelimbs=[against(limbs[a],limbs[b]) for a,b in adj_link]
    return relativelimbs

def toNumpyArray(sk):
    sk = [(j.pixel.x,j.pixel.y) for j in sk]
    return np.asarray(sk)

def similarity(a,b):
    sum = 0.0
    count = 0
    min=1.0
    for i in range(len(a)):
        sim = np.dot(a[i], b[i])
        if sim is not None and abs(sim) > 0.0:
            sum += sim
            min = sim if sim<min else min
            count += 1 
            
    if count>0:
        return sum/count, min, count
    else:
        return 0.0, 0.0, 0.0
