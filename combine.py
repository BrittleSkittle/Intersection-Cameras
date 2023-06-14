import cv2 as cv
import numpy as np
import pickle
import calibrate_kabsch as kabsch
import pyrealsense2 as rs
import math
from numpy.linalg import det

def yawpitchrolldecomposition(R):
    sin_x    = math.sqrt(R[2,0] * R[2,0] +  R[2,1] * R[2,1])    
    validity  = sin_x < 1e-6
    if not det(R)==0:
        z1    = math.atan2(R[2,0], R[2,1])     # around z1-axis
        x      = math.atan2(sin_x,  R[2,2])     # around x-axis
        z2    = math.atan2(R[0,2], -R[1,2])    # around z2-axis
    else: # gimbal lock
        z1    = 0                                         # around z1-axis
        x      = math.atan2(sin_x,  R[2,2])     # around x-axis
        z2    = 0                                         # around z2-axis

    return np.array([[z1], [x], [z2]])



try:
    ids_node1 = pickle.load(open('./ids1.pkl', 'rb'))
    corners_node1 = pickle.load(open('./corners3D1.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 1: '+str(e))

try:
    ids_node2 = pickle.load(open('./ids2.pkl', 'rb'))
    corners_node2 = pickle.load(open('./corners3D2.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 2: '+str(e))

try:
    ids_node3 = pickle.load(open('./ids3.pkl', 'rb'))
    corners_node3 = pickle.load(open('./corners3D3.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 3: '+str(e))

try:
    ids_node4 = pickle.load(open('./ids4.pkl', 'rb'))
    corners_node4 = pickle.load(open('./corners3D4.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 4: '+str(e))
    

common_points1 = []
common_points2 = []
common_count = 0

Node1 = input("Input first Node\n")
Node2 = input("Input second Node\n")

if int(Node1) == 1:
    ids1 = ids_node1
    corners1 = corners_node1
elif int(Node1) == 2:
    ids1 = ids_node2
    corners1 = corners_node2
elif int(Node1) == 3:
    ids1 = ids_node3
    corners1 = corners_node3
elif int(Node1) == 4:
    ids1 = ids_node4
    corners1 = corners_node4

if int(Node2) == 1:
    ids2 = ids_node1
    corners2 = corners_node1
elif int(Node2) == 2:
    ids2 = ids_node2
    corners2 = corners_node2
elif int(Node2) == 3:
    ids2 = ids_node3
    corners2 = corners_node3
elif int(Node2) == 4:
    ids2 = ids_node4
    corners2 = corners_node4

try:
    for x in range(len(ids1)):
        for y in range(len(ids2)):
            if ids1[x] == ids2[y]:
                common_count = common_count + 1
                for z in range(4):
                    common_points1.append(corners1[x][z])
                    common_points2.append(corners2[y][z])
    print(str(common_count) + " common ids found.")
except Exception as e:
    print('Exception in comparison of Nodes '+Node1+' and '+Node2+': '+str(e))
common_points1arr = np.asarray(common_points1)
common_points4arr = np.asarray(common_points2)
#print(common_points1arr)
rotation1_2, translation1_2, rmsd1_2 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points4arr))
print('Node '+Node2+' to '+Node1+':\n')
print(rotation1_2)
print(translation1_2)
yawpitchroll_angles = -180*yawpitchrolldecomposition(rotation1_2)/math.pi
#yawpitchroll_angles[0,0] = (360-yawpitchroll_angles[0,0])%360 # change rotation sense if needed, comment this line otherwise
#yawpitchroll_angles[1,0] = yawpitchroll_angles[1,0]+90
print(yawpitchroll_angles)
print('')


