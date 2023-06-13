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
    ids1 = pickle.load(open('./ids1.pkl', 'rb'))
    corners1 = pickle.load(open('./corners3D1.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 1: '+str(e))

try:
    ids2 = pickle.load(open('./ids2.pkl', 'rb'))
    corners2 = pickle.load(open('./corners3D2.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 2: '+str(e))

try:
    ids3 = pickle.load(open('./ids3.pkl', 'rb'))
    corners3 = pickle.load(open('./corners3D3.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 3: '+str(e))

try:
    ids4 = pickle.load(open('./ids4.pkl', 'rb'))
    corners4 = pickle.load(open('./corners3D4.pkl', 'rb'))
except Exception as e:
    print('Exception for Node 4: '+str(e))
    
common_points1_2 = []
common_points1_3 = []
common_points1_4 = []
common_points2 = []
common_points3 = []
common_points4 = []
common_count = 0
try:
    for x in range(len(ids1)):
        for y in range(len(ids4)):
            if ids1[x] == ids4[y]:
                common_count = common_count + 1
                for z in range(4):
                    common_points1_4.append(corners1[x][z])
                    common_points4.append(corners4[y][z])
    print(str(common_count) + " common ids found.")
except Exception as e:
    print('Exception in comparison of Nodes 1 and 4: '+str(e))
common_points1arr = np.asarray(common_points1_4)
common_points4arr = np.asarray(common_points4)
#print(common_points1arr)
rotation1_4, translation1_4, rmsd1_4 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points4arr))
print('Node 4 to 1:\n')
print(rotation1_4)
print(translation1_4)
yawpitchroll_angles = -180*yawpitchrolldecomposition(rotation1_4)/math.pi
yawpitchroll_angles[0,0] = (360-yawpitchroll_angles[0,0])%360 # change rotation sense if needed, comment this line otherwise
yawpitchroll_angles[1,0] = yawpitchroll_angles[1,0]+90
print(yawpitchroll_angles)
print('')

common_count = 0
try:
    for x in range(len(ids1)):
        for y in range(len(ids2)):
            if ids1[x] == ids2[y]:
                common_count = common_count + 1
                for z in range(4):
                    common_points1_2.append(corners1[x][z])
                    common_points2.append(corners2[y][z])
    print(str(common_count) + " common ids found.")
except Exception as e:
    print('Exception in comparison of Nodes 1 and 2: '+str(e))
common_points1arr = np.asarray(common_points1_2)
common_points2arr = np.asarray(common_points2)
#print(common_points1)
rotation1_2, translation1_2, rmsd1_2 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points2arr))
print('Node 2 to 1:\n')
print(rotation1_2)
print(translation1_2)
yawpitchroll_angles = -180*yawpitchrolldecomposition(rotation1_2)/math.pi
yawpitchroll_angles[0,0] = (360-yawpitchroll_angles[0,0])%360 # change rotation sense if needed, comment this line otherwise
yawpitchroll_angles[1,0] = yawpitchroll_angles[1,0]+90
print(yawpitchroll_angles)
print('')

common_count = 0
try:
    for x in range(len(ids1)):
        for y in range(len(ids3)):
            if ids1[x] == ids3[y]:
                common_count = common_count + 1
                for z in range(4):
                    common_points1_3.append(corners1[x][z])
                    common_points3.append(corners3[y][z])
    print(str(common_count) + " common ids found.")
except Exception as e:
    print('Exception in comparison of Nodes 1 and 3: '+str(e))
common_points1arr = np.asarray(common_points1_3)
common_points3arr = np.asarray(common_points3)
#print(common_points1)
rotation1_3, translation1_3, rmsd1_3 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points3arr))
print('Node 3 to 1:\n')
print(rotation1_3)
print(translation1_3)
yawpitchroll_angles = -180*yawpitchrolldecomposition(rotation1_3)/math.pi
yawpitchroll_angles[0,0] = (360-yawpitchroll_angles[0,0])%360 # change rotation sense if needed, comment this line otherwise
yawpitchroll_angles[1,0] = yawpitchroll_angles[1,0]+90
print(yawpitchroll_angles)

