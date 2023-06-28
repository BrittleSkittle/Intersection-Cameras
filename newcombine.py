import cv2 as cv
import numpy as np
import pickle
import calibrate_kabsch as kabsch
import pyrealsense2 as rs
import math
from numpy.linalg import det
import matplotlib.pyplot as plt
import matplotlib
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL) #allows ctrl+c to close figure

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
common_points3 = []
common_points4 = []
common_count = 0

int(Node1) = 1
int(Node2) = 2
int(Node3) = 3
int(Node4) = 4


try:
    for x in range(len(ids_node1)):
        for y in range(len(ids_node2)):
             for i in range(len(ids_node3)):
                for j in range(len(ids_node4)):
                    if ids_node1[x] == ids_node2[y] == ids_node3[i] == ids_node4[j]:
                        common_count = common_count + 1
                for z in range(4):
                    common_points1.append(corners_node1[x][z])
                    common_points2.append(corners_node2[y][z])
                    common_points3.append(corners_node3[i][z])
                    common_points4.append(corners_node4[j][z])
    print(str(common_count) + " common ids found.")
except Exception as e:
    print('Exception in comparison of Nodes '+Node1+' and '+Node2+' and '+Node3+' and '+Node4+': '+str(e))
common_points1arr = np.asarray(common_points1)
common_points2arr = np.asarray(common_points2)
common_points3arr = np.asarray(common_points3)
common_points4arr = np.asarray(common_points4)
rotation1_2, translation1_2, rmsd1_2 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points2arr))
rotation1_3, translation1_3, rmsd1_3 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points3arr))
rotation1_4, translation1_4, rmsd1_4 = kabsch.calculate_transformation_kabsch(np.transpose(common_points1arr),np.transpose(common_points4arr))
print('Node '+Node2+' and '+Node3+' and '+Node4+' to '+Node1+':\n')
print(rotation1_2)
print(rotation1_3)
print(rotation1_4)
print(translation1_2)
print(translation1_3)
print(translation1_4)
yawpitchroll_angles2 = -180*yawpitchrolldecomposition(rotation1_2)/math.pi
yawpitchroll_angles3 = -180*yawpitchrolldecomposition(rotation1_3)/math.pi
yawpitchroll_angles4 = -180*yawpitchrolldecomposition(rotation1_4)/math.pi
print(yawpitchroll_angles2)
print(yawpitchroll_angles3)
print(yawpitchroll_angles4)

command1_2 = 'rosrun tf2_ros static_transform_publisher '+str(translation1_2[0])+' '+str(translation1_2[1])+' '+str(translation1_2[2])\
    +' '+str(yawpitchroll_angles2[0][0])+' '+str(yawpitchroll_angles2[1][0])+' '+str(yawpitchroll_angles2[2][0])+' cam_'+str(Node1)+'_depth_optical_frame cam_'+str(Node2)+'_depth_optical_frame'
print(command1_2)

command1_3 = 'rosrun tf2_ros static_transform_publisher '+str(translation1_3[0])+' '+str(translation1_3[1])+' '+str(translation1_3[2])\
    +' '+str(yawpitchroll_angles3[0][0])+' '+str(yawpitchroll_angles3[1][0])+' '+str(yawpitchroll_angles3[2][0])+' cam_'+str(Node1)+'_depth_optical_frame cam_'+str(Node3)+'_depth_optical_frame'
print(command1_3)

command1_4 = 'rosrun tf2_ros static_transform_publisher '+str(translation1_4[0])+' '+str(translation1_4[1])+' '+str(translation1_4[2])\
    +' '+str(yawpitchroll_angles4[0][0])+' '+str(yawpitchroll_angles4[1][0])+' '+str(yawpitchroll_angles4[2][0])+' cam_'+str(Node1)+'_depth_optical_frame cam_'+str(Node4)+'_depth_optical_frame'
print(command1_4)


#Checking calibration error
common_points1arr = np.transpose(common_points1arr)
common_points2arr = np.transpose(common_points2arr)
common_points3arr = np.transpose(common_points3arr)
common_points4arr = np.transpose(common_points4arr)

checkrot = np.dot(rotation1_2, common_points1arr)
for i in range(3):
    checkrot[i,:] = checkrot[i,:]+translation1_2[i]
print(str(checkrot-common_points2arr))#numerical error

checkrot = np.transpose(checkrot)
Ys = checkrot[:,0]
Zs = checkrot[:,1]
Xs = checkrot[:,2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(Xs, Ys, Zs, c='purple', marker='o')#plots rotated and translated points
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')

checkrot3 = np.dot(rotation1_3, common_points1arr)
for i in range(3):
    checkrot3[i,:] = checkrot3[i,:]+translation1_3[i]
print(str(checkrot3-common_points3arr))#numerical error

checkrot3 = np.transpose(checkrot3)
Ys3 = checkrot3[:,0]
Zs3 = checkrot3[:,1]
Xs3 = checkrot3[:,2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(Xs3, Ys3, Zs3, c='purple', marker='o')#plots rotated and translated points
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')

checkrot4 = np.dot(rotation1_4, common_points1arr)
for i in range(3):
    checkrot4[i,:] = checkrot4[i,:]+translation1_4[i]
print(str(checkrot4-common_points4arr))#numerical error

checkrot4 = np.transpose(checkrot4)
Ys4 = checkrot4[:,0]
Zs4 = checkrot4[:,1]
Xs4 = checkrot4[:,2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(Xs4, Ys4, Zs4, c='purple', marker='o')#plots rotated and translated points
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')

common_points1arr = np.transpose(common_points1arr)#plots points from first node
Ys = common_points1arr[:,0]
Zs = common_points1arr[:,1]
Xs = common_points1arr[:,2]

ax.scatter(Xs, Ys, Zs, c='blue', marker='o')
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')

common_points2arr = np.transpose(common_points2arr)#plots points from second node
Ys = common_points2arr[:,0]
Zs = common_points2arr[:,1]
Xs = common_points2arr[:,2]

ax.scatter(Xs, Ys, Zs, c='r', marker='o')
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')
plt.show()

common_points3arr = np.transpose(common_points3arr)#plots points from third node
Ys = common_points3arr[:,0]
Zs = common_points3arr[:,1]
Xs = common_points3arr[:,2]

ax.scatter(Xs, Ys, Zs, c='green', marker='o')
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')
plt.show()

common_points4arr = np.transpose(common_points4arr)#plots points from fourth node
Ys = common_points4arr[:,0]
Zs = common_points4arr[:,1]
Xs = common_points4arr[:,2]

ax.scatter(Xs, Ys, Zs, c='yellow', marker='o')
ax.set_xlabel('Y')
ax.set_ylabel('Z')
ax.set_zlabel('X')
plt.show()
