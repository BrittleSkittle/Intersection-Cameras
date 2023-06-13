import pyrealsense2 as rs
import numpy as np
import pickle
import cv2 as cv

print('Type the number of the node you are currently on.\n')
Node = input("")

try:
    ids = pickle.load(open('./ids'+Node+'.pkl', 'rb'))
    corners = pickle.load(open('./corners'+Node+'.pkl', 'rb'))
except Exception as e:
    print('Pickle load exception: '+str(e))

def pixel_to_3d(pixel, depth_value, intrinsics):
    x = (float(pixel[0]) -intrinsics.ppx) / intrinsics.fx
    y = (float(pixel[1]) -intrinsics.ppy) / intrinsics.fy
    point_3d = np.array([x, y, 1]) * depth_value
    return point_3d
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth)
pipeline.start(config)
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
frame = pipeline.wait_for_frames()
depth_frame = frame.get_depth_frame()
#x = input('x = ')
#y = input('y = ')
corners3D = []
for id in corners:
    corners4 = []
    for corner in id[0]:
        x = corner[0]
        y = corner[1]
        depth = depth_frame.get_distance(int(x), int(y))
    # print("width :", intrinsics.width)
    # print("height :", intrinsics.height)
    # print("focale x :", intrinsics.fx)
    # print("focale y :", intrinsics.fy)
    # print("centre_optique x :", intrinsics.ppx)
    # print("centre_optique y :", intrinsics.ppy)
        pixel = (x, y)
        point_3d = pixel_to_3d(pixel, depth, intrinsics)
        #print("3D point :", point_3d)
        corners4.append(point_3d)
    corners3D.append(corners4)
        

pipeline.stop()
pickle.dump(corners3D, open('corners3D'+Node+'.pkl','wb'))
if(Node!=1):
    print('Points from node '+Node+' successfully converted to 3D. \n Now run server.py on Node 1 and client.py on this Node and input corners3d'+Node+'.pkl to transfer to Node 1.')
#result = np.round(point_3d, 3)
