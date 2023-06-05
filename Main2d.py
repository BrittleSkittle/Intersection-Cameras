import pyrealsense2 as rs
import numpy as np
import cv2 as cv
def pixel_to_3d(pixel, depth_value, intrinsics):
    x = (float(pixel[0]) -intrinsics.ppx) / intrinsics.fx
    y = (float(pixel[1]) -intrinsics.ppy) / intrinsics.fy
    point_3d = np.array([x, y, 1]) * depth_value
    return point_3d
x = input('x = ')
y = input('y = ')
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth)
pipeline.start(config)
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
frame = pipeline.wait_for_frames()
depth_frame = frame.get_depth_frame()
depth_value = depth_frame.get_distance(int(x), int(y))
print("width :", intrinsics.width)
print("height :", intrinsics.height)
print("focale x :", intrinsics.fx)
print("focale y :", intrinsics.fy)
print("centre_optique x :", intrinsics.ppx)
print("centre_optique y :", intrinsics.ppy)
pipeline.stop()
pixel = (x, y)
depth = depth_value
point_3d = pixel_to_3d(pixel, depth, intrinsics)
result = np.round(point_3d, 3)
print("le point 3D :", point_3d)
print("le point 3D :", result)
