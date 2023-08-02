# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""
OpenCV and Numpy Point cloud Software Renderer

This sample is mostly for demonstration and educational purposes.
It really doesn't offer the quality or performance that can be
achieved with hardware acceleration.

Usage:
------
Mouse: 
    Drag with left button to rotate around pivot (thick small axes), 
    with right button to translate and the wheel to zoom.

Keyboard: 
    [p]     Pause
    [r]     Reset View
    [d]     Cycle through decimation values
    [z]     Toggle point scaling
    [c]     Toggle color source
    [s]     Save PNG (./out.png)
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""

import math
import pickle
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import tqdm
import socket
import platform

class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


state = AppState()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()


def mouse_cb(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False

    if event == cv2.EVENT_MOUSEMOVE:

        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2

        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)

        elif state.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz

    state.prev_mouse = (x, y)


cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow(state.WIN_NAME, w, h)
cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation



def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]


out = np.empty((h, w, 3), dtype=np.uint8)



HOST = "10.33.1.1"  # Node1-1 IP
PORT = 65432  # The port used by the server
SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096
streaming = False
downloadfile = False
Node = platform.node().split('-')[1][0]
print("Currently on Node "+Node+".\n") 
def sendFrame(filename):
    filesize = os.path.getsize(filename)
    s.send(f"{filename}{SEPARATOR}{filesize}".encode())
    progress = tqdm.tqdm(range(filesize), f"Sending {filename}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(filename, "rb") as f:
        while True:
            # read the bytes from the file
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                # file transmitting is done
                break
            # we use sendall to assure transimission in busy networks
            s.sendall(bytes_read)
            # update the progress bar
            progress.update(len(bytes_read))
    progress.close()
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    while True:
        # Grab camera data
        if not state.paused:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_frame = decimate.process(depth_frame)

            # Grab new intrinsics (may be changed by decimation)
            depth_intrinsics = rs.video_stream_profile(
                depth_frame.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = np.asanyarray(
                colorizer.colorize(depth_frame).get_data())

            if state.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame, depth_colormap

            points = pc.calculate(depth_frame)
            pc.map_to(mapped_frame)

            # Pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
            pickle.dump(verts, open('verts'+Node+'.pkl','wb'))
            pickle.dump(texcoords, open('texcoords'+Node+'.pkl','wb'))
            pickle.dump(color_source, open('color_source'+Node+'.pkl','wb'))

        # Render
        now = time.time()

        out.fill(0)

        

        if not state.scale or out.shape[:2] == (h, w):
            pointcloud(out, verts, texcoords, color_source)
        else:
            tmp = np.zeros((h, w, 3), dtype=np.uint8)
            pointcloud(tmp, verts, texcoords, color_source)
            tmp = cv2.resize(
                tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
            np.putmask(out, tmp > 0, tmp)

        
        dt = time.time() - now

        cv2.setWindowTitle(
            state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
            (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

        cv2.imshow(state.WIN_NAME, out)
        key = cv2.waitKey(1)

        if key == ord("r"):
            state.reset()

        if key == ord("p"):
            state.paused ^= True

        if key == ord("d"):
            state.decimate = (state.decimate + 1) % 3
            decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)

        if key == ord("z"):
            state.scale ^= True

        if key == ord("c"):
            state.color ^= True

        # if key == ord("f"):
        #     #out, verts, texcoords, color_source
        #     pickle.dump(verts, open('verts'+Node+'.pkl','wb'))
        #     pickle.dump(texcoords, open('texcoords'+Node+'.pkl','wb'))
        #     pickle.dump(color_source, open('color_source'+Node+'.pkl','wb'))
        #     break

        if key == ord("s") or streaming==True:
            if streaming == False:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
                    s.connect((HOST, PORT))
                    streaming = True

                except:
                    print("\n Server not running")
                    continue
                print("Start point cloud video on main node.")
                Node = platform.node().split('-')[1][0]
                print("Currently on Node "+Node+".\n")
            
            if Node == str(1):
                print("Host Node entered, not sending files.")
            else:
                pickle.dump(verts, open('verts'+Node+'.pkl','wb'))
                pickle.dump(texcoords, open('texcoords'+Node+'.pkl','wb'))
                pickle.dump(color_source, open('color_source'+Node+'.pkl','wb'))
                time.sleep(.2)
                try:
                    sendFrame("color_source"+str(Node)+".pkl")
                    time.sleep(.2)               
                except BrokenPipeError as p:
                    print("Pipeline broken")
                    streaming = False
                    s.close()
                
                except Exception as e:
                    print("color_source not sent due to Exception: "+str(e)+".\n")
                try:
                    sendFrame("verts"+str(Node)+".pkl")
                    time.sleep(.2)
                except Exception as e:
                    print("verts not sent due to Exception: "+str(e)+".\n")
                try:
                    sendFrame("texcoords"+str(Node)+".pkl")
                    time.sleep(.2)
                except Exception as e:
                    print("texcoords not sent due to Exception: "+str(e)+".\n")
                    

            

        if key == ord("f") or downloadfile ==True:
            downloadfile = True
            Node = platform.node().split('-')[0][4]
            print("Currently on Node "+Node+".\n") 
            time.sleep(0.1)
            #out, verts, texcoords, color_source
            pickle.dump(verts, open('verts'+Node+'.pkl','wb'))
            pickle.dump(texcoords, open('texcoords'+Node+'.pkl','wb'))
            pickle.dump(color_source, open('color_source'+Node+'.pkl','wb'))
            #break
        if key == ord("e"):
            points.export_to_ply('./out.ply', mapped_frame)

        if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
            break

# Stop streaming
pipeline.stop()
s.close()

