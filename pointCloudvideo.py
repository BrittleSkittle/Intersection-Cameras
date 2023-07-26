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


w, h = 640,480

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

view1 = True
view2 = True
view3 = False
Node1 = input("Please enter first node.")
Node2 = input("Please enter second node.")
try:
    rotation1_2 = pickle.load(open('./rotation'+Node1+'_'+Node2+'.pkl', 'rb'))
    translation1_2 = pickle.load(open('./translation'+Node1+'_'+Node2+'.pkl', 'rb'))
    verts = pickle.load(open('./verts'+Node1+'.pkl', 'rb'))
    
    verts1_2 = verts
    verts1_2 = np.transpose(verts1_2)
    verts1_2 = np.dot(rotation1_2, verts1_2)
    for i in range(3):
        verts1_2[i,:] = verts1_2[i,:]+translation1_2[i]
    verts1_2 = np.transpose(verts1_2)

    texcoords = pickle.load(open('./texcoords'+Node1+'.pkl', 'rb'))

    color_source = pickle.load(open('./color_source'+Node1+'.pkl', 'rb'))

except Exception as e:
    print('Pickle load exception: '+str(e))

try:
    verts2 = pickle.load(open('./verts'+Node2+'.pkl', 'rb'))
    texcoords2 = pickle.load(open('./texcoords'+Node2+'.pkl', 'rb'))
    color_source2 = pickle.load(open('./color_source'+Node2+'.pkl', 'rb'))
except Exception as e:
    print('Pickle load exception: '+str(e))

Node3 = str(3)
try:
    verts3 = pickle.load(open('./verts'+Node3+'.pkl', 'rb'))
    texcoords3 = pickle.load(open('./texcoords'+Node3+'.pkl', 'rb'))
    color_source3 = pickle.load(open('./color_source'+Node3+'.pkl', 'rb'))
except Exception as e:
    print('Pickle load exception: '+str(e))

Node4 = str(4)
try:
    verts4 = pickle.load(open('./verts'+Node4+'.pkl', 'rb'))
    texcoords4 = pickle.load(open('./texcoords'+Node4+'.pkl', 'rb'))
    color_source4 = pickle.load(open('./color_source'+Node4+'.pkl', 'rb'))
except Exception as e:
    print('Pickle load exception: '+str(e))

viewTrans = False
view4 = False
view3 = False
while True:
    # Grab camera data
    
    # Render
    now = time.time()

    out.fill(0)

    try:
        rotation1_2 = pickle.load(open('./rotation'+Node1+'_'+Node2+'.pkl', 'rb'))
        translation1_2 = pickle.load(open('./translation'+Node1+'_'+Node2+'.pkl', 'rb'))
        verts = pickle.load(open('./verts'+Node1+'.pkl', 'rb'))
        
        verts1_2 = verts
        verts1_2 = np.transpose(verts1_2)
        verts1_2 = np.dot(rotation1_2, verts1_2)
        for i in range(3):
            verts1_2[i,:] = verts1_2[i,:]+translation1_2[i]
        verts1_2 = np.transpose(verts1_2)

        texcoords = pickle.load(open('./texcoords'+Node1+'.pkl', 'rb'))

        color_source = pickle.load(open('./color_source'+Node1+'.pkl', 'rb'))

    except Exception as e:
        print('Pickle load exception: '+str(e))

    try:
        verts2 = pickle.load(open('./verts'+Node2+'.pkl', 'rb'))
        texcoords2 = pickle.load(open('./texcoords'+Node2+'.pkl', 'rb'))
        color_source2 = pickle.load(open('./color_source'+Node2+'.pkl', 'rb'))
    except Exception as e:
        print('Pickle load exception: '+str(e))

    if not state.scale or out.shape[:2] == (h, w):
        if(view1):
            pointcloud(out, verts, texcoords, color_source)
        if(view2):
            pointcloud(out,verts2,texcoords2,color_source2)
        if(viewTrans):
            pointcloud(out,verts1_2,texcoords,color_source)
        if(view3):
            pointcloud(out,verts3,texcoords3,color_source3)
        if(view4):
            pointcloud(out,verts4,texcoords4,color_source4)
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
    if key == ord("1"):
        view1 = not view1
    if key == ord("2"):
        view2 = not view2
    if key == ord("t"):
        viewTrans = not viewTrans
    if key == ord("3"):
        view3 = not view3
    if key == ord("4"):
        view4 = not view4



    if key == ord("s"):
        cv2.imwrite('./out.png', out)


    if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
        break

# Stop streaming
