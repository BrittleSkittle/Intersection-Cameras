# Intersection Camera
The goal of this project is to combine the 3D point clouds of 4 different realsense depth cameras into a single calibrated view. 

# Prerequisites
- ros (tested with melodic)
- librealsense2 (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
- realsense_ros (https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
- opencv for python (sudo apt-get install python3-opencv)
- pip(pip3)
    - numpy
    - pickle
    - tqdm

# Instructions
1. Run `python3 corners.py` on current node. Enter the amount of ids you want to detect, this can range from 9 to 40 depending on the setup. Try multiple `/dev/video` sources until it works. You may need to run as sudo. 
2. Run `python3 2Dto3D.py` on the current node to generate the 3D points from the 2D points. 
3. Run `python3 server.py` on the main node and `python3 client.py` on the other nodes, sending the appropriate pickle files "idsX.pkl" and/or "corners3DX.pkl"to the main node.
4. Run `python3 combine.py` on the main node, which will return the transformation matrices, the rotation matrices, and the "roll" "pitch" "yaw" matrices for each node relative to the main node. 
5. Run `python3 opencv_pointcloud_viewer` on each node and press 'd' until the resolution shown at the top of the window is as desired. Then press 'f' to download the pointcloud data and enter the current node. _This may need to be done twice to ensure data capture._
6. Run `python3 server.py` again on the main node and `python3 client.py` on the other nodes and send "color_sourceX.pkl", "texcoordsX.pkl", and "vertsX.pkl" to the main node. 
7. `Run static_pointcloud_viewer` on the main node. Press the number keys to toggle viewing of each respective node. Press 't' to toggle the transform from the selected nodes.  


- The steps below send low quality depth data to a node. We are unsure how to access and manipulate this data.
 
6. ~~Run `python3 EtherSenseClient.py` on each node besides 1.~~
7. ~~Run `python3 EtherSenseServer.py` on node 1.~~ 
8. ~~We are not yet sure how to access the streamed data.~~ 


- The steps below were found to be inconsistent

6. ~~Run `roscore` on main node.~~
7. ~~Run `export ROS_MASTER_URI=http://node1-1.intersection.orbit-lab.org:11311/ ` on each node using the main node for the link.~~
8. ~~Run the following commands on each node respectively:~~

- ~~`roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=821312060217 filters:=pointcloud`~~

- ~~`roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=751412060500 filters:=pointcloud`~~

- ~~`roslaunch realsense2_camera rs_camera.launch camera:=cam_3 serial_no:=751412060120 filters:=pointcloud`~~

- ~~`roslaunch realsense2_camera rs_camera.launch camera:=cam_4 serial_no:=821312060260 filters:=pointcloud`~~

9. ~~run `rosrun tf2_ros static_transform_publisher 0.75464242 -1.047124 1.22725315 53.92585782 11.09066286 127.50423895 cam_1_depth_optical_frame cam_2_depth_optical_frame` on main node using the returned `x y z yaw pitch roll` for each camera transformation.~~ 
10. ~~run `rviz` and add each pointcloud by selecting add -> by topic -> cam_x/depth/color/points/pointcloud2. Change flat squares to points under each pointcloud -> style for better fps.~~ 
11. ~~Change the fixed frame from "map" to "cam_2_depth_optical_frame"~~


# Troubleshooting
- If camera is not loading or is very slow try `sudo apt-get remove librealsense2-dkms`
    - Next try running `realsense-viewer` and turning the streams on and off again. If that doesn't work click on the 3 Horizontal lines and select "hardware reset"
- Some scripts may need to be run as sudo
- To test corners.py script set ids to be detected to 40 or greater
- If image appears spotty or unclear run `realsense-viewer` and click "hardware reset"

# Desired Improvements
- Improvement of corners.py. It currently only records markers captured in single frame, while each fram detects different ids. It would be optimal to record all markers detected over a period of time. 
- Automation of client.py. It currently uses manual user input to determine which file to send. It would be better to automatically detect relevant files and send them. 
- Development of a "master script" that runs all of the required steps on one of the nodes, detecting whether or not the server is running on another node. 
