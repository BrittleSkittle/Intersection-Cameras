# Intersection Camera
The goal of this project is to combine the 3D point clouds of 4 different realsense depth cameras into a single calibrated view. Currently pointcloud display only works with 2 cameras. 

# Prerequisites
- ros (tested with melodic)
- librealsense2 (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
- realsense_ros (https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
- opencv for python (sudo apt-get install python3-opencv)
- pip
    - numpy
    - pickle
    - tqdm

# Instructions
1. Run `python3 corners.py` on current node. Enter the amount of ids you want to detect, this can range from 9 to 25. Try multiple `/dev/video` sources until it works. You may need to run as sudo. 
2. If on main node skip this step. Otherwise run `python3 server.py` on the main node and `python3 client.py` on the current node, sending the appropriate ids pickle file.
3. Run `python3 2Dto3D.py` on the current node to generate the 3D points from the 2D points. 
4. Repeat step 2, except send the generated corners3D file instead of the ids file. 
5. Run `combine.py` on the main node, which will return the transformation matrices, the rotation matrices, and the "roll" "pitch" "yaw" matrices for each node relative to the main node. 
6. Run `roscore` on main node. 
7. Run `export ROS_MASTER_URI=http://node1-1.intersection.orbit-lab.org:11311/ ` on each node using the main node for the link. 
8. Run the following commands on each node respectively:

`roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=821312060217 filters:=pointcloud`

`roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=751412060500 filters:=pointcloud`

`roslaunch realsense2_camera rs_camera.launch camera:=cam_3 serial_no:=751412060120 filters:=pointcloud`

`roslaunch realsense2_camera rs_camera.launch camera:=cam_4 serial_no:=821312060260 filters:=pointcloud`

9. run `rosrun tf2_ros static_transform_publisher 0.75464242 -1.047124 1.22725315 53.92585782 11.09066286 127.50423895 cam_1_depth_optical_frame cam_2_depth_optical_frame` on main node using the returned `x y z yaw pitch roll` for each camera transformation. 
10. run `rviz` and add each pointcloud by selecting add -> by topic -> cam_x/depth/color/points/pointcloud2. Change flat squares to points under each pointcloud -> style for better fps. 
11. Change the fixed frame from "map" to "cam_2_depth_optical_frame"
