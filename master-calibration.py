import platform
Node = platform.node().split('-')[0][4]
print("Currently on Node "+Node+".\n")

import corners
import TwoDto3D
import opencv_pointcloud_viewer
import server
import client
import combine
import static_pointcloud_viewer

