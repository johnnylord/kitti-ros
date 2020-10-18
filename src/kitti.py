#!/usr/bin/python3
import os
import os.path as osp

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from data_utils import *
from publish_utils import *

BASE_DIR = '/home/johnnylord/Simulate/catkin_ws/src/kitti/'
IMG_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/image_02/data')
PCL_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/velodyne_points/data')
IMU_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/oxts/data')


if __name__ == "__main__":
    # Graph initialization
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
    car_pub = rospy.Publisher('kitti_car', MarkerArray, queue_size=10)

    # Data loop
    fid = 0
    fcount = len(os.listdir(IMG_DIR))
    bridge = CvBridge()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Read in raw data in numpy type
        img = read_camera(osp.join(IMG_DIR, '%010d.png'%fid))
        pcl = read_point_cloud(osp.join(PCL_DIR, '%010d.bin'%fid))

        # Publish data
        publish_camera(cam_pub, img, bridge)
        publish_point_cloud(pcl_pub, pcl)
        publish_car(car_pub, 'package://kitti/model/Lowpoly-Car-000.dae')

        # Update loop
        fid += 1
        fid %= fcount
        rate.sleep()
        rospy.loginfo("camera image published")
