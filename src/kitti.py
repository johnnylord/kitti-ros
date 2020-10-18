#!/usr/bin/python3
import os
import os.path as osp

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from data_utils import *
from publish_utils import *
from kitti_utils import *

BASE_DIR = '/home/johnnylord/Simulate/catkin_ws/src/kitti/'
CAM_DIR = osp.join(BASE_DIR, 'data')
IMG_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/image_02/data')
PCL_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/velodyne_points/data')
IMU_DIR = osp.join(BASE_DIR, 'data/2011_09_26_drive_0005_sync/oxts/data')
TRACKING_FILE = osp.join(BASE_DIR, 'data/tracking.txt')


def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    R = np.array([
        [np.cos(yaw), 0, np.sin(yaw)],
        [0, 1, 0],
        [-np.sin(yaw), 0, np.cos(yaw)]
    ])
    x_corners = [l/2, l/2, -l/2, -l/2,  l/2,  l/2, -l/2, -l/2]
    y_corners = [0,   0,    0,    0,   -h,   -h,   -h,   -h  ]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2  ]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d_cam2 += np.vstack([x, y, z])
    return corners_3d_cam2

if __name__ == "__main__":
    # Graph initialization
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
    car_pub = rospy.Publisher('kitti_car', MarkerArray, queue_size=10)
    box_pub = rospy.Publisher('kitti_box', MarkerArray, queue_size=10)

    # Data loop
    fid = 0
    fcount = len(os.listdir(IMG_DIR))
    df_tracking = read_tracking(TRACKING_FILE)
    bridge = CvBridge()
    calib = Calibration(CAM_DIR, from_video=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tracking_frame = df_tracking[df_tracking.frame==fid]

        # Read in raw data in numpy type
        types = np.array(tracking_frame["type" ])
        bboxes_2d = np.array(tracking_frame[["xmin", "ymin", "xmax", "ymax" ]])
        bboxes_3d = np.array(tracking_frame[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rotate_y']])
        img = read_camera(osp.join(IMG_DIR, '%010d.png'%fid), bboxes_2d, types)
        pcl = read_point_cloud(osp.join(PCL_DIR, '%010d.bin'%fid))

        corners_3d_velos = []
        for bbox_3d in bboxes_3d:
            corner_3d_cam2 = compute_3d_box_cam2(*bbox_3d)
            corner_3d_velo = calib.project_rect_to_velo(corner_3d_cam2.T)
            corners_3d_velos.append(corner_3d_velo)

        # Publish data
        publish_camera(cam_pub, img, bridge)
        publish_point_cloud(pcl_pub, pcl)
        publish_3dbox(box_pub, corners_3d_velos, types)
        publish_car(car_pub, 'package://kitti/model/Lowpoly-Car-000.dae')

        # Update loop
        fid += 1
        fid %= fcount
        rate.sleep()
        rospy.loginfo("camera image published")
