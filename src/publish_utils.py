import cv2
import numpy as np

import tf
import rospy
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


FRAME_ID = "map"

def publish_camera(pub, img, bridge):
    pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

def publish_point_cloud(pub, pcl):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pub.publish(pcl2.create_cloud_xyz32(header, pcl[:, :3]))

def publish_car(pub, path):
    marker_arr = MarkerArray()

    # Fov line
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = FRAME_ID

    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2

    marker.points = []
    marker.points.append(Point(10, -10, 0))
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(10, 10, 0))

    marker_arr.markers.append(marker)

    # Car model
    mesh_marker = Marker()
    mesh_marker.header.stamp = rospy.Time.now()
    mesh_marker.header.frame_id = FRAME_ID

    mesh_marker.id = 1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = path

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0

    mesh_marker.scale.x = 1.5
    mesh_marker.scale.y = 1.5
    mesh_marker.scale.z = 1.2

    marker_arr.markers.append(mesh_marker)
    pub.publish(marker_arr)
