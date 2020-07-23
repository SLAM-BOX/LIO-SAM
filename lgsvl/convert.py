#!env python
# -*- coding: utf-8 -*-
import time
import rospy
import math
import numpy as np
from time import time
from sensor_msgs.msg import Imu, NavSatFix, PointField, PointCloud2
import sensor_msgs.point_cloud2 as pcl2

pubIMU   = []
pubLiDAR = []
pubGPS   = []

imu_count = 0

def gpsCB(data):
    global pubGPS
    data.header.frame_id = "base_link"
    pubGPS.publish(data)

def lidarCB(data):
    global pubLiDAR

    pc = pcl2.read_points(data, skip_nans=True)
    scan = []
    for p in pc:
        scan.append([p[0],p[1],p[2],p[3]])
    scan = np.array(scan)

    # get ring channel
    depth = np.linalg.norm(scan, 2, axis=1)
    pitch = np.arcsin(scan[:, 2] / depth) # arcsin(z, depth)
    # fov_down = -24.8 / 180.0 * np.pi
    # fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
    fov_down = -10.0 / 180.0 * np.pi
    fov = (abs(-10.0) + abs(33.0)) / 180.0 * np.pi
    proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
    proj_y *= 64  # in [0.0, H]
    proj_y = np.floor(proj_y)
    proj_y = np.minimum(64 - 1, proj_y)
    proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
    proj_y = proj_y.reshape(-1, 1)
    scan = np.concatenate((scan, proj_y), axis=1)

    # fill pcl msg
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1),
              PointField('ring', 16, PointField.UINT16, 1)]
    pcl_msg = pcl2.create_cloud(data.header, fields, scan)
    pcl_msg.is_dense = True

    pubLiDAR.publish(pcl_msg)

def imuCB(data):
    global pubIMU
    data.header.frame_id = "base_link"
    pubIMU.publish(data)

def listener():

    global pubIMU
    global pubGPS
    global pubLiDAR

    rospy.init_node('listener', anonymous=False)

    #* Sub imu/gps information
    rospy.Subscriber("/imu/data",           Imu,        imuCB)
    rospy.Subscriber("/points_raw",    PointCloud2,       lidarCB)
    # rospy.Subscriber("/gps_pub_2",          NavSatFix,  gpsCB)

    #* Public imu/gps information
    pubIMU      = rospy.Publisher('/imu_correct', Imu,       queue_size=10)
    pubLiDAR    = rospy.Publisher('/velodyne_points',  PointCloud2,      queue_size=10)
    pubGPS      = rospy.Publisher('/gps/fix',     NavSatFix, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
	listener()
