#!/usr/bin/env python

import time
import rospy
import math
import numpy as np
from time import time
from sensor_msgs.msg import Imu, NavSatFix

pubIMU = []
pubGPS = []

def gpsCB(data):
    global pubGPS

    data.header.frame_id = "base_link"

    pubGPS.publish(data)


def imuCB(data):
    global pubIMU
    
    data.header.frame_id = "base_link"

    pubIMU.publish(data)

def listener():

    global pubIMU
    global pubGPS

    rospy.init_node('listener', anonymous=False)

    #* Sub imu/gps information
    rospy.Subscriber("/imu/data", Imu,      imuCB)
    rospy.Subscriber("/gps_pub_2", NavSatFix, gpsCB)

    #* Public imu/gps information
    pubIMU = rospy.Publisher('/imu_correct', Imu,       queue_size=10)
    pubGPS = rospy.Publisher('/gps/fix',     NavSatFix, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
	listener()