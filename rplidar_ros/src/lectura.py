#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    
    #float32 angle_min
    #float32 angle_max
    #float32 angle_increment
    #float32 time_increment
    #float32 scan_time
    #float32 range_min
    #float32 range_max
    #float32[] ranges
    #float32[] intensities

    rospy.loginfo(scan.header)

def listener():
    rospy.init_node('laser_listener', anonymous=False)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()