#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("filteredscan", LaserScan, queue_size=10)

    def callback(self, data):

        newdata = data
        # Los datos de distancia e intensidad leídos del mensaje son tuplas, que deben convertirse a lista para su funcionamiento
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        #Mantenga datos válidos borrando los datos de sectores innecesarios
        for x in range(120,240):
            newdata.ranges[x]=0
            newdata.intensities[x]=0

        # Sector 180 ° delantero
        #for x in range(90,270):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        # 60 ° sector recto
        #for x in range(30,330):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        self.pub.publish(newdata)
        rospy.loginfo(data)


if __name__ == '__main__':

    # Initialize
    rospy.init_node('LidarFilter', anonymous=False)
    lidar = DoFilter()

    rospy.spin()

        
        
        
        
        
           
        
           
 



   
    