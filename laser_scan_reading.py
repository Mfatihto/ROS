#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 13 09:56:34 2022

@author: quietbc
"""
import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    regions = [
        min(min(msg.ranges[0:143]), 10),
        min(min(msg.ranges[144:287]), 10),
        min(min(msg.ranges[288:431]), 10),
        min(min(msg.ranges[432:575]), 10),
        min(min(msg.ranges[576:713]), 10),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
