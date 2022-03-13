#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 13 10:04:56 2022

@author: quietbc
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstAvoid():
    def __init__(self):
        rospy.init_node("obst_node")
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.msg_twist = Twist()
        rospy.Subscriber("scan",LaserScan,self.laserCallBack)
        rospy.spin()
        
    def takeAction(self,regions):
        self.msg_twist = Twist()
        self.msg_twist.linear.x = 0.0
        self.msg_twist.angular.z = 0.0
        
        self.state_name = ''
        
        if self.regions['on'] > 1 and self.regions['sol'] > 1 and self.regions['sag'] > 1:
            self.state_name = 'case 1 - nothing'
            self.msg_twist.linear.x = 0
            self.msg_twist.angular.z = 0
            self.pub.publish(self.msg_twist)
        elif self.regions['on'] > 1 and self.regions['sol'] > 1 and self.regions['sag'] > 1:
            self.state_name = 'case 2 - front'
            self.msg_twist.linear.x = 0.5
            self.msg_twist.angular.z = 0
            self.pub.publish(self.msg_twist)
        elif self.regions['on'] > 1 and self.regions['sol'] > 1 and self.regions['sag'] > 1:
            self.state_name = 'case 3 - right'
            self.msg_twist.linear.x = 0
            self.msg_twist.angular.z = 0.2
            self.pub.publish(self.msg_twist)
        elif self.regions['on'] > 1 and self.regions['sol'] > 1 and self.regions['sag'] > 1:
            self.state_name = 'case 4 - left'
            self.msg_twist.linear.x = 0
            self.msg_twist.angular.z = -0.2
            self.pub.publish(self.msg_twist)
        elif self.regions['on'] > 1 and self.regions['sol'] > 1 and self.regions['sag'] > 1:
            self.state_name = 'case 5 - back'
            self.msg_twist.linear.x = -0.5
            self.msg_twist.angular.z = 0
            self.pub.publish(self.msg_twist)
        else:
            self.state_name = 'Unknown State'
            rospy.loginfo(self.state_name)
            
    def laserCallBack(self,msg):
        self.regions = {
                'on': min(list(msg.ranges[0:9])+list(msg.ranges[350:359])),
                'sol': min(list(msg.ranges[80:100])),
                'sag': min(list(msg.ranges[260:280])),
                'arka': min(list(msg.ranges[170:190]))
                }
        takeAction(self.regions)
        
a = obstAvoid()
