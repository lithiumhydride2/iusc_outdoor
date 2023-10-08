#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

def node_simulator1():
    rospy.init_node('node_simulator1', anonymous=True)
    pub = rospy.Publisher("Task1Done", String, queue_size=10)
    time.sleep(2)
    pub.publish("Task1 Done")

if __name__ == '__main__':
    try:
        node_simulator1()
    except rospy.ROSInterruptException:
        pass

