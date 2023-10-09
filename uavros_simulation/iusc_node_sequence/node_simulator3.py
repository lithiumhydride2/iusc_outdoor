#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

def node_simulator3():
    rospy.init_node('node_simulator3', anonymous=True)
    pub = rospy.Publisher("AllTaskDone", String, queue_size=10)
    time.sleep(2)
    pub.publish("AllTaskDone")

if __name__ == '__main__':
    try:
        node_simulator3()
    except rospy.ROSInterruptException:
        pass

