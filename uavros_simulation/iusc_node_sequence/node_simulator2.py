#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

def node_simulator2():
    rospy.init_node('node_simulator2', anonymous=True)
    pub = rospy.Publisher("Task2Done", String, queue_size=10)
    time.sleep(2)
    pub.publish("Task2 Done")

if __name__ == '__main__':
    try:
        node_simulator2()
    except rospy.ROSInterruptException:
        pass

