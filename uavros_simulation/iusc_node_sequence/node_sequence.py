#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import subprocess
import time

def node_sequence():
    rospy.init_node('node_sequence', anonymous=True)

    data = rospy.wait_for_message('Task2Done', String, timeout=None)
    if data.data == "Task2 Done":
        rospy.loginfo("Received 'Task2 Done' message. Starting the next node.")
        # 在此处启动下一个节点，您可以使用roslaunch或rospy来实现
        subprocess.Popen(["rosrun", "node_sequence", "node_simulator3.py"])

if __name__ == '__main__':
    subprocess.Popen(["rosrun", "node_sequence", "node_simulator1.py"])
    subprocess.Popen(["rosrun", "node_sequence", "node_simulator2.py"])
    try:
        node_sequence()
    except rospy.ROSInterruptException:
        pass



