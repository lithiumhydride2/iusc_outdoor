#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import subprocess
import time
import sys


def node_sequence():
    data = rospy.wait_for_message("Task1Done", String, timeout=None)
    if data.data == "Task1 Done":
        rospy.loginfo("Received 'Task1 Done' message. Starting the next node.")
        # 在此处启动下一个节点，您可以使用roslaunch或rospy来实现
        subprocess.Popen(["rosrun", "node_sequence", "node_simulator2.py"])

    data = rospy.wait_for_message("Task2Done", String, timeout=None)
    if data.data == "Task2 Done":
        rospy.loginfo("Received 'Task2 Done' message. Starting the next node.")
        # 在此处启动下一个节点，您可以使用roslaunch或rospy来实现
        # 启动 stage3 节点
        subprocess.Popen(
            ["roslaunch", "iusc_land", "iusc_outdoor_stage3.launch", "i:=1"]
        )


if __name__ == "__main__":
    rospy.init_node("node_sequence", anonymous=True, argv=sys.argv)
    # need to get uav number here
    assert len(sys.argv) >= 2, "argument count not enough"
    uav_id = sys.argv[1]
    subprocess.Popen(["rosrun", "node_sequence", "node_simulator1.py"])
    subprocess.Popen(["rosrun", "node_sequence", "node_simulator2.py"])
    try:
        node_sequence()
    except rospy.ROSInterruptException:
        pass
