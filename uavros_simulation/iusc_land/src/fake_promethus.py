#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


class FakePromethus:
    def __init__(self) -> None:
        # under uav{} namespace
        rospy.init_node("fake_promethus", argv=sys.argv)
        args = rospy.myargv(argv=sys.argv)
        assert len(args) >= 2, "args not enough"
        self.uav_id = int(args[1])

        # publisher and subscriber
        template = "/uav{}/mavros/setpoint_position/local"
        self.local_pub = rospy.Publisher(
            template.format(self.uav_id), PoseStamped, queue_size=10
        )
        self.last_local_pos = PoseStamped()
        self.last_local_pos.header.frame_id = "map"
        self.last_local_pos.pose.position.x = 0
        self.last_local_pos.pose.position.y = 0
        self.last_local_pos.pose.position.z = 2
        self.state_sub = rospy.Subscriber(
            template.format(self.uav_id), PoseStamped, self.state_callback
        )
        self.rate30 = rospy.Rate(30)

    def state_callback(self, msg):
        self.last_local_pos = msg

    def run(self):
        while not rospy.is_shutdown():
            self.local_pub.publish(self.last_local_pos)
            self.rate30.sleep()
        rospy.spin()


def main():
    fake_promethus_node = FakePromethus()
    fake_promethus_node.run()


if __name__ == "__main__":
    main()
    rospy.spin()