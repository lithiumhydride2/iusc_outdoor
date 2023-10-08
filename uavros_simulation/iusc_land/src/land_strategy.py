#!/usr/bin/env python
import sys
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from colorama import Fore, Style
from sensor_msgs.msg import NavSatFix


class LandStrategy:
    def __init__(self) -> None:
        rospy.init_node("land_strategy", argv=sys.argv)
        args = rospy.myargv(argv=sys.argv)
        assert len(args) >= 2, "args not enough"
        self.uav_id = int(args[1])
        self.state = None
        self.local_pos = [0.0 for _ in range(3)]
        # this node is in uav1 namespace
        self.state_sub = rospy.Subscriber(
            "mavros/state", State, self.state_callback, queue_size=10
        )
        # self.global_pos_sub = rospy.Subscriber(
        #     "mavros/global_position/global", NavSatFix, self.global_pos_cb
        # )
        self.local_pos_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.local_pos_cb, queue_size=10
        )
        # self.global_pos_pub = rospy.Publisher(
        #     "mavros/setpoint_position/global", GeoPoseStamped, queue_size=10
        # )
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        # wait for px4
        rospy.wait_for_service("/uav{}/mavros/get_loggers".format(self.uav_id), 5)
        self.loginfo(" uav{} land strategy init done!".format(self.uav_id))
        self.rate30 = rospy.Rate(30.0)
        self.rate1 = rospy.Rate(1.0)

    def run(self):
        # wait for offboard mode
        while not rospy.is_shutdown():
            # self.loginfo(" uav{} wait for OFFBOARD".format(self.uav_id))
            if self.state is not None and self.state.mode == "OFFBOARD":
                rospy.loginfo_once(
                    Fore.GREEN
                    + "uav{} is in offboard mode".format(self.uav_id)
                    + Style.RESET_ALL
                )
                break

        if self.uav_id == 1:
            # track way point
            self.follow_way_point()
            pass
        elif self.uav_id > 1 and self.uav_id <= 6:
            self.follow_way_point()
            pass

    def follow_way_point(self):
        template = "/uav{}_way_point"
        self.way_points = rospy.get_param(template.format(self.uav_id))
        num_way_points = len(self.way_points)
        assert num_way_points > 1, "way points not enough"

        for way_point in self.way_points:
            way_point_in_local_axis = self.way_point_to_local_axis(way_point=way_point)
            self.set_way_point(way_point_in_local_axis)
            self.wait_for_approach(way_point_in_local_axis)

        self.loginfo("approach all way point")

    def wait_for_approach(self, way_point):
        # error = [self.local_pos[i] - way_point[i] for i in range(3)]
        error = []
        # TODO: BUG: index out of range
        error[0] = self.local_pos[0] - way_point[0]
        error[1] = self.local_pos[1] - way_point[1]
        error[2] = self.local_pos[2] - way_point[2]
        while not all([abs(i) < 0.2 for i in error]):
            self.loginfo(
                "uav{} waitting for approaching way point with error : ".format(
                    self.uav_id
                ),
                error,
            )
            self.loginfo(self.local_pos, way_point)
            self.rate1.sleep()
        self.loginfo(" uav{} approach one way point".format(self.uav_id))

    def set_way_point(self, way_point: list):
        temp = PoseStamped()
        temp.pose.position.x = way_point[0]
        temp.pose.position.y = way_point[1]
        temp.pose.position.z = way_point[2]
        for _ in range(5):
            self.local_pos_pub.publish(temp)
            self.rate30.sleep()

    def state_callback(self, msg):
        self.state = msg
        # self.loginfo("current uav{} state : {}".format(self.uav_id, self.state))

    def local_pos_cb(self, msg):
        self.rate1.sleep()
        self.local_pos[0] = float(msg.pose.position.x)
        self.local_pos[1] = float(msg.pose.position.y)
        self.local_pos[2] = float(msg.pose.position.z)
        # TODO : transform local_pos to world_pos

    def way_point_to_global_axis(self, way_point: list):
        pass

    def way_point_to_local_axis(self, way_point: list):
        # TODO : transform way_point to world axis
        gap = [self.uav_id * 2.0, 0.0, 0.0]
        temp = [way_point[i] - gap[i] for i in range(len(way_point))]
        pass
        return temp

    def loginfo(self, *args, **kwargs):
        print(Fore.GREEN + "Land strategy : " + Style.RESET_ALL, *args, **kwargs)

    def logerr(self, *args, **kwargs):
        print("Land strategy : ", *args, file=sys.stderr, **kwargs)


def main():
    # try:
    land_strategy_node = LandStrategy()
    rate10 = rospy.Rate(10.0)
    try:
        while not rospy.is_shutdown():
            land_strategy_node.run()
            rate10.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    # except rospy.ROSInterruptException as e:
    # print("error in land strategy")
    # rospy.logerror(e)


if __name__ == "__main__":
    main()
