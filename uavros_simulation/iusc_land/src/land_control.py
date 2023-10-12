#!/usr/bin/env python
import sys
import argparse
import rospy

from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix

# mavros service
from mavros_msgs.srv import (
    CommandTOL,
    CommandTOLRequest,
    CommandBool,
    CommandBoolRequest,
    SetMode,
    SetModeRequest,
)
from rosgraph import masterapi


class LandControl:
    def __init__(self) -> None:
        rospy.init_node("land_control", argv=sys.argv)
        self.timeout = 1.0

    def run(self):
        parser = argparse.ArgumentParser(description="iusc out door land")

        # command you could choice:
        commands = ["arm", "disarm", "takeoff", "offboard", "land"]
        parser.add_argument("uav_id", type=int, default=None, help="id")
        parser.add_argument("command", choices=commands, help="command ")
        args = parser.parse_args()

        # print(args)
        rosmaster = masterapi.Master("/")
        systemstate = rosmaster.getSystemState()
        self.services = [service[0] for service in systemstate[2]]
        self.topics = [t for t, _ in rospy.get_published_topics()]

        if args.command == "takeoff":
            # takeoff and arm
            # set home position
            template = "/uav{}/mavros/home_position/home"
            home_position = rospy.wait_for_message(
                template.format(args.uav_id), HomePosition
            )
            # take off request
            request = CommandTOLRequest()
            request.latitude = home_position.geo.latitude
            request.longitude = home_position.geo.longitude
            request.altitude = 2.5  # TODO 首先设为最低高度 2.5m
            request.yaw = 1.570796

            self.call_service(
                "mavros/cmd/takeoff",
                service_class=CommandTOL,
                requst=request,
                uav_id=int(args.uav_id),
            )

            self.call_service(
                "mavros/cmd/arming",
                service_class=CommandBool,
                requst=CommandBoolRequest(True),
                uav_id=int(args.uav_id),
            )

        elif args.command == "offboard":
            # send some message to px4, if not your request will be rejucted
            # 原地
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2
            template = "/uav{}/mavros/setpoint_position/local"

            local_pub = rospy.Publisher(
                template.format(args.uav_id), PoseStamped, queue_size=10
            )

            rate_20 = rospy.Rate(20.0)
            for _ in range(10):
                local_pub.publish(pose)
                rate_20.sleep()

            self.call_service(
                "mavros/set_mode",
                service_class=SetMode,
                requst=SetModeRequest(custom_mode="OFFBOARD"),
                uav_id=int(args.uav_id),
            )

        elif args.command == "disarm":
            pass

        elif args.command == "land":
            self.call_service(
                "mavros/cmd/land",
                service_class=CommandTOL,
                requst=CommandTOLRequest(),
                uav_id=int(args.uav_id),
            )

    def loginfo(self, *args, **kwargs):
        print("Land Control: ", *args, **kwargs)

    def logerr(self, *args, **kwargs):
        print("Land Control:", *args, file=sys.stderr, **kwargs)

    def _arg_uav_on_ground(self):
        pass

    def call_service(self, name, service_class, requst=None, uav_id=-1, function=None):
        # call function and return on Fasle
        # TODO 完成起飞
        if function is not None and not function():
            self.logerr("function false")
            return
        services = [s for s in self.services if name in s]
        # 过滤 编号 服务
        if uav_id != -1:
            services = [s for s in services if "/uav{}/".format(uav_id) in s]

        # call each service
        for service in services:
            rospy.wait_for_service(service, self.timeout)
            service_proxy = rospy.ServiceProxy(service, service_class)
            response = service_proxy.call(requst)
            if getattr(response, "success", False) or getattr(
                response, "mode_sent", False
            ):
                self.loginfo("{} success.".format(service))
            else:
                self.logerr("{} failed.".format(service))


def main():
    LandControl().run()
    rospy.spin()


if __name__ == "__main__":
    main()
