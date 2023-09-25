#!/usr/bin/env python
import sys
import argparse
import rospy

from mavros_msgs.msg import HomePosition

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
        commands = ["arm", "disarm", "takeoff", "offboard"]
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
            request.altitude = 1.0  # TODO 首先设为1
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
            self.call_service(
                "mavros/set_mode",
                service_class=SetMode,
                requst=SetModeRequest(custom_mode="OFFBOARD"),
                uav_id=int(args.uav_id),
            )

    def loginfo(self, *args, **kwargs):
        print("Land Control: ", *args, **kwargs)

    def logerr(self, *args, **kwargs):
        print("Land Control:", *args, file=sys.stderr, **kwargs)

    def call_service(self, name, service_class, requst=None, uav_id=-1, function=None):
        # call function and return on Fasle
        # TODO 完成起飞
        if function is not None and not function():
            self.logerr("function false")
            return
        services = [s for s in self.services if name in s]
        # 过滤 编号 服务
        if uav_id != -1:
            services = [s for s in self.services if "/uav{}/".format(uav_id) in s]

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


if __name__ == "__main__":
    main()
