#!/usr/bin/env python
import rospy
import tf2_ros
import sys
import math
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
from haversine import haversine, Unit


class CustomTF:
    def __init__(self) -> None:
        rospy.init_node("custom_tf_node", argv=sys.argv)
        args = rospy.myargv(argv=sys.argv)
        assert len(args) >= 2, "args not enough"
        self.uav_id = int(args[1])
        templete = "/uav{}/mavros/global_position/global"
        # broadcaster
        self.global_pos_sub = rospy.Subscriber(
            templete.format(self.uav_id), NavSatFix, self.global_pos_cb, queue_size=10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.map_origin_global_pos = rospy.get_param(
            "/map_origin_global_pos", [47.3977417, 8.5455949, 535.23]
        )

    def global_pos_cb(self, msg: NavSatFix):
        # Geodesic.Inverse()
        g = Geodesic.WGS84.Inverse(
            msg.latitude,
            msg.longitude,
            self.map_origin_global_pos[0],
            self.map_origin_global_pos[1],
        )
        s12 = g["s12"]
        a12 = math.radians(g["azi1"])
        pos_in_world_axis = (-s12 * math.sin(a12), -s12 * math.cos(a12))

        tfs = TransformStamped()
        tfs.header.frame_id = "world"
        tfs.header.stamp = rospy.Time.now()

        tfs.child_frame_id = "uav{}".format(self.uav_id)
        tfs.transform.translation.x = pos_in_world_axis[0]
        tfs.transform.translation.y = pos_in_world_axis[1]
        tfs.transform.translation.z = self.map_origin_global_pos[2] - msg.altitude
        tfs.transform.rotation.x = 0.0
        tfs.transform.rotation.y = 0.0
        tfs.transform.rotation.z = 0.0
        tfs.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tfs)
        # point1 = (msg.latitude, msg.longitude)
        # point2 = (self.map_origin_global_pos[0], self.map_origin_global_pos[1])
        # result1 = haversine(point1, point2, unit=Unit.METERS)
        # print(
        #     msg.latitude,
        #     msg.longitude,
        #     self.map_origin_global_pos[0],
        #     self.map_origin_global_pos[1],
        #     g["s12"],
        # )


def main():
    custom_tf = CustomTF()
    rospy.spin()


if __name__ == "__main__":
    main()
