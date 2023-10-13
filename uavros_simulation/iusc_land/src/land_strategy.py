#!/usr/bin/env python
import sys
import rospy
import subprocess
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from colorama import Fore, Style
from sensor_msgs.msg import NavSatFix
from iusc_maze.srv import map2localRequest, map2localResponse, map2local
from std_msgs.msg import String
from collections import defaultdict
from std_msgs.msg import Int8
import math

velocity = 4.0  # 设置速度，单位可以是米/秒

class LandStrategy:
    def __init__(self) -> None:
        rospy.init_node("land_strategy", argv=sys.argv, anonymous=True)

        args = rospy.myargv(argv=sys.argv)
        assert len(args) >= 2, "args not enough"
        self.uav_id = int(args[1])
        self.state: State = None
        self.local_pos = [0.0 for _ in range(3)]
        self.rate30 = rospy.Rate(30.0)
        self.rate1 = rospy.Rate(1.0)
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
            "mavros/setpoint_raw/local", PoseStamped, queue_size=10
        )
        # wait for map2local service
        templete = "/uav{}/map2local_server"
        self.map2local_client = rospy.ServiceProxy(
            templete.format(self.uav_id), map2local
        )
        # wait for px4
        rospy.wait_for_service("/uav{}/mavros/get_loggers".format(self.uav_id), 5)
        self.loginfo(" uav{} land strategy init done!".format(self.uav_id))

    def run(self):
        # wait for offboard mode
        rospy.wait_for_message("mavros/state", State)
        while not rospy.is_shutdown():
            self.rate1.sleep()
            rospy.loginfo_once(" uav{} wait for OFFBOARD".format(self.uav_id))
            if self.state.mode == "OFFBOARD":
                rospy.loginfo_once(
                    Fore.GREEN
                    + "uav{} is in offboard mode".format(self.uav_id)
                    + Style.RESET_ALL
                )
                break

        if self.uav_id == 1:
            # track way point
            template = rospy.resolve_name("uav{}_way_point")
            self.way_points = rospy.get_param(template.format(self.uav_id))
            self.follow_way_point()
            # 此处应该等待 5 秒使无人机稳定，图像稳定
            rospy.sleep(5.0)
            subprocess.Popen(
                ["rosrun", "iusc_land", "formation_detect.py", str(self.uav_id)]
            )
            # 等待 formation str 信息
            formation_str: String = rospy.wait_for_message(
                "/uav{}/formation_str".format(self.uav_id), String
            )
            formation_str = str(formation_str.data)
            self.loginfo("formation_detect is : ", formation_str)
            assert len(formation_str) == 16, "wrong formation str"
            # store it in dict
            formation_dict = defaultdict(list)
            for index, item in enumerate(formation_str):
                formation_dict[item].append(index)
            # 如果出于某种原因，绿色降落点数量不为6，将其补全为6,从 formation_dict["R"] 中随机抽取
            if len(formation_dict["G"]) > 6:
                formation_dict["G"] = formation_dict["G"][:6]
            elif len(formation_dict["G"]) < 6:
                missing_count = 6 - len(formation_dict["G"])
                avaliable_indices = formation_dict["R"][:missing_count]
                formation_dict["G"].extend(avaliable_indices)
            # 现在可以发布 ， 当前简单的使用顺序发布目标降落位置，你可以采取自己的策略
            rospy.loginfo(formation_dict["G"])
            self.land_pos_index = formation_dict["G"]

            ################ 发布目标降落索引 #################################
            self.pub_land_pos_index()
            self.loginfo("land position index publish !")
            # follow land way point
            self.follow_land_way_point()
            pass

        elif self.uav_id > 1 and self.uav_id <= 3:
            template = rospy.resolve_name("uav{}_way_point")
            self.way_points = rospy.get_param(template.format(self.uav_id))
            self.follow_way_point()
            # follow land way point
            self.follow_land_way_point()
            #
            pass

    def follow_land_way_point(self):
        template = rospy.resolve_name("land_way_point")
        land_way_points = rospy.get_param(template)
        template = "/uav1/uav{}/land_index".format(self.uav_id)

        land_points_index = rospy.wait_for_message(template, Int8)
        self.loginfo(
            " uav{} get land index :".format(self.uav_id), land_points_index.data
        )
        self.way_points = land_way_points[land_points_index.data]
        print(self.way_points, land_way_points)
        self.follow_way_point()
        pass

    def pub_land_pos_index(self):
        pass
        # uav1 控制 uav2 至 uav6
        index_publishers = [
            rospy.Publisher(
                "/uav{}/uav{}/land_index".format(self.uav_id, uav_number),
                Int8,
                latch=True,
                queue_size=10,
            )
            for uav_number in range(1, 7)
        ]

        for _ in range(5):
            for index, index_pub in enumerate(index_publishers):
                # 根据 index 索引目标降落位置
                index_pub.publish(self.land_pos_index[index])
            self.rate1.sleep()

    def follow_way_point(self):
        """
        get parameters from the topic : "/uav{}_way_point" and follow it!
        """
        # num_way_points = len(self.way_points)
        if not self.way_points:
            self.logerr("No way points")
            return
        if type(self.way_points[0]) != list:
            self.way_points = [self.way_points]
        # 设置速度和航点之间的最小距离
        min_distance = velocity  # 假设速度为1.0米/秒，最小距离也为1.0米

        # 生成新的路径点序列
        new_waypoints = []

        for i in range(len(self.way_points) - 1):
            start_point = self.way_points[i]
            end_point = self.way_points[i + 1]

            # 计算路径点之间的总距离
            total_distance = distance(start_point, end_point)

            # 计算需要的航点数量
            num_waypoints = int(total_distance / min_distance) + 1  # 加1是为了包括起点和终点

            for j in range(num_waypoints):
                alpha = float(j) / (num_waypoints - 1)  # 在0到1之间线性插值
                x = start_point[0] + alpha * (end_point[0] - start_point[0])
                y = start_point[1] + alpha * (end_point[1] - start_point[1])
                z = start_point[2] + alpha * (end_point[2] - start_point[2])
                new_waypoints.append([x, y, z])

        for way_point in new_waypoints:
            way_point_in_local_axis = self.way_point_to_local_axis(way_point)
            self.set_way_point(way_point_in_local_axis)
            self.wait_for_approach(way_point_in_local_axis)

        self.loginfo("approach all way point")

    def wait_for_approach(self, way_point):
        # error = [self.local_pos[i] - way_point[i] for i in range(3)]
        error = [1 for _ in range(3)]
        # self.loginfo(error, self.local_pos, way_point)

        while not all([abs(i) < 0.2 for i in error]):
            error = [self.local_pos[i] - way_point[i] for i in range(len(way_point))]
            self.loginfo(
                "uav{} waitting for approaching way point with error : ".format(
                    self.uav_id
                ),
                error,
            )
            # self.loginfo(self.local_pos, way_point)
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
        self.local_pos[0] = msg.pose.position.x
        self.local_pos[1] = msg.pose.position.y
        self.local_pos[2] = msg.pose.position.z
        self.rate1.sleep()
        # TODO : transform local_pos to world_pos

    def way_point_to_local_axis(self, way_point):
        request = map2localRequest()
        request.x_map = way_point[0]
        request.y_map = way_point[1]
        self.map2local_client.wait_for_service(5.0)
        response: map2localResponse = self.map2local_client.call(request)
        return [response.x_local, response.y_local, way_point[2]]

    def way_point_to_global_axis(self, way_point: list):
        pass

    # def way_point_to_local_axis(self, way_point: list):
    #     # TODO : transform way_point to world axis
    #     gap = [self.uav_id * 2.0, 0.0, 0.0]
    #     gap = {}
    #     gap[1] = [30 - 22.5, -1.0, 0.0]
    #     gap[2] = [30 - 15.0, -1.0, 0.0]
    #     gap[3] = [30 - 7.5 - 1.0, 0.0]
    #     gap[4] = [30 - 22.5, -3.0, 0.0]
    #     gap[5] = [30 - 15.0, -3.0, 0.0]
    #     gap[6] = [30 - 7.5, -3.0, 0.0]
    #     temp = [way_point[i] - gap[self.uav_id][i] for i in range(len(way_point))]
    #     pass
    #     return temp

    def loginfo(self, *args, **kwargs):
        print(Fore.GREEN + "Land strategy : " + Style.RESET_ALL, *args, **kwargs)

    def logerr(self, *args, **kwargs):
        print("Land strategy : ", *args, file=sys.stderr, **kwargs)

# 计算两点之间的距离
def distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)




def main():
    # try:
    land_strategy_node = LandStrategy()
    # try:
    land_strategy_node.run()
    # except rospy.ROSInterruptException as e:
    #     rospy.logerr(e)


if __name__ == "__main__":
    main()
