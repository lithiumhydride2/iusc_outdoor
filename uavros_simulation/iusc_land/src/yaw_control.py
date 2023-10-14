#!/usr/bin/env python
# no use now !!!
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class YawControlNode:
    def __init__(self):
        rospy.init_node("yaw_control_node", anonymous=True)

        # 订阅yaw角目标值的话题
        rospy.Subscriber("desired_yaw", Float64, self.desired_yaw_callback)

        # 发布控制指令的话题
        topic_name = rospy.resolve_name("mavros/cmd_vel")
        self.cmd_vel_pub = rospy.Publisher("/mavros/cmd_vel", Twist, queue_size=10)

        self.desired_yaw = 0.0
        self.current_yaw = 0.0

    def desired_yaw_callback(self, data):
        # 订阅到目标yaw角的回调函数
        self.desired_yaw = data.data

    def control_yaw(self):
        rate = rospy.Rate(10)  # 控制频率，可以根据需要调整

        while not rospy.is_shutdown():
            # 计算yaw角误差
            yaw_error = self.desired_yaw - self.current_yaw  # 这里需要获取当前的yaw角

            # 创建Twist消息，设置yaw角的控制命令
            twist_msg = Twist()
            twist_msg.angular.z = k * yaw_error  # k 是控制增益，根据需要调整

            # 发布控制指令
            self.cmd_vel_pub.publish(twist_msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        yaw_control_node = YawControlNode()
        yaw_control_node.control_yaw()
    except rospy.ROSInterruptException:
        pass
