#!/usr/bin/env python
import rospy
import gazebo_msgs.msg
from gazebo_msgs.srv import GetWorldProperties

rospy.init_node("get_gazebo_world_origin", anonymous=True)

# 创建一个Gazebo服务客户端来获取世界属性
world_properties_client = rospy.ServiceProxy(
    "/gazebo/get_world_properties", GetWorldProperties
)
response = world_properties_client()

# 获取世界坐标系的原点经纬度
print(response)

# rospy.loginfo("Gazebo World Origin Latitude: %f" % origin_latitude)
# rospy.loginfo("Gazebo World Origin Longitude: %f" % origin_longitude)

rospy.spin()
