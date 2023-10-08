#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("custom_tf_pub")
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    custom_tf = TransformStamped()
    custom_tf.header.frame_id = "world"
    custom_tf.child_frame_id = "base_link"
    custom_tf.transform.translation.x = 0.0
    custom_tf.transform.translation.y = 0.0
    custom_tf.transform.translation.z = 0.0
    custom_tf.transform.rotation.x = 0.0
    custom_tf.transform.rotation.y = 0.0
    custom_tf.transform.rotation.z = 0.0
    custom_tf.transform.rotation.w = 1.0

    rate20 = rospy.Rate(20)
    while not rospy.is_shutdown():
        custom_tf.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(custom_tf)
        rate20.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
