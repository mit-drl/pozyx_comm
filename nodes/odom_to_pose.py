#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

pub = None


def odom_cb(odom):
    if pub:
        pub.publish(odom.pose.pose)


if __name__ == "__main__":
    rospy.init_node("odom_to_pose", anonymous=False)
    pub = rospy.Publisher("pose", Pose)
    sub = rospy.Subscriber("odom", odom_cb, Odometry)
    rospy.spin()
