#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import time


def msg_cb(msg):
    global first_stamp, now
    if first_stamp is None:
        now = rospy.Time.now()
        first_stamp = msg.header.stamp
    msg.header.stamp -= first_stamp
    msg.header.stamp += now
    for i in range(3):
        cloud_pub.publish(msg)

rospy.init_node('header_time_adjust')
first_stamp = None

cloud_pub = rospy.Publisher('/camera/depth_registered/points', PointCloud2, queue_size=20, latch=True)

cloud_sub = rospy.Subscriber('/camera/depth_registered/points_old', PointCloud2, msg_cb)

rospy.spin()
