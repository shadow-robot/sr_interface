#!/usr/bin/env python

# Reading the optoforce tactiles from the hand.

import rospy
from geometry_msgs.msg import WrenchStamped


def callback(data):
    rospy.loginfo("At:" + str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs) +
                  " sensor:" + str(data.header.frame_id) +
                  " Force:" + str(data.wrench.force.x) + "," +
                  str(data.wrench.force.y) + "," + str(data.wrench.force.z))


def listener():
    num_sensors = 5
    rospy.init_node("optoforce_tactile_reader", anonymous=True)
    for sensor_num in range(num_sensors):
        rospy.Subscriber("optoforce_" + str(sensor_num), WrenchStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
