#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.

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
