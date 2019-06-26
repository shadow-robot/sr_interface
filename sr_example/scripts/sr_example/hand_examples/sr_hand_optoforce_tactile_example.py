#!/usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

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
