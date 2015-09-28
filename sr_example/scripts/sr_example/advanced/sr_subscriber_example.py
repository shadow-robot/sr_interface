#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

"""
This is a simple subscriber example, subscribing to the joint_states topic and printing
out the data in a per-joint basis.
To view the joint_states, type the following in a terminal:
> rostopic echo /joint_states

"""

import roslib
import rospy
import math
from sensor_msgs.msg import JointState

roslib.load_manifest('sr_example')


def callback(joint_state):
    """
    The callback function for the topic /joint_states

    It displays the received information in the console.

    @param joint_state: the message containing the joints data.
    """
    for joint_name, position, velocity, effort in zip(joint_state.name, joint_state.position,
                                                      joint_state.velocity, joint_state.effort):
        rospy.loginfo(
            "[%s] : Pos = %f | Pos_deg = %f | Vel = %f | Effort = %f",
            joint_name, position, math.degrees(position), velocity, effort)


def listener():
    """
    Initialize the ROS node and the topic to which it subscribes.
    """
    rospy.init_node(
        'subscriber_example', anonymous=True)

    # Subscribes to topic 'joint_states'
    rospy.Subscriber("joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
