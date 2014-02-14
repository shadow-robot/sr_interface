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
This is a simple subscriber example, subscribing to the joint_states topic and printing out the data in a per-joint basis
To see how the joint_states topic looks like you can type the following in a terminal:
> rostopic echo /joint_states

"""

import roslib; roslib.load_manifest('sr_example')
import rospy
import math
from sensor_msgs.msg import JointState

def callback(joint_state):
    """
    The callback function for the topic /joint_states

    It just displays the received information on the console.

    @param joint_state: the message containing the joints data.
    """
    for joint_name, position, velocity, effort in zip(joint_state.name, joint_state.position, joint_state.velocity, joint_state.effort):
        rospy.loginfo("[%s] : Pos = %f | Pos_deg = %f | Vel = %f | Effort = %f",
                      joint_name, position, math.degrees(position), velocity, effort)

def listener():
    """
    Initialize the ROS node and the topic to which it subscribes.
    """
    rospy.init_node('shadowhand_joint_states_subscriber_python', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
