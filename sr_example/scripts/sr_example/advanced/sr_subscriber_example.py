#!/usr/bin/env python3
#
# Software License Agreement (BSD License)
# Copyright © 2011, 2022-2023 belongs to Shadow Robot Company Ltd.
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

"""
This is a simple subscriber example, subscribing to the joint_states topic and printing
out the data in a per-joint basis.
To view the joint_states, type the following in a terminal:
> rostopic echo /joint_states

"""

import math
import rospy
import roslib
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
