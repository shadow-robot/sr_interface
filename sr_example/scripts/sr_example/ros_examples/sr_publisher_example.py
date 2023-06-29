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
#
# -*- coding: utf-8 -*-

"""
An example that demonstrates how to publish command messages to joints in the hand.
The hand should be launched but the trajectory controllers should
not be running as they overwrite position commands. To check if they are, a call to
rosservice can be made with the following command:
>rosservice call /controller_manager/list_controllers
To stop the trajectory controllers, open the gui (type rqt) and select position control in
plugins > Shadow Robot > Change controllers

"""

import time
import math
import rospy
import roslib
from std_msgs.msg import Float64

roslib.load_manifest('sr_example')

# Controller that controls joint position
controller_type = "_position_controller"


def talker():
    """
    The Publisher publishes two commands to move joint rh_FFJ0 and rh_RFJ0

    """
    joint1 = 'rh_ffj0'
    joint2 = 'rh_rfj0'

    # Initialise the ROS node
    rospy.init_node('publisher_example')

    pub1 = rospy.Publisher(
        'sh_' + joint1 + controller_type + '/command', Float64, latch=True, queue_size=1)
    pub2 = rospy.Publisher(
        'sh_' + joint2 + controller_type + '/command', Float64, latch=True, queue_size=1)

    # define a new target value for the joint position in degrees.
    # The position controllers expect their commands in radians
    new_target_1 = math.radians(20)
    new_target_2 = math.radians(20)

    time.sleep(1)
    print("publishing:")
    print(joint1 + " to " + str(math.degrees(new_target_1)) + " degrees\n" +
          joint2 + " to " + str(math.degrees(new_target_2)) + " degrees")

    # This will move the joint rh_ffj0 to the defined target (20 degrees)
    pub1.publish(new_target_1)

    # This will move the joint rh_rfj0 to the defined target (20 degrees)
    pub2.publish(new_target_2)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
