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


import roslib
import rospy
import time
import math
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
    print ("publishing:")
    print (joint1 + " to " + str(math.degrees(new_target_1)) + " degrees\n" +
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
