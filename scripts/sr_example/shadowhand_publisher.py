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
This is an example showing how to publish command messages to the hand.
"""


import roslib; roslib.load_manifest('sr_example')
import rospy
import time
import math

from std_msgs.msg import Float64

# type of controller that is running
controller_type = "_position_controller"

def talker():
    """
    The Publisher publishes two commands to move joint FFJ0 and RFJ0

    """
    joint1 = 'ffj0'
    joint2 = 'rfj0'

    #Initalize the ROS node
    rospy.init_node('shadowhand_command_publisher_python')

    pub1 = rospy.Publisher('sh_'+ joint1 + controller_type + '/command', Float64, latch=True)
    pub2 = rospy.Publisher('sh_'+ joint2 + controller_type + '/command', Float64, latch=True)


    # define a new target value for the joint position.
    # The position controllers expect their commands in radians
    new_target_1 = math.radians(0.0)
    new_target_2 = math.radians(0.0)

    time.sleep(1)
    print "publishing"

    #This will move the joint ffj0 to the defined target (0 degrees)
    pub1.publish(new_target_1)

    #This will move the joint rfj0 to the defined target (0 degrees)
    pub2.publish(new_target_2)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
