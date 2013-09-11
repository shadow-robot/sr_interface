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


import roslib; roslib.load_manifest('sr_hand')
import rospy

from std_msgs.msg import Int16
import time
import math

def talker():
    """
    The Publisher publishes to the different topics on which the sr_subscriber subscribes. It sends commands to
    the hand.

    Please set the message you want to test.
    """
    pub2 = rospy.Publisher('/test_2', Int16)
    pub1 = rospy.Publisher('/test_1', Int16)
    rospy.init_node('shadowhand_command_publisher_python')
    angle1 = 0
    angle2 = 0
    msg1 = Int16()
    msg2 = Int16()

    step_nb = 0

    while not rospy.is_shutdown():
        target1 = math.cos( math.radians(angle1) )
        angle1 += .01
        if angle1 > 90:
            angle1 = -90
        msg1.data = target1 * 20000
        pub1.publish( msg1 )

        if step_nb ==  5:
            target2 = math.sin( math.radians(angle2) )
            angle2 += 1
            if angle2 > 180:
                angle2 = 0

            msg2.data = target2*30000
            pub2.publish( msg2 )
            step_nb = 0

        step_nb += 1
        time.sleep(.005)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
