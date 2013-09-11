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
import time

from sr_robot_msgs.msg import joint, sendupdate, contrlr

def talker():
    """
    The Publisher publishes to the different topics on which the sr_subscriber subscribes. It sends commands to
    the hand.

    Please set the message you want to test.
    """
    test_what = "sendupdate" # choose sendupdate or contrlr

    """
    Test the sendupdate command
    """
    if test_what == "sendupdate":
        pub1 = rospy.Publisher('srh/sendupdate', sendupdate)
        rospy.init_node('shadowhand_command_publisher_python')

        new_target = 0

        time.sleep(1)
        print "publishing"
        data_to_send = [ joint(joint_name="FFJ0", joint_target=new_target),
                         joint(joint_name="FFJ3", joint_target=new_target) ]

        pub1.publish(sendupdate(len(data_to_send), data_to_send))

    """
    Tests the contrlr command
    """
    if test_what == "contrlr":
        pub2 = rospy.Publisher('contrlr', contrlr)

        data_to_send = ["p:0","i:0"]

        pub2.publish( contrlr( contrlr_name="smart_motor_ff2" ,
                               list_of_parameters = data_to_send,
                               length_of_list = len(data_to_send) ) )



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
