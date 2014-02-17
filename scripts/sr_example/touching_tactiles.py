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
This is an example showing how to get the data from the tactiles. We first touch the first
finger and the thumb together to create a contact.
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
    The Publisher publishes a few commands to touch the first finger and the thumb
    """
    joints = ["ffj0", "ffj3", "thj4", "thj5"]
    #the fingers are touching with the following targets
    targets = [math.radians(80), #ffj0
               math.radians(44), #ffj3
               math.radians(46), #thj4
               math.radians(48)]  #thj5

    #Initalize the ROS node
    rospy.init_node('shadowhand_tactile_example_python')

    #initializing the publishers we'll use
    pubs = []
    for joint in joints:
        pubs.append( rospy.Publisher('sh_'+ joint + controller_type + '/command', Float64, latch=True) )

    #This will move the first finger and the thumb to touch
    for pub,target in zip(pubs,targets):
        print "sending: ",math.degrees(target)
        pub.publish(target)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
