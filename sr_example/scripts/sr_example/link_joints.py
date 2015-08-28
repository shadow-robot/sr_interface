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

# This example can be tested with the simulated hand (or simulated hand and arm)
# by using the following command in a separate terminal
# roslaunch sr_hand  gazebo_arm_and_hand.launch
# or
# roslaunch sr_hand  gazebo_hand.launch
# and then
# rosrun sr_example link_joints.py
# rosrun rqt_gui rqt_gui

# in the rqt_gui go to plugins->ShadowRobot->joint slider and select EtherCAT hand
# If you move the joint slider for FFJ3, then MFJ3 will move as well.

import roslib; roslib.load_manifest('sr_example')
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

parent_name = "ffj3"
child_name = "mfj3"

# type of controller that is running
controller_type = "_position_controller"

pub = rospy.Publisher('sh_' + child_name + controller_type + '/command', Float64)

def callback(data):
    """
    The callback function: called each time a message is received on the
    topic parent joint controller state topic

    @param data: the message
    """
    # publish the message to the child joint controller command topic.
    # here we insert the joint name into the topic name
    pub.publish(data.set_point)


def listener():
    """
    The main function
    """
    # init the ros node
    rospy.init_node('joints_link_test', anonymous=True)

    # init the subscriber: subscribe to the
    # parent joint controller topic, using the callback function
    # callback()
    rospy.Subscriber('sh_'+parent_name + controller_type + '/state', JointControllerState, callback)
    # subscribe until interrupted
    rospy.spin()


if __name__ == '__main__':
    listener()
