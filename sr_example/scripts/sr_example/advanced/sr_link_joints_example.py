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
This example demonstrates how two joints can have their positions 'linked' by
having a child joint subscribing to the parent joint's controller state topic.
The messages are then published to the child joint's controller.

The hand should be launched but the trajectory controllers should
not be running as they overwrite position commands. To check if they are, a call
to rosservice can be made with the following command:
>rosservice call /controller_manager/list_controllers
To stop the trajectory controllers, open the gui (type rqt) and select position control in
plugins > Shadow Robot > Change controllers
NOTE: If the joint sliders plugin is open during this change of controllers, it will
need to be reloaded.

A position can then be published to the parent joint or the joint could be moved by
using the joint sliders in the gui, plugins > Shadow Robot > joint slider

If you move the joint slider for the parent joint, the child joint will move as well.
"""

import roslib
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

roslib.load_manifest('sr_example')

# Joints to be linked
parent_name = "rh_ffj3"
child_name = "rh_rfj3"

# Controller that controls joint position
controller_type = "_position_controller"

pub = rospy.Publisher(
    '/sh_' + child_name + controller_type + '/command', Float64, queue_size=1)


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
    rospy.init_node('link_joints_example', anonymous=True)

    # init the subscriber: subscribe to the
    # parent joint controller topic, using the callback function
    # callback()
    rospy.Subscriber('/sh_' + parent_name + controller_type +
                     '/state', JointControllerState, callback)
    # subscribe until interrupted
    rospy.spin()


if __name__ == '__main__':
    listener()
