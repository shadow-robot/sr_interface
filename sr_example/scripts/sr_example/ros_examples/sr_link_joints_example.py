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
