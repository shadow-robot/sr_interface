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
import roslib; roslib.load_manifest('sr_example')
import rospy
from sr_robot_msgs.msg import JointControllerState
from std_msgs.msg import Float64

parent_name = "ffj3"
child_name = "mfj3"
controller_type = "_mixed_position_velocity_controller"

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

