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
This is a simple subscriber example, subscribing to the srh/shadowhand_data topic
"""

import roslib; roslib.load_manifest('sr_hand')
import rospy
from sr_robot_msgs.msg import joints_data, joint

def callback(joint_data):
    """
    The callback function for the topic srh/shadowhand_Data. It just displays the received information on the console.
    
    @param joint_data: the message containing the joint data.
    """
    for joint in joint_data.joints_list:
        rospy.loginfo(rospy.get_name()+"[%s] : Pos = %f | Target = %f | Temp = %f | Current = %f",
                      joint.joint_name, joint.joint_position, joint.joint_target, joint.joint_temperature, 
                      joint.joint_current)

def listener():
    """
    Initialize the ROS node and the topic to which it subscribes.
    """
    rospy.init_node('shadowhand_subscriber_python', anonymous=True)
    rospy.Subscriber("srh/shadowhand_data", joints_data, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
