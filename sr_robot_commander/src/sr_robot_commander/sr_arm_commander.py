#!/usr/bin/python

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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

import rospy
from geometry_msgs.msg import PoseStamped
from sr_robot_commander.sr_robot_commander import SrRobotCommander


class SrArmCommander(SrRobotCommander):
    """
    Commander class for arm
    """

    def __init__(self, name="right_arm", set_ground=True):
        """
        Initialize object
        @param name - name of the MoveIt group
        @param set_ground - sets the ground plane in moveit for planning
        """
        try:
            super().__init__(name)
        except Exception as exception:
            # TODO(@dg-shadow): Raise SrRobotCommanderException here. Not doing
            # now as no time to check for and repair unforeseen consequences.
            rospy.logerr(f"Couldn't initialise robot commander - is there an arm running?: {str(exception)}")
            self._move_group_commander = None
            return

        if set_ground:
            self.set_ground()

    def arm_found(self):
        return self._move_group_commander is not None

    def set_ground(self, height=0.1, z_position=-0.1):
        """
        Sets a plane for the ground.
        @param height - specifies the height of the plane
        @param z_position - position in z to place the plane. Should not collide with the robot.
        """

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = z_position - (height / 2.0)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = self._robot_commander.get_root_link()
        self._planning_scene.add_box("ground", pose, (3, 3, height))

    def get_pose_reference_frame(self):
        return self._move_group_commander.get_pose_reference_frame()
