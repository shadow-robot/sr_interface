#!/usr/bin/python

# Copyright 2015 Shadow Robot Company Ltd.
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

from sr_robot_commander import SrRobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from rospy import get_rostime
import rospy
from tf import TransformerROS


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
            super(SrArmCommander, self).__init__(name)
        except Exception as e:
            rospy.logerr("Couldn't initialise robot commander - is there an arm running?: " + str(e))
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
        pose.pose.position.z = z_position - (height/2.0)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = get_rostime()
        pose.header.frame_id = self._robot_commander.get_root_link()
        self._planning_scene.add_box("ground", pose, (3, 3, height))

    def get_pose_reference_frame(self):
        return self._move_group_commander.get_pose_reference_frame()
