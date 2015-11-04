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
        super(SrArmCommander, self).__init__(name)

        if set_ground:
            self._set_ground()

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_to_position_target(xyz, end_effector_link, wait=wait)

    def plan_to_position_target(self, xyz, end_effector_link=""):
        """
        Specify a target position for the end-effector and plans.
        This is a blocking method.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        """
        self._plan_to_position_target(xyz, end_effector_link)

    def move_to_pose_target(self, pose, end_effector_link="", wait=True):
        """
        Specify a target pose for the end-effector and moves to it
        @param pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z]
        or a list of 7 floats: [x, y, z, qx, qy, qz, qw]

        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_to_pose_target(pose, end_effector_link, wait=wait)

    def plan_to_pose_target(self, pose, end_effector_link=""):
        """
        Specify a target pose for the end-effector and plans.
        This is a blocking method.
        @param pose - new pose of end-effector: a Pose message, PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z]
        or a list of 7 floats [x, y, z, qx, qy, qz, qw]

        @param end_effector_link - name of the end effector link
        """
        self._plan_to_pose_target(pose, end_effector_link)

    def _set_ground(self):
        """
        Sets a plane for the ground.
        """
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        # offset such that the box is 0.1 mm below ground
        # (to prevent collision with the robot itself)
        pose.pose.position.z = -0.0501
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = get_rostime()
        pose.header.frame_id = self._robot_commander.get_root_link()
        self._planning_scene.add_box("ground", pose, (3, 3, 0.1))

    def get_pose_reference_frame(self):
        return self._move_group_commander.get_pose_reference_frame()

    def plan_cartesian_path_to_pose(self, target_pose, min_fraction=1, eef_step=.01, jump_threshold=1000):
        if type(target_pose) != Pose:
            rospy.logerr("Target should be Pose.")
            return None

        start_pose = self._move_group_commander.get_current_pose().pose

        (plan, fraction) = self._move_group_commander.compute_cartesian_path(
            [start_pose, target_pose], eef_step, jump_threshold)

        if fraction < min_fraction:
            rospy.logerr("Couldn't reach enough waypoints, only %f" % fraction)
            self._reset_plan()
            return None

        self._set_plan(plan)
        return True
