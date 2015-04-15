# #!/usr/bin/python
#
# # Copyright 2015 Shadow Robot Company Ltd.
# #
# # This program is free software: you can redistribute it and/or modify it
# # under the terms of the GNU General Public License as published by the Free
# # Software Foundation, either version 2 of the License, or (at your option)
# # any later version.
# #
# # This program is distributed in the hope that it will be useful, but WITHOUT
# # ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# # FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# # more details.
# #
# # You should have received a copy of the GNU General Public License along
# # with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#
# from moveit_commander import MoveGroupCommander
# from moveit_msgs.msg import RobotTrajectory
#
#
# class SrRobotCommander(object):
#     """
#     Base class for hand and arm commanders
#     """
#
#     def __init__(self, name):
#         """
#         Initialize MoveGroupCommander object
#         @param name - name of the MoveIt group
#         """
#         self._move_group_commander = MoveGroupCommander(name)
#
#     def move_to_joint_value_target(self, joint_states, wait_result=True):
#         """
#         Set target of the robot's links and moves to it
#         @param joint_states - dictionary with joint name and value
#         @param wait_result - should method wait for movement end or not
#         """
#         self._move_group_commander.set_joint_value_target(joint_states)
#         self._move_group_commander.go(wait=wait_result)
#
#     def _run_joint_trajectory(self, joint_trajectory):
#         """
#         Moves robot through all joint states with specified timeouts
#         @param joint_trajectory - JointTrajectory class object. Represents trajectory of the joints which would be
#         executed.
#         """
#         plan = RobotTrajectory()
#         plan.joint_trajectory = joint_trajectory
#         self._move_group_commander.execute(plan)
#
#     def _move_to_position_target(self, xyz, end_effector_link="", wait_result=True):
#         """
#         Specify a target position for the end-effector and moves to it
#         @param xyz - new position of end-effector
#         @param end_effector_link - name of the end effector link
#         @param wait_result - should method wait for movement end or not
#         """
#         self._move_group_commander.set_position_target(xyz, end_effector_link)
#         self._move_group_commander.go(wait=wait_result)
