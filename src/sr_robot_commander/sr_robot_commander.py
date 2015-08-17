#!/usr/bin/env python

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

import threading

import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from moveit_commander import MoveGroupCommander, RobotCommander, \
    PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, \
    RobotTeachModeResponse
from trajectory_msgs.msg import JointTrajectoryPoint
from math import radians


class SrRobotCommander(object):
    """
    Base class for hand and arm commanders
    """
    __group_prefixes = {"right_arm": "ra_",
                        "left_arm": "la_",
                        "right_hand": "rh_",
                        "left_hand": "lh_"}

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object
        @param name - name of the MoveIt group
        """
        self._name = name
        self._move_group_commander = MoveGroupCommander(name)
        self._robot_commander = RobotCommander()
        self._planning_scene = PlanningSceneInterface()

        self._move_group_commander.set_planner_id("ESTkConfigDefault")

        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = \
            rospy.Subscriber("joint_states", JointState,
                             self._joint_states_callback, queue_size=1)
        self._joints_position = {}
        self._joints_velocity = {}
        self._joints_effort = {}
        self.__plan = None

        # prefix of the trajectory controller
        self._prefix = self.__group_prefixes[name]

        self._set_up_action_client()

        threading.Thread(None, rospy.spin)

    def execute(self):
        """
        Executes the last plan made.
        """
        if self.__plan is not None:
            self._move_group_commander.execute(self.__plan)
            self.__plan = None
        else:
            rospy.logwarn("No plans where made, not executing anything.")

    def move_to_joint_value_target(self, joint_states, wait=True,
                                   angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param wait - should method wait for movement end or not
        @param angle_degrees - are joint_states in degrees or not
        """
        if angle_degrees:
            joint_states.update((joint, radians(i))
                                for joint, i in joint_states.items())
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_joint_value_target(joint_states)
        self._move_group_commander.go(wait=wait)

    def plan_to_joint_value_target(self, joint_states, angle_degrees=False):
        """
        Set target of the robot's links and plans.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param angle_degrees - are joint_states in degrees or not
        This is a blocking method.
        """
        if angle_degrees:
            joint_states.update((joint, radians(i))
                                for joint, i in joint_states.items())
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_joint_value_target(joint_states)
        self.__plan = self._move_group_commander.plan()

    def move_to_named_target(self, name, wait=True):
        """
        Set target of the robot's links and moves to it
        @param name - name of the target pose defined in SRDF
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_named_target(name)
        self._move_group_commander.go(wait=wait)

    def plan_to_named_target(self, name):
        """
        Set target of the robot's links and plans
        This is a blocking method.
        @param name - name of the target pose defined in SRDF
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_named_target(name)
        self.__plan = self._move_group_commander.plan()

    def get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        with self._joint_states_lock:
            return self._joints_position

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        with self._joint_states_lock:
            return self._joints_velocity

    def _get_joints_effort(self):
        """
        Returns joints effort
        @return - dictionary with joints efforts
        """
        with self._joint_states_lock:
            return self._joints_effort

    def run_joint_trajectory(self, joint_trajectory):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents
        trajectory of the joints which would be executed.
        """
        plan = RobotTrajectory()
        plan.joint_trajectory = joint_trajectory
        self._move_group_commander.execute(plan)

    def _move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self._move_group_commander.go(wait=wait)

    def _plan_to_position_target(self, xyz, end_effector_link=""):
        """
        Specify a target position for the end-effector and plans.
        This is a blocking method.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self.__plan = self._move_group_commander.plan()

    def _move_to_pose_target(self, pose, end_effector_link="", wait=True):
        """
        Specify a target pose for the end-effector and moves to it
        @param pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
        of 7 floats [x, y, z, qx, qy, qz, qw]
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_pose_target(pose, end_effector_link)
        self._move_group_commander.go(wait=wait)

    def _plan_to_pose_target(self, pose, end_effector_link=""):
        """
        Specify a target pose for the end-effector and plans.
        This is a blocking method.
        @param pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
        of 7 floats [x, y, z, qx, qy, qz, qw]
        @param end_effector_link - name of the end effector link
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_pose_target(pose, end_effector_link)
        self.__plan = self._move_group_commander.plan()

    def _joint_states_callback(self, joint_state):
        """
        The callback function for the topic joint_states.
        It will store the received joint position, velocity and efforts
        information into dictionaries
        @param joint_state - the message containing the joints data.
        """
        with self._joint_states_lock:
            self._joints_position = {n: p for n, p in
                                     zip(joint_state.name,
                                         joint_state.position)}
            self._joints_velocity = {n: v for n, v in
                                     zip(joint_state.name,
                                         joint_state.velocity)}
            self._joints_effort = {n: v for n, v in
                                   zip(joint_state.name, joint_state.effort)}

    def _set_up_action_client(self):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        self._client = SimpleActionClient(
            self._prefix + "trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        if self._client.wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to action server in 4 sec")
            raise

    def move_to_joint_value_target_unsafe(self, joint_states, time=0.002,
                                          wait=True, angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param time - time in s (counting from now) for the robot to reach the
        target (it needs to be greater than 0.0 for it not to be rejected by
        the trajectory controller)
        @param wait - should method wait for movement end or not
        @param angle_degrees - are joint_states in degrees or not
        """
        # self._update_default_trajectory()
        # self._set_targets_to_default_trajectory(joint_states)
        if angle_degrees:
            joint_states.update((joint, radians(i))
                                for joint, i in joint_states.items())
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(joint_states.keys())
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(time)
        for key in goal.trajectory.joint_names:
            point.positions.append(joint_states[key])

        goal.trajectory.points = []
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)

        if not wait:
            return

        if not self._client.wait_for_result():
            rospy.loginfo("Trajectory not completed")

    def run_joint_trajectory_unsafe(self, joint_trajectory, wait=True):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents
        trajectory of the joints which would be executed.
        @param wait - should method wait for movement end or not
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory
        self._client.send_goal(goal)

        if not wait:
            return

        if not self._client.wait_for_result():
            rospy.loginfo("Trajectory not completed")

    def set_teach_mode(self, teach):
        """
        Activates/deactivates the teach mode for the robot.
        Activation: stops the the trajectory controllers for the robot, and
        sets it to teach mode.
        Deactivation: stops the teach mode and starts trajectory controllers
        for the robot.
        Currently this method blocks for a few seconds when called on a hand,
        while the hand parameters are reloaded.
        @param teach - bool to activate or deactivate teach mode
        """

        if teach:
            mode = RobotTeachModeRequest.TEACH_MODE
        else:
            mode = RobotTeachModeRequest.TRAJECTORY_MODE
        self.change_teach_mode(mode, self._name)

    @staticmethod
    def change_teach_mode(mode, robot):
        teach_mode_client = rospy.ServiceProxy('/teach_mode', RobotTeachMode)

        req = RobotTeachModeRequest()
        req.teach_mode = mode
        req.robot = robot
        try:
            resp = teach_mode_client(req)
            if resp.result == RobotTeachModeResponse.ERROR:
                rospy.logerr("Failed to change robot %s to mode %d", robot,
                             mode)
            else:
                rospy.loginfo("Changed robot %s to mode %d Result = %d", robot,
                              mode, resp.result)
        except rospy.ServiceException:
            rospy.logerr("Failed to call service teach_mode")
