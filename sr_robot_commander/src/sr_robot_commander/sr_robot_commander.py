#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2015, 2022-2023 belongs to Shadow Robot Company Ltd.
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

import threading
from math import radians
import copy
import rospy
import numpy
from std_msgs.msg import Header

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, RobotTeachModeResponse
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListStates
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import tf2_ros
from actionlib import SimpleActionClient

# Since Moveit update to noetic, the plan() method returns a tuple where trajectory is indexed at 1.
# More info here: https://github.com/ros-planning/moveit/blob/master/MIGRATION.md
CONST_TUPLE_TRAJECTORY_INDEX = 1


class SrRobotCommanderException(Exception):
    def __init__(self, value):
        super().__init__(value)
        self._value = value

    def __str__(self):
        return repr(self._value)


class SrRobotCommander:
    """
    Base class for hand and arm commanders.
    """

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object.
        @param name - name of the MoveIt group.
        """
        self._name = name
        self._move_group_commander = MoveGroupCommander(name)
        self._robot_commander = RobotCommander()
        self._robot_name = self._robot_commander._r.get_robot_name()
        self.refresh_named_targets()
        self._warehouse_name_get_srv = rospy.ServiceProxy("get_robot_state", GetState)
        self._planning_scene = PlanningSceneInterface()
        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = rospy.Subscriber("joint_states", JointState, self._joint_states_callback,
                                                       queue_size=1)
        self._joints_position = {}
        self._joints_velocity = {}
        self._joints_effort = {}
        self._joints_state = None
        self._clients = {}
        self.__plan = None
        self._controllers = {}
        rospy.wait_for_service('compute_ik')
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._forward_k = rospy.ServiceProxy('compute_fk', GetPositionFK)
        controller_list_param = rospy.get_param("/move_group/controller_list")
        self._controllers = {item["name"]: item["joints"] for item in controller_list_param}
        self._set_up_action_client(self._controllers)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        threading.Thread(None, rospy.spin)

    @staticmethod
    def _is_trajectory_valid(trajectory, required_keys):
        if not isinstance(trajectory, list):
            rospy.logerr("Trajectory is not a list of waypoints")
            return False
        no_error = True
        for key in required_keys:
            if "|" in key:
                optional = key.split("|")
                if len(set(optional).intersection(set(trajectory[0].keys()))) == 0:
                    rospy.logerr(f"Trajectory is missing both of {optional} keys")
                    no_error = False
            else:
                if key not in list(trajectory[0].keys()):
                    rospy.logerr(f"Trajectory waypoint missing {key}")
                    no_error = False
        return no_error

    def set_planner_id(self, planner_id):
        """
        Sets the planner_id used for all future planning requests.
        @param planner_id - The string for the planner id, set to None to clear.
        """
        self._move_group_commander.set_planner_id(planner_id)

    def set_num_planning_attempts(self, num_planning_attempts):
        self._move_group_commander.set_num_planning_attempts(num_planning_attempts)

    def set_planning_time(self, seconds):
        """
        Specifies the amount of time to be used for motion planning.
        Some planning algorithms might require more time than others to compute a valid solution.
        """
        self._move_group_commander.set_planning_time(seconds)

    def get_moveit_robot_commander(self):
        return self._robot_commander

    def get_moveit_planning_scene(self):
        return self._planning_scene

    def get_move_group_commander(self):
        return self._move_group_commander

    def get_end_effector_pose_from_named_state(self, name):
        state = self._warehouse_name_get_srv(name, self._robot_name).state
        return self.get_end_effector_pose_from_state(state)

    def get_end_effector_pose_from_state(self, state):
        header = Header()
        fk_link_names = [self._move_group_commander.get_end_effector_link()]
        header.frame_id = self._move_group_commander.get_pose_reference_frame()
        response = self._forward_k(header, fk_link_names, state)
        return response.pose_stamped[0]

    def get_planning_frame(self):
        """
        @return - returns the name of the frame wrt which the motion planning is computed.
        """
        return self._move_group_commander.get_planning_frame()

    def set_pose_reference_frame(self, reference_frame):
        """
        Set the reference frame to assume for poses of end-effectors.
        @param reference_frame - name of the frame.
        """
        self._move_group_commander.set_pose_reference_frame(reference_frame)

    def get_group_name(self):
        return self._name

    def refresh_named_targets(self):
        self._srdf_names = self.get_srdf_names()
        self._warehouse_names = self.get_warehouse_names()

    def set_max_velocity_scaling_factor(self, value):
        """
        Set a scaling factor for optionally reducing the maximum joint velocity.
        @param value - Allowed values are in (0,1].
        """
        self._move_group_commander.set_max_velocity_scaling_factor(value)

    def set_max_acceleration_scaling_factor(self, value):
        """
        Set a scaling factor for optionally reducing the maximum joint acceleration.
        @param value - Allowed values are in (0,1].
        """
        self._move_group_commander.set_max_acceleration_scaling_factor(value)

    def allow_looking(self, value):
        """
        Enable/disable looking around for motion planning.
        @param value - boolean.
        """
        self._move_group_commander.allow_looking(value)

    def allow_replanning(self, value):
        """
        Enable/disable replanning in case new obstacles are detected while executing a plan.
        @param value - boolean.
        """
        self._move_group_commander.allow_replanning(value)

    def execute(self):
        """
        Executes the last plan made.
        @return - Success of execution.
        """
        is_executed = False
        if self.check_plan_is_valid():
            is_executed = self._move_group_commander.execute(self.__plan)
            self.__plan = None
        else:
            rospy.logwarn("No plans were made, not executing anything.")
        if not is_executed:
            rospy.logerr("Execution failed.")
        else:
            rospy.loginfo("Execution succeeded.")
        return is_executed

    def execute_plan(self, plan):
        """
        Executes a given plan.
        @param plan - RobotTrajectory msg that contains the trajectory to the set goal state.
        @return - Success of execution.
        """
        is_executed = False
        if self.check_given_plan_is_valid(plan):
            is_executed = self._move_group_commander.execute(plan)
            self.__plan = None
        else:
            rospy.logwarn("Plan is not valid, not executing anything.")
        if not is_executed:
            rospy.logerr("Execution failed.")
        else:
            rospy.loginfo("Execution succeeded.")
        return is_executed

    def move_to_joint_value_target(self, joint_states, wait=True, angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param wait - should method wait for movement end or not.
        @param angle_degrees - are joint_states in degrees or not.
        """
        joint_states_cpy = copy.deepcopy(joint_states)

        if angle_degrees:
            joint_states_cpy.update((joint, radians(i)) for joint, i in joint_states_cpy.items())
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_joint_value_target(joint_states_cpy)
        self._move_group_commander.go(wait=wait)

    def set_start_state_to_current_state(self):
        return self._move_group_commander.set_start_state_to_current_state()

    def plan_to_joint_value_target(self, joint_states, angle_degrees=False, custom_start_state=None):
        """
        Set target of the robot's links and plans.
        @param joint_states - dict with joint name and value.
        It can contain only joints values of which need to be changed.
        @param angle_degrees - are joint_states in degrees or not.
        @param custom_start_state - specify a start state different from the current state. This is a blocking method.
        @return - motion plan (RobotTrajectory msg) that contains the trajectory to the set goal state.
        """
        joint_states_cpy = copy.deepcopy(joint_states)
        if angle_degrees:
            joint_states_cpy.update((joint, radians(i)) for joint, i in joint_states_cpy.items())
        if custom_start_state is None:
            self._move_group_commander.set_start_state_to_current_state()
        else:
            self._move_group_commander.set_start_state(custom_start_state)
        self._move_group_commander.set_joint_value_target(joint_states_cpy)
        self.__plan = self._move_group_commander.plan()[CONST_TUPLE_TRAJECTORY_INDEX]
        return self.__plan

    def check_plan_is_valid(self):
        """
        Checks if current plan contains a valid trajectory
        """
        return self.__plan is not None and len(self.__plan.joint_trajectory.points) > 0

    @staticmethod
    def check_given_plan_is_valid(plan):
        """
        Checks if given plan contains a valid trajectory
        """
        return plan is not None and len(plan.joint_trajectory.points) > 0

    @staticmethod
    def evaluate_given_plan(plan):
        """
        Returns given plan quality calculated by a weighted sum of angles traveled by each of the joints, giving higher
        weights to the joints closer to the base of the robot, thus penalizing them as small movements of these joints
        will result in bigger movements of the end effector.
        Formula: PQ = sum_(i=0)^(n-1){w_i * abs(x_i - x_(i0)}, where:
        n - number of robot's joints,
        w - weight specified for each joint,
        x - joint's goal position,
        x_0 - joint's initial position.
        The lower the value, the better the plan.
        """
        if plan is None:
            return None
        num_of_joints = len(plan.joint_trajectory.points[0].positions)
        weights = numpy.array(sorted(range(1, num_of_joints + 1), reverse=True))
        plan_array = numpy.empty(shape=(len(plan.joint_trajectory.points), num_of_joints))
        for i, point in enumerate(plan.joint_trajectory.points):
            plan_array[i] = point.positions
        deltas = abs(numpy.diff(plan_array, axis=0))
        sum_deltas = numpy.sum(deltas, axis=0)
        sum_deltas_weighted = sum_deltas * weights
        plan_quality = float(numpy.sum(sum_deltas_weighted))
        return plan_quality

    def evaluate_plan(self):
        return self.evaluate_given_plan(self.__plan)

    @staticmethod
    def evaluate_plan_quality(plan_quality, good_threshold=20, medium_threshold=50):
        if plan_quality > medium_threshold:
            rospy.logwarn(f"Low plan quality! Value: {plan_quality}")
            return 'poor'
        if good_threshold < plan_quality < medium_threshold:
            rospy.loginfo(f"Medium plan quality. Value: {plan_quality}")
            return 'medium'
        rospy.loginfo(f"Good plan quality. Value: {plan_quality}")
        return 'good'

    def get_robot_name(self):
        return self._robot_name

    def named_target_in_srdf(self, name):
        return name in self._srdf_names

    def set_named_target(self, name):
        """
        Set a joint configuration by name.
        @param name - target's name which must correspond to a name defined in the srdf or in the mongo warehouse db.
        @return - bool to confirm that the target has been correctly set.
        """
        if name in self._srdf_names:
            self._move_group_commander.set_named_target(name)
            return True
        if name in self._warehouse_names:
            response = self._warehouse_name_get_srv(name, self._robot_name)
            active_names = self._move_group_commander.get_active_joints()
            joints = response.state.joint_state.name
            positions = response.state.joint_state.position
            joint_state = {}
            for name_index, this_name in enumerate(joints):
                if this_name in active_names:
                    joint_state[this_name] = positions[name_index]
            try:
                self._move_group_commander.set_joint_value_target(joint_state)
            except Exception as exception:
                rospy.loginfo(exception)
            return True
        rospy.logerr(f"Unknown named state '{name}'...")
        return False

    def get_named_target_joint_values(self, name):
        """
        Get the joint angles for targets specified by name.
        @param name - @param name - name of the target which must correspond to a name defined,
        either in the srdf or in the mongo warehouse database.
        @return - joint values of the named target.
        """
        output = {}
        if name in self._srdf_names:
            output = self._move_group_commander.get_named_target_values(str(name))
        elif name in self._warehouse_names:
            service = self._warehouse_name_get_srv(name, self._robot_name).state.joint_state
            for js_index, js_name in enumerate(service.name):
                if js_name in self._move_group_commander.get_joints():
                    output[js_name] = service.position[js_index]
        else:
            rospy.logerr(f"No target named {name}")
            return None
        return output

    def get_end_effector_link(self):
        return self._move_group_commander.get_end_effector_link()

    def get_current_pose(self, reference_frame=None):
        """
        Get the current pose of the end effector.
        @param reference_frame - The desired reference frame in which end effector pose should be returned.
        If none is passed, it will use the planning frame as reference.
        @return - geometry_msgs.msg.Pose() - current pose of the end effector.
        """
        if reference_frame is not None:
            try:
                trans = self.tf_buffer.lookup_transform(reference_frame,
                                                        self._move_group_commander.get_end_effector_link(),
                                                        rospy.Time(0), rospy.Duration(5.0))
                current_pose = geometry_msgs.msg.Pose()
                current_pose.position.x = trans.transform.translation.x
                current_pose.position.y = trans.transform.translation.y
                current_pose.position.z = trans.transform.translation.z
                current_pose.orientation.x = trans.transform.rotation.x
                current_pose.orientation.y = trans.transform.rotation.y
                current_pose.orientation.z = trans.transform.rotation.z
                current_pose.orientation.w = trans.transform.rotation.w
                return current_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,  # pylint: disable=E1101
                    tf2_ros.ExtrapolationException):  # pylint: disable=E1101
                rospy.logwarn(f"Couldn't get the pose from {self._move_group_commander.get_end_effector_link()}" +
                              f" in {reference_frame} reference frame")
            return None
        return self._move_group_commander.get_current_pose().pose

    def get_current_state(self):
        """
        Get the current joint state of the group being used.
        @return - a dictionary with the joint names as keys and current joint values.
        """
        joint_names = self._move_group_commander.get_active_joints()
        joint_values = self._move_group_commander.get_current_joint_values()
        return dict(zip(joint_names, joint_values))

    def get_current_state_bounded(self):
        """
        Get the current joint state of the group being used, enforcing that they are within each joint limits.
        @return - a dictionary with the joint names as keys and current joint values.
        """
        current = self._move_group_commander._g.get_current_state_bounded()  # pylint: disable=W0212
        names = self._move_group_commander._g.get_active_joints()  # pylint: disable=W0212
        output = {n: current[n] for n in names if n in current}
        return output

    def get_robot_state_bounded(self):
        return self._move_group_commander._g.get_current_state_bounded()  # pylint: disable=W0212

    def move_to_named_target(self, name, wait=True):
        """
        Set target of the robot's links and moves to it
        @param name - name of the target pose defined in SRDF
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        if self.set_named_target(name):
            self._move_group_commander.go(wait=wait)

    def plan_to_named_target(self, name, custom_start_state=None):
        """
        Set target of the robot's links and plans. This is a blocking method.
        @param name - name of the target pose defined in SRDF.
        @param custom_start_state - specify a start state different from the current state.
        @return - a motion plan (RobotTrajectory msg) that contains the trajectory to the named target.
        """
        if custom_start_state is None:
            self._move_group_commander.set_start_state_to_current_state()
        else:
            self._move_group_commander.set_start_state(custom_start_state)
        if self.set_named_target(name):
            self.__plan = self._move_group_commander.plan()[CONST_TUPLE_TRAJECTORY_INDEX]
        else:
            rospy.logwarn("Could not find named target, plan not generated")
            return False
        return True

    def get_warehouse_names(self):
        try:
            list_srv = rospy.ServiceProxy("list_robot_state", ListStates)
            return list_srv("", self._robot_name).states
        except rospy.ServiceException as exc:
            rospy.logwarn("Couldn't access warehouse: " + str(exc))
            return []

    def reset_plan(self):
        self.__plan = None

    def _set_plan(self, plan):
        self.__plan = plan

    def get_plan(self):
        return self.__plan

    def get_srdf_names(self):
        return self._move_group_commander.get_named_targets()

    def get_named_targets(self):
        """
        Get the complete list of named targets, from SRDF
        as well as warehouse poses if available.
        @return - list of strings containing names of targets.
        """
        return self._srdf_names + self._warehouse_names

    def get_joints_position(self):
        """
        Returns joints position.
        @return - dictionary with joints positions.
        """
        with self._joint_states_lock:
            return self._joints_position

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities.
        """
        with self._joint_states_lock:
            return self._joints_velocity

    def _get_joints_effort(self):
        """
        Returns joints effort.
        @return - dictionary with joints efforts.
        """
        with self._joint_states_lock:
            return self._joints_effort

    def get_joints_state(self):
        """
        Returns joints state
        @return - JointState message
        """
        with self._joint_states_lock:
            return self._joints_state

    def run_joint_trajectory(self, joint_trajectory):
        """
        Moves robot through all joint states with specified timeouts.
        @param joint_trajectory - JointTrajectory class object. Represents
        trajectory of the joints which would be executed.
        @return - Success of execution.
        """
        plan = RobotTrajectory()
        plan.joint_trajectory = joint_trajectory
        return self._move_group_commander.execute(plan)

    def make_named_trajectory(self, trajectory):
        """
        Makes joint value trajectory from specified by named poses (either from SRDF or from warehouse).
        @param trajectory - list of waypoints, each waypoint is a dict with
                            the following elements (n.b either name or joint_angles is required)
                          - name -> the name of the way point
                          - joint_angles -> a dict of joint names and angles
                          - interpolate_time -> time to move from last wp
                            OPTIONAL:
                          - pause_time -> time to wait at this wp
                          - degrees -> set to true if joint_angles is specified in degrees. Assumed false if absent.
        """
        joint_trajectory = JointTrajectory()
        if not self._is_trajectory_valid(trajectory, ["name|joint_angles", "interpolate_time"]):
            return joint_trajectory

        current = self.get_current_state_bounded()
        joint_names = list(current.keys())
        joint_trajectory.joint_names = joint_names
        start = JointTrajectoryPoint()
        start.positions = list(current.values())
        start.time_from_start = rospy.Duration.from_sec(0.001)
        joint_trajectory.points.append(start)
        time_from_start = 0.0
        for way_point in trajectory:
            joint_positions = None
            if 'name' in way_point.keys():
                joint_positions = self.get_named_target_joint_values(way_point['name'])
            elif 'joint_angles' in way_point.keys():
                joint_positions = copy.deepcopy(way_point['joint_angles'])
                if 'degrees' in way_point.keys() and way_point['degrees']:
                    for joint, angle in joint_positions.items():
                        joint_positions[joint] = radians(angle)
            if joint_positions is None:
                rospy.logerr("Invalid waypoint. Must contain valid name for named target or dict of joint angles.")
                return None
            new_positions = {}
            for name in joint_names:
                new_positions[name] = joint_positions[name] if name in joint_positions else current[name]
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = [new_positions[n] for n in joint_names]
            current = new_positions
            time_from_start += way_point['interpolate_time']
            trajectory_point.time_from_start = rospy.Duration.from_sec(time_from_start)
            joint_trajectory.points.append(trajectory_point)
            if 'pause_time' in way_point and way_point['pause_time'] > 0:
                extra = JointTrajectoryPoint()
                extra.positions = trajectory_point.positions
                time_from_start += way_point['pause_time']
                extra.time_from_start = rospy.Duration.from_sec(time_from_start)
                joint_trajectory.points.append(extra)
        return joint_trajectory

    def send_stop_trajectory_unsafe(self):
        """
        Sends a trajectory of all active joints at their current position. This stops the robot.
        """
        current = self.get_current_state_bounded()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = list(current.values())
        trajectory_point.time_from_start = rospy.Duration.from_sec(0.1)
        trajectory = JointTrajectory()
        trajectory.points.append(trajectory_point)
        trajectory.joint_names = list(current.keys())
        self.run_joint_trajectory_unsafe(trajectory)

    def run_named_trajectory_unsafe(self, trajectory, wait=False):
        """
        Moves robot through trajectory specified by named poses, either from
        SRDF or from warehouse. Runs trajectory directly via controller.
        @param trajectory - list of waypoints, each waypoint is a dict with the following elements:
                          - name -> the name of the way point
                          - interpolate_time -> time to move from last wp
                            OPTIONAL:
                          - pause_time -> time to wait at this wp
        """
        if self._is_trajectory_valid(trajectory, ['name|joint_angles', 'interpolate_time']):
            joint_trajectory = self.make_named_trajectory(trajectory)
            self.run_joint_trajectory_unsafe(joint_trajectory, wait)

    def run_named_trajectory(self, trajectory):
        """
        Moves robot through trajectory specified by named poses, either from
        SRDF or from warehouse. Runs trajectory via moveit.
        @param trajectory - list of waypoints, each waypoint is a dict with the following elements:
                          - name -> the name of the way point
                          - interpolate_time -> time to move from last wp
                            OPTIONAL:
                          - pause_time -> time to wait at this wp
        """
        if self._is_trajectory_valid(trajectory, ['name|joint_angles', 'interpolate_time']):
            joint_trajectory = self.make_named_trajectory(trajectory)
            self.run_joint_trajectory(joint_trajectory)

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it preserving the current orientation of the eef.
        @param xyz - new position of end-effector.
        @param end_effector_link - name of the end effector link.
        @param wait - should method wait for movement end or not.
        """
        pose = self._move_group_commander.get_current_pose()
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_pose_target(pose, end_effector_link)
        self._move_group_commander.go(wait=wait)

    def plan_to_position_target(self, xyz, end_effector_link="", custom_start_state=None):
        """
        Specify a target position for the end-effector and plans preserving the current orientation of end-effector.
        This is a blocking method.
        @param xyz - new position of end-effector.
        @param end_effector_link - name of the end effector link.
        @param custom_start_state - specify a start state different than the current state.
        """
        pose = self._move_group_commander.get_current_pose()
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        if custom_start_state is None:
            self._move_group_commander.set_start_state_to_current_state()
        else:
            self._move_group_commander.set_start_state(custom_start_state)
        self._move_group_commander.set_pose_target(pose, end_effector_link)
        self.__plan = self._move_group_commander.plan()[CONST_TUPLE_TRAJECTORY_INDEX]
        return self.__plan

    def move_to_pose_target(self, pose, end_effector_link="", wait=True):
        """
        Specify a target pose for the end-effector and moves to it
        @param pose - new pose of end-effector: a Pose message, a PoseStamped message or a list of 6 floats:
        [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw].
        @param end_effector_link - name of the end effector link.
        @param wait - should method wait for movement end or not.
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_pose_target(pose, end_effector_link)
        self._move_group_commander.go(wait=wait)

    def plan_to_pose_target(self, pose, end_effector_link="", alternative_method=False, custom_start_state=None):
        """
        Specify a target pose for the end-effector and plans. This is a blocking method.
        @param pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw].
        @param end_effector_link - name of the end effector link.
        @param alternative_method - use set_joint_value_target instead of set_pose_target.
        @param custom_start_state - specify a start state different from the current state.
        """
        if custom_start_state is None:
            self._move_group_commander.set_start_state_to_current_state()
        else:
            self._move_group_commander.set_start_state(custom_start_state)
        if alternative_method:
            self._move_group_commander.set_joint_value_target(pose, end_effector_link)
        else:
            self._move_group_commander.set_pose_target(pose, end_effector_link)
        self.__plan = self._move_group_commander.plan()[CONST_TUPLE_TRAJECTORY_INDEX]
        return self.__plan

    def _joint_states_callback(self, joint_state):
        """
        The callback function for the topic joint_states.
        It will store the received joint position, velocity and efforts
        information into dictionaries.
        @param joint_state - the message containing the joints data.
        """
        with self._joint_states_lock:
            self._joints_state = joint_state
            self._joints_position = dict(zip(joint_state.name, joint_state.position))
            self._joints_velocity = dict(zip(joint_state.name, joint_state.velocity))
            self._joints_effort = dict(zip(joint_state.name, joint_state.effort))

    def _set_up_action_client(self, controller_list):
        """
        Sets up an action client to communicate with the trajectory controller.
        """
        self._action_running = {}
        for controller_name in controller_list.keys():
            self._action_running[controller_name] = False
            service_name = controller_name + "/follow_joint_trajectory"
            self._clients[controller_name] = SimpleActionClient(service_name, FollowJointTrajectoryAction)
            if self._clients[controller_name].wait_for_server(timeout=rospy.Duration(4)) is False:
                rospy.logwarn(f'Failed to connect to action server ({service_name}) in 4 sec')

    def move_to_joint_value_target_unsafe(self, joint_states, time=0.002,
                                          wait=True, angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param time - time in s (counting from now) for the robot to reach the target (it needs to be greater than 0.0
        for it not to be rejected by the trajectory controller).
        @param wait - should method wait for movement end or not.
        @param angle_degrees - are joint_states in degrees or not.
        """
        goals = {}
        joint_states_cpy = copy.deepcopy(joint_states)
        if angle_degrees:
            joint_states_cpy.update((joint, radians(i)) for joint, i in joint_states_cpy.items())
        for controller in self._controllers:
            controller_joints = self._controllers[controller]
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.positions = []
            for key in joint_states_cpy.keys():
                if key in controller_joints:
                    goal.trajectory.joint_names.append(key)
                    point.positions.append(joint_states_cpy[key])
            point.time_from_start = rospy.Duration.from_sec(time)
            goal.trajectory.points = [point]
            goals[controller] = goal
        self._call_action(goals)
        if not wait:
            return
        for client, action_client in self._clients.items():
            if not self.action_is_running(client):
                continue
            if not action_client.wait_for_result():
                rospy.loginfo("Trajectory not completed")

    def action_is_running(self, controller=None):
        if controller is not None:
            return self._action_running[controller]
        for controller_running in self._action_running.values():
            if controller_running:
                return True
        return False

    def _action_done_cb(self, controller, terminal_state, result):
        self._action_running[controller] = False

    def _call_action(self, goals):
        for client, action_client in self._clients.items():
            if goals[client].trajectory.joint_names:
                self._action_running[client] = True
                action_client.send_goal(
                    goals[client], lambda terminal_state, result:
                    self._action_done_cb(client, terminal_state, result))  # pylint:disable=W0640

    def run_joint_trajectory_unsafe(self, joint_trajectory, wait=True):
        """
        Moves robot through all joint states with specified timeouts.
        @param joint_trajectory - JointTrajectory class object. Represent joints' trajectory which would be executed.
        @param wait - should method wait for movement end or not.
        """
        goals = {}
        for controller in self._controllers:
            controller_joints = self._controllers[controller]
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = copy.deepcopy(joint_trajectory)
            indices_of_joints_in_this_controller = []
            for i, joint in enumerate(joint_trajectory.joint_names):
                if joint in controller_joints:
                    indices_of_joints_in_this_controller.append(i)
            goal.trajectory.joint_names = [
                joint_trajectory.joint_names[i] for i in indices_of_joints_in_this_controller]
            for point in goal.trajectory.points:
                if point.positions:
                    point.positions = [point.positions[i] for i in indices_of_joints_in_this_controller]
                if point.velocities:
                    point.velocities = [point.velocities[i] for i in indices_of_joints_in_this_controller]
                if point.effort:
                    point.effort = [point.effort[i] for i in indices_of_joints_in_this_controller]
            goals[controller] = goal
        self._call_action(goals)
        if not wait:
            return
        for client, action_client in self._clients.items():
            if not self.action_is_running(client):
                continue
            if not action_client.wait_for_result():
                rospy.loginfo("Trajectory not completed")

    def plan_to_waypoints_target(self, waypoints, reference_frame=None,
                                 eef_step=0.005, jump_threshold=0.0, custom_start_state=None):
        """
        Specify a set of waypoints for the end-effector and plans.
        This is a blocking method.
        @param reference_frame - the reference frame in which the waypoints are given.
        @param waypoints - an array of poses of end-effector.
        @param eef_step - configurations are computed for every eef_step meters.
        @param jump_threshold - maximum distance in configuration space between consecutive points in the
        resulting path.
        @param custom_start_state - specify a start state different than the current state.
        @return - motion plan (RobotTrajectory msg) that contains the trajectory to the set wayapoints targets.
        """
        if custom_start_state is None:
            self._move_group_commander.set_start_state_to_current_state()
        else:
            self._move_group_commander.set_start_state(custom_start_state)
        old_frame = self._move_group_commander.get_pose_reference_frame()
        if reference_frame is not None:
            self.set_pose_reference_frame(reference_frame)
        self.__plan, fraction = self._move_group_commander.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        self.set_pose_reference_frame(old_frame)
        return self.__plan, fraction

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

    def move_to_trajectory_start(self, trajectory, wait=True):
        """
        Make and execute a plan from the current state to the first state in an pre-existing trajectory.
        @param trajectory - moveit_msgs/JointTrajectory.
        @param wait - Bool to specify if movement should block untill finished.
        """

        if len(trajectory.points) <= 0:
            rospy.logerr("Trajectory has no points in it, can't reverse...")
            return

        first_point = trajectory.points[0]
        end_state = dict(zip(trajectory.joint_names, first_point.positions))
        self.move_to_joint_value_target(end_state, wait=wait)

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

    def get_ik(self, target_pose, avoid_collisions=False, joint_states=None, ik_constraints=None):
        """
        Computes the inverse kinematics for a given pose. It returns a JointState.
        @param target_pose - A given pose of type PoseStamped.
        @param avoid_collisions - Find an IK solution that avoids collisions. By default, this is false.
        @param joint_states - initial joint configuration of type JointState from which the IK solution is computed.
        If set to None, the current joint state is retrieved automatically.
        @param ik_constraints - Set constraints of type Constraints for computing the IK solution.
        """
        service_request = PositionIKRequest()
        service_request.group_name = self._name
        service_request.ik_link_name = self._move_group_commander.get_end_effector_link()
        service_request.pose_stamped = target_pose
        service_request.timeout.secs = 1
        service_request.avoid_collisions = avoid_collisions
        if ik_constraints is not None:
            service_request.constraints = ik_constraints
        if joint_states is None:
            service_request.robot_state.joint_state = self.get_joints_state()
        else:
            service_request.robot_state.joint_state = joint_states

        try:
            resp = self._compute_ik(ik_request=service_request)
            # Check if error_code.val is SUCCESS=1
            if resp.error_code.val != 1:
                if resp.error_code.val == -10:
                    rospy.logerr("Unreachable point: Start state in collision")
                elif resp.error_code.val == -12:
                    rospy.logerr("Unreachable point: Goal state in collision")
                elif resp.error_code.val == -31:
                    rospy.logerr("Unreachable point: No IK solution")
                else:
                    rospy.logerr(f"Unreachable point (error: {resp.error_code})")
                return None

            if resp.solution.joint_state is not None:
                joint_state = resp.solution.joint_state
                active_joints = self._move_group_commander.get_active_joints()
                current_indices = [i for i, x in enumerate(joint_state.name)
                                   if any(thing in x for thing in active_joints)]
                current_names = [joint_state.name[i] for i in current_indices]
                current_positions = [joint_state.position[i] for i in current_indices]
                resp.solution.joint_state = dict(zip(current_names, current_positions))
            return resp.solution.joint_state

        except rospy.ServiceException as exception:
            rospy.logerr(f"Service call failed: {exception}")
            return None

    def move_to_pose_value_target_unsafe(self, target_pose, avoid_collisions=False,
                                         time=0.002, wait=True, ik_constraints=None):
        """
        Specify a target pose for the end-effector and moves to it.
        @param target_pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
        of 7 floats [x, y, z, qx, qy, qz, qw].
        @param avoid_collisions - Find an IK solution that avoids collisions. By default, this is false.
        @param time - time in s (counting from now) for the robot to reach the
        target (it needs to be greater than 0.0 for it not to be rejected by
        the trajectory controller).
        @param wait - should method wait for movement end or not.
        @param ik_constraints - Set constraints of type Constraints for computing the IK solution.
        """
        joint_states = self.get_ik(target_pose, avoid_collisions, ik_constraints=ik_constraints)
        if joint_states is not None:
            self.move_to_joint_value_target_unsafe(joint_states, time=time, wait=wait)
