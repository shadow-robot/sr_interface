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
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, \
    RobotTeachModeResponse

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListStates
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from math import radians

from sr_utilities.hand_finder import HandFinder

from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header

import tf2_ros
import copy
import rospkg


class SrRobotCommander(object):
    """
    Base class for hand and arm commanders
    """

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object
        @param name - name of the MoveIt group
        """
        self._name = name
        self._move_group_commander = MoveGroupCommander(name)

        self._robot_commander = RobotCommander()

        self._robot_name = self._robot_commander._r.get_robot_name()

        self.refresh_named_targets()

        self._warehouse_name_get_srv = rospy.ServiceProxy("get_robot_state",
                                                          GetState)
        self._planning_scene = PlanningSceneInterface()

        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = \
            rospy.Subscriber("joint_states", JointState,
                             self._joint_states_callback, queue_size=1)
        self._joints_position = {}
        self._joints_velocity = {}
        self._joints_effort = {}
        self._clients = {}
        self.__plan = None

        self._controllers = {}

        rospy.wait_for_service('compute_ik')
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._forward_k = rospy.ServiceProxy('compute_fk', GetPositionFK)

        controller_list_param = rospy.get_param("/move_group/controller_list")

        # create dictionary with name of controllers and corresponding joints
        self._controllers = {item["name"]: item["joints"] for item in controller_list_param}

        self._set_up_action_client(self._controllers)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        threading.Thread(None, rospy.spin)

    def set_planner_id(self, planner_id):
        self._move_group_commander.set_planner_id(planner_id)

    def set_num_planning_attempts(self, num_planning_attempts):
        self._move_group_commander.set_num_planning_attempts(num_planning_attempts)

    def set_planning_time(self, seconds):
        self._move_group_commander.set_planning_time(seconds)

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
        return self._move_group_commander.get_planning_frame()

    def set_pose_reference_frame(self, reference_frame):
        self._move_group_commander.set_pose_reference_frame(reference_frame)

    def get_group_name(self):
        return self._name

    def refresh_named_targets(self):
        self._srdf_names = self.__get_srdf_names()
        self._warehouse_names = self.__get_warehouse_names()

    def set_max_velocity_scaling_factor(self, value):
        self._move_group_commander.set_max_velocity_scaling_factor(value)

    def set_max_acceleration_scaling_factor(self, value):
        self._move_group_commander.set_max_acceleration_scaling_factor(value)

    def allow_looking(self, value):
        self._move_group_commander.allow_looking(value)

    def allow_replanning(self, value):
        self._move_group_commander.allow_replanning(value)

    def execute(self):
        """
        Executes the last plan made.
        """
        if self.check_plan_is_valid():
            self._move_group_commander.execute(self.__plan)
            self.__plan = None
        else:
            rospy.logwarn("No plans were made, not executing anything.")

    def execute_plan(self, plan):
        if self.check_given_plan_is_valid(plan):
            self._move_group_commander.execute(plan)
            self.__plan = None
        else:
            rospy.logwarn("Plan is not valid, not executing anything.")

    def move_to_joint_value_target(self, joint_states, wait=True,
                                   angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param wait - should method wait for movement end or not
        @param angle_degrees - are joint_states in degrees or not
        """
        joint_states_cpy = copy.deepcopy(joint_states)

        if angle_degrees:
            joint_states_cpy.update((joint, radians(i))
                                    for joint, i in joint_states_cpy.items())
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_joint_value_target(joint_states_cpy)
        self._move_group_commander.go(wait=wait)

    def plan_to_joint_value_target(self, joint_states, angle_degrees=False):
        """
        Set target of the robot's links and plans.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param angle_degrees - are joint_states in degrees or not
        This is a blocking method.
        """
        joint_states_cpy = copy.deepcopy(joint_states)

        if angle_degrees:
            joint_states_cpy.update((joint, radians(i))
                                    for joint, i in joint_states_cpy.items())
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_joint_value_target(joint_states_cpy)
        self.__plan = self._move_group_commander.plan()

    def check_plan_is_valid(self):
        """
        Checks if current plan contains a valid trajectory
        """
        return (self.__plan is not None and len(self.__plan.joint_trajectory.points) > 0)

    def check_given_plan_is_valid(self, plan):
        """
        Checks if given plan contains a valid trajectory
        """
        return (plan is not None and len(plan.joint_trajectory.points) > 0)

    def get_robot_name(self):
        return self._robot_name

    def named_target_in_srdf(self, name):
        return name in self._srdf_names

    def set_named_target(self, name):
        if name in self._srdf_names:
            self._move_group_commander.set_named_target(name)
        elif (name in self._warehouse_names):
            response = self._warehouse_name_get_srv(name, self._robot_name)

            active_names = self._move_group_commander._g.get_active_joints()
            joints = response.state.joint_state.name
            positions = response.state.joint_state.position
            js = {}

            for n, this_name in enumerate(joints):
                if this_name in active_names:
                    js[this_name] = positions[n]
            self._move_group_commander.set_joint_value_target(js)
        else:
            rospy.logerr("Unknown named state '%s'..." % name)
            return False
        return True

    def get_named_target_joint_values(self, name):
        output = dict()

        if (name in self._srdf_names):
            output = self._move_group_commander.\
                           _g.get_named_target_values(str(name))

        elif (name in self._warehouse_names):
            js = self._warehouse_name_get_srv(
                name, self._robot_name).state.joint_state

            for x, n in enumerate(js.name):
                if n in self._move_group_commander._g.get_joints():
                    output[n] = js.position[x]

        else:
            rospy.logerr("No target named %s" % name)

            return None

        return output

    def get_end_effector_link(self):
        return self._move_group_commander.get_end_effector_link()

    def get_current_pose(self, reference_frame=None):
        """
        Get the current pose of the end effector.
        @param reference_frame - The desired reference frame in which end effector pose should be returned.
        If none is passed, it will use the planning frame as reference.
        @return geometry_msgs.msg.Pose() - current pose of the end effector
        """
        if reference_frame is not None:
            try:
                trans = self.tf_buffer.lookup_transform(reference_frame,
                                                        self._move_group_commander.get_end_effector_link(),
                                                        rospy.Time(0),
                                                        rospy.Duration(5.0))
                current_pose = geometry_msgs.msg.Pose()
                current_pose.position.x = trans.transform.translation.x
                current_pose.position.y = trans.transform.translation.y
                current_pose.position.z = trans.transform.translation.z
                current_pose.orientation.x = trans.transform.rotation.x
                current_pose.orientation.y = trans.transform.rotation.y
                current_pose.orientation.z = trans.transform.rotation.z
                current_pose.orientation.w = trans.transform.rotation.w
                return current_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Couldn't get the pose from " + self._move_group_commander.get_end_effector_link() +
                              " in " + reference_frame + " reference frame")
            return None
        else:
            return self._move_group_commander.get_current_pose().pose

    def get_current_state(self):
        """
        Get the current joint state of the group being used.
        @return a dictionary with the joint names as keys and current joint values
        """
        joint_names = self._move_group_commander._g.get_active_joints()
        joint_values = self._move_group_commander._g.get_current_joint_values()

        return dict(zip(joint_names, joint_values))

    def get_current_state_bounded(self):
        """
        Get the current joint state of the group being used, enforcing that they are within each joint limits.
        @return a dictionary with the joint names as keys and current joint values
        """
        current = self._move_group_commander._g.get_current_state_bounded()
        names = self._move_group_commander._g.get_active_joints()
        output = {n: current[n] for n in names if n in current}

        return output

    def get_robot_state_bounded(self):
        return self._move_group_commander._g.get_current_state_bounded()

    def move_to_named_target(self, name, wait=True):
        """
        Set target of the robot's links and moves to it
        @param name - name of the target pose defined in SRDF
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        if self.set_named_target(name):
            self._move_group_commander.go(wait=wait)

    def plan_to_named_target(self, name):
        """
        Set target of the robot's links and plans
        This is a blocking method.
        @param name - name of the target pose defined in SRDF
        """
        self._move_group_commander.set_start_state_to_current_state()
        if self.set_named_target(name):
            self.__plan = self._move_group_commander.plan()

    def __get_warehouse_names(self):
        try:
            list_srv = rospy.ServiceProxy("list_robot_states", ListStates)
            return list_srv("", self._robot_name).states

        except rospy.ServiceException as exc:
            rospy.logwarn("Couldn't access warehouse: " + str(exc))
            return list()

    def _reset_plan(self):
        self.__plan = None

    def _set_plan(self, plan):
        self.__plan = plan

    def __get_srdf_names(self):
        return self._move_group_commander._g.get_named_targets()

    def get_named_targets(self):
        """
        Get the complete list of named targets, from SRDF
        as well as warehouse poses if available.
        @return list of strings containing names of targets.
        """
        return self._srdf_names + self._warehouse_names

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

    def make_named_trajectory(self, trajectory):
        """
        Makes joint value trajectory from specified by named poses (either from
        SRDF or from warehouse)
        @param trajectory - list of waypoints, each waypoint is a dict with
                            the following elements (n.b either name or joint_angles is required)
                            - name -> the name of the way point
                            - joint_angles -> a dict of joint names and angles
                            - interpolate_time -> time to move from last wp
                            - pause_time -> time to wait at this wp
                            - degrees -> set to true if joint_angles is specified in degrees. Assumed false if absent.
        """
        current = self.get_current_state_bounded()

        joint_trajectory = JointTrajectory()
        joint_names = current.keys()
        joint_trajectory.joint_names = joint_names

        start = JointTrajectoryPoint()
        start.positions = current.values()
        start.time_from_start = rospy.Duration.from_sec(0.001)
        joint_trajectory.points.append(start)

        time_from_start = 0.0

        for wp in trajectory:

            joint_positions = None
            if 'name' in wp.keys():
                joint_positions = self.get_named_target_joint_values(wp['name'])
            elif 'joint_angles' in wp.keys():
                joint_positions = copy.deepcopy(wp['joint_angles'])
                if 'degrees' in wp.keys() and wp['degrees']:
                    for joint, angle in joint_positions.iteritems():
                        joint_positions[joint] = radians(angle)

            if joint_positions is None:
                rospy.logerr("Invalid waypoint. Must contain valid name for named target or dict of joint angles.")
                return None

            new_positions = {}

            for n in joint_names:
                new_positions[n] = joint_positions[n] if n in joint_positions else current[n]

            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = [new_positions[n] for n in joint_names]

            current = new_positions

            time_from_start += wp['interpolate_time']
            trajectory_point.time_from_start = rospy.Duration.from_sec(time_from_start)
            joint_trajectory.points.append(trajectory_point)

            if 'pause_time' in wp and wp['pause_time'] > 0:
                extra = JointTrajectoryPoint()
                extra.positions = trajectory_point.positions
                time_from_start += wp['pause_time']
                extra.time_from_start = rospy.Duration.from_sec(time_from_start)
                joint_trajectory.points.append(extra)

        return joint_trajectory

    def send_stop_trajectory_unsafe(self):
        """
        Sends a trajectory of all active joints at their current position.
        This stops the robot.
        """

        current = self.get_current_state_bounded()

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = current.values()
        trajectory_point.time_from_start = rospy.Duration.from_sec(0.1)

        trajectory = JointTrajectory()
        trajectory.points.append(trajectory_point)
        trajectory.joint_names = current.keys()

        self.run_joint_trajectory_unsafe(trajectory)

    def run_named_trajectory_unsafe(self, trajectory, wait=False):
        """
        Moves robot through trajectory specified by named poses, either from
        SRDF or from warehouse. Runs trajectory directly via contoller.
        @param trajectory - list of waypoints, each waypoint is a dict with
                            the following elements:
                            - name -> the name of the way point
                            - interpolate_time -> time to move from last wp
                            - pause_time -> time to wait at this wp
        """
        joint_trajectory = self.make_named_trajectory(trajectory)
        if joint_trajectory is not None:
            self.run_joint_trajectory_unsafe(joint_trajectory, wait)

    def run_named_trajectory(self, trajectory):
        """
        Moves robot through trajectory specified by named poses, either from
        SRDF or from warehouse. Runs trajectory via moveit.
        @param trajectory - list of waypoints, each waypoint is a dict with
                            the following elements:
                            - name -> the name of the way point
                            - interpolate_time -> time to move from last wp
                            - pause_time -> time to wait at this wp
        """
        joint_trajectory = self.make_named_trajectory(trajectory)
        if joint_trajectory is not None:
            self.run_joint_trajectory(joint_trajectory)

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self._move_group_commander.go(wait=wait)

    def plan_to_position_target(self, xyz, end_effector_link=""):
        """
        Specify a target position for the end-effector and plans.
        This is a blocking method.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        """
        self._move_group_commander.set_start_state_to_current_state()
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self.__plan = self._move_group_commander.plan()

    def move_to_pose_target(self, pose, end_effector_link="", wait=True):
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

    def plan_to_pose_target(self, pose, end_effector_link="", alternative_method=False):
        """
        Specify a target pose for the end-effector and plans.
        This is a blocking method.
        @param pose - new pose of end-effector: a Pose message, a PoseStamped
        message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
        of 7 floats [x, y, z, qx, qy, qz, qw]
        @param end_effector_link - name of the end effector link
        @param alternative_method - use set_joint_value_target instead of set_pose_target
        """
        self._move_group_commander.set_start_state_to_current_state()
        if alternative_method:
            self._move_group_commander.set_joint_value_target(pose, end_effector_link)
        else:
            self._move_group_commander.set_pose_target(pose, end_effector_link)
        self.__plan = self._move_group_commander.plan()
        return self.__plan

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

    def _set_up_action_client(self, controller_list):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        self._action_running = {}

        for controller_name in controller_list.keys():
            self._action_running[controller_name] = False
            self._clients[controller_name] = SimpleActionClient(controller_name+"/follow_joint_trajectory",
                                                                FollowJointTrajectoryAction)
            if self._clients[controller_name].wait_for_server(timeout=rospy.Duration(4)) is False:
                rospy.logfatal("Failed to connect to action server in 4 sec")
                raise Exception("Failed to connect to action server in 4 sec")

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
        goals = {}
        joint_states_cpy = copy.deepcopy(joint_states)

        if angle_degrees:
            joint_states_cpy.update((joint, radians(i))
                                    for joint, i in joint_states_cpy.items())

        for controller in self._controllers:
            controller_joints = self._controllers[controller]
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.positions = []

            for x in joint_states_cpy.keys():
                if x in controller_joints:
                    goal.trajectory.joint_names.append(x)
                    point.positions.append(joint_states_cpy[x])

            point.time_from_start = rospy.Duration.from_sec(time)

            goal.trajectory.points = [point]

            goals[controller] = goal

        self._call_action(goals)

        if not wait:
            return

        for i in self._clients.keys():
            if not self._clients[i].wait_for_result():
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
        for client in self._clients:
            self._action_running[client] = True
            self._clients[client].send_goal(
                goals[client], lambda terminal_state, result: self._action_done_cb(client, terminal_state, result))

    def run_joint_trajectory_unsafe(self, joint_trajectory, wait=True):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents
        trajectory of the joints which would be executed.
        @param wait - should method wait for movement end or not
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

        for i in self._clients.keys():
            if not self._clients[i].wait_for_result():
                rospy.loginfo("Trajectory not completed")

    def plan_to_waypoints_target(self, waypoints, reference_frame=None, eef_step=0.005, jump_threshold=0.0):
        """
        Specify a set of waypoints for the end-effector and plans.
        This is a blocking method.
        @param reference_frame - the reference frame in which the waypoints are given
        @param waypoints - an array of poses of end-effector
        @param eef_step - configurations are computed for every eef_step meters
        @param jump_threshold - maximum distance in configuration space between consecutive points in the resulting path
        """
        old_frame = self._move_group_commander.get_pose_reference_frame()
        if reference_frame is not None:
            self.set_pose_reference_frame(reference_frame)
        (self.__plan, fraction) = self._move_group_commander.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        self.set_pose_reference_frame(old_frame)

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
        Make and execute a plan from the current state to the first state in an pre-existing trajectory
        @param trajectory - moveit_msgs/JointTrajectory
        @param wait - Bool to specify if movement should block untill finished.
        """

        if len(trajectory.points) <= 0:
            rospy.logerr("Trajectory has no points in it, can't reverse...")
            return None

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

    def get_ik(self, target_pose, avoid_collisions=False):
        """
        Computes the inverse kinematics for a given pose. It returns a JointState
        @param target_pose - A given pose of type PoseStamped
        @param avoid_collisions - Find an IK solution that avoids collisions. By default, this is false
        """
        service_request = PositionIKRequest()
        service_request.group_name = self._name
        service_request.ik_link_name = self._move_group_commander.get_end_effector_link()
        service_request.pose_stamped = target_pose
        service_request.timeout.secs = 0.005
        service_request.avoid_collisions = avoid_collisions

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
                    rospy.logerr("Unreachable point (error: %s)" % resp.error_code)
                return
            else:
                return resp.solution.joint_state

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

    def move_to_pose_value_target_unsafe(self, target_pose,  avoid_collisions=False, time=0.002, wait=True):
        joint_state = self.get_ik(target_pose, avoid_collisions)
        if joint_state is not None:
            active_joints = self._move_group_commander.get_active_joints()
            current_indices = [i for i, x in enumerate(joint_state.name) if any(thing in x for thing in active_joints)]
            current_names = [joint_state.name[i] for i in current_indices]
            current_positions = [joint_state.position[i] for i in current_indices]
            state_as_dict = dict(zip(current_names, current_positions))
            self.move_to_joint_value_target_unsafe(state_as_dict, time=time, wait=wait)
