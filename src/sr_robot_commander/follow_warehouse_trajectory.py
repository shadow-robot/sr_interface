#!/usr/bin/env python

import rospy

from moveit_commander import RobotCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListStates
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import GetPositionFK
from sr_robot_msgs.srv import PlanTrajectoryFromList
from sr_robot_msgs.srv import PlanTrajectoryFromPrefix
from sr_robot_msgs.srv import ExecutePlannedTrajectory

from std_msgs.msg import Header
import geometry_msgs.msg


class WarehousePlanner(object):
    def __init__(self):
        rospy.init_node('moveit_warehouse_planner', anonymous=True)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        rospy.sleep(2)
        group_id = rospy.get_param("~arm_group_name")
        self.eef_step = rospy.get_param("~eef_step", 0.01)
        self.jump_threshold = rospy.get_param("~jump_threshold", 1000)

        self.group = MoveGroupCommander(group_id)
        self._add_ground()
        self._robot_name = self.robot._r.get_robot_name()
        self._robot_link = self.group.get_end_effector_link()
        self._robot_frame = self.group.get_pose_reference_frame()

        self._min_wp_fraction = rospy.get_param("~min_waypoint_fraction", 0.9)

        rospy.sleep(4)
        rospy.loginfo("Waiting for warehouse services...")
        rospy.wait_for_service('moveit_warehouse_services/list_robot_states')
        rospy.wait_for_service('moveit_warehouse_services/get_robot_state')
        rospy.wait_for_service('moveit_warehouse_services/has_robot_state')

        rospy.wait_for_service('/compute_fk')
        self._list_states = rospy.ServiceProxy(
            'moveit_warehouse_services/list_robot_states', ListStates)
        self._get_state = rospy.ServiceProxy(
            'moveit_warehouse_services/get_robot_state', GetState)
        self._has_state = rospy.ServiceProxy(
            'moveit_warehouse_services/has_robot_state', HasState)
        self._forward_k = rospy.ServiceProxy(
            'compute_fk', GetPositionFK)
        rospy.loginfo("Service proxies connected")

        self._tr_frm_list_srv = rospy.Service('plan_trajectory_from_list',
                                              PlanTrajectoryFromList,
                                              self._plan_from_list_cb)

        self._tr_frm_prfx_srv = rospy.Service('plan_trajectory_from_prefix',
                                              PlanTrajectoryFromPrefix,
                                              self._plan_from_prefix_cb)

        self._execute_tr_srv = rospy.Service('execute_planned_trajectory',
                                             ExecutePlannedTrajectory,
                                             self._execute_plan_cb)

        self.__plan = None

    def get_waypoint_names_by_prefix(self, prefix):
        regex = "^" + str(prefix) + ".*"
        waypoint_names = self._list_states(regex, self._robot_name).states
        return waypoint_names

    def get_pose_from_state(self, state):
        header = Header()
        fk_link_names = [self._robot_link]
        header.frame_id = self._robot_frame
        response = self._forward_k(header, fk_link_names, state)
        return response.pose_stamped[0].pose

    def get_cartesian_waypoints(self, waypoint_names):
        waypoints = []
        waypoints.append(self.group.get_current_pose().pose)
        for name in waypoint_names:
            if self._has_state(name, self._robot_name).exists:
                robot_state = self._get_state(name, "").state
                waypoints.append(self.get_pose_from_state(robot_state))
            else:
                rospy.logerr("Waypoint %s doesn't exist for robot %s.", name,
                             self._robot_name)
        return waypoints

    def _add_ground(self):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()

        p.pose.position.x = 0
        p.pose.position.y = 0
        # offset such that the box is below the dome
        p.pose.position.z = -0.12
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        self.scene.add_box("ground", p, (3, 3, 0.1))
        rospy.sleep(1)

    def plan_from_filter(self, prefix):
        waypoint_names = self.get_waypoint_names_by_prefix(prefix)
        waypoint_names.sort()

        if 0 == len(waypoint_names):
            rospy.logerr("No waypoints found for robot %s with prefix %s. " +
                         "Can't make trajectory :(",
                         self._robot_name, str(prefix))
            return False
        rospy.loginfo("Creating trajectory with %d waypoints selected by " +
                      "prefix %s.", len(waypoint_names), str(prefix))
        return self.plan_from_list(waypoint_names)

    def plan_from_list(self, waypoint_names):
        self.group.clear_pose_targets()
        waypoints = self.get_cartesian_waypoints(waypoint_names)
        if len(waypoints) != len(waypoint_names) + 1:
            # +1 because current position is used as first waypiont.
            rospy.logerr("Not all waypoints existed, not executing.")
            return False
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, self.eef_step, self.jump_threshold)

        if fraction < self._min_wp_fraction:
            rospy.logerr("Only managed to generate trajectory through" +
                         "%f of waypoints. Not executing", fraction)
            return False

        self.__plan = plan
        return True

    def _plan_from_list_cb(self, request):
        # check all exist
        self.__plan = None
        rospy.loginfo("Creating trajectory from points given: %s",
                      ",".join(request.waypoint_names))
        return self.plan_from_list(request.waypoint_names)

    def _plan_from_prefix_cb(self, request):
        self.__plan = None
        rospy.loginfo("Creating trajectory from points filtered by prefix %s",
                      request.prefix)
        return self.plan_from_filter(request.prefix)

    def _execute_plan_cb(self, request):
        if self.__plan is None:
            rospy.logerr("No plan stored!")
            return False
        rospy.loginfo("Executing stored plan")
        response = self.group.execute(self.__plan)
        self.__plan = None
        return response

if __name__ == "__main__":
    sf = WarehousePlanner()
    rospy.spin()
