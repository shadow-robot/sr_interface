#!/usr/bin/env python

import rospy
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState, MoveGroupActionResult
from visualization_msgs.msg import Marker, MarkerArray
from tabulate import tabulate
import time
from copy import deepcopy

PKG = "sr_moveit_planner_benchmarking"


class TestPlanners(object):
    def __init__(self, planning_attempts, planner_id):
        group_id = "right_arm"
        rospy.init_node('moveit_test_planners', anonymous=True)
        self.planner_id = planner_id
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(group_id)
        self._marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10, latch=True)
        self._planning_time_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult,
                                                   self._check_computation_time)
        rospy.sleep(1)

        self.group.set_planner_id(self.planner_id)
        self.group.set_num_planning_attempts(planning_attempts)
        self._comp_time = []

        self.planner_data = []

        self.test_goal_1()
        rospy.sleep(3)
        self.test_goal_2()
        rospy.sleep(3)
        self.test_goal_3()
        rospy.sleep(3)

    def _add_markers(self, point, text1, point_2, text2):
        # add marker for start and goal pose of EE
        marker_array = MarkerArray()
        marker_1 = Marker()
        marker_1.header.frame_id = "world"
        marker_1.header.stamp = rospy.Time.now()
        marker_1.type = Marker.SPHERE
        marker_1.scale.x = 0.04
        marker_1.scale.y = 0.04
        marker_1.scale.z = 0.04
        marker_1.lifetime = rospy.Duration()

        marker_2 = deepcopy(marker_1)

        marker_1.color.g = 0.5
        marker_1.color.a = 1.0
        marker_1.id = 0
        marker_1.pose.position.x = point[0]
        marker_1.pose.position.y = point[1]
        marker_1.pose.position.z = point[2]
        marker_2.color.r = 0.5
        marker_2.color.a = 1.0
        marker_2.id = 1
        marker_2.pose.position.x = point_2[0]
        marker_2.pose.position.y = point_2[1]
        marker_2.pose.position.z = point_2[2]

        marker_3 = Marker()
        marker_3.header.frame_id = "world"
        marker_3.header.stamp = rospy.Time.now()
        marker_3.type = Marker.TEXT_VIEW_FACING
        marker_3.scale.z = 0.10
        marker_3.lifetime = rospy.Duration()

        marker_4 = deepcopy(marker_3)

        marker_3.text = text1
        marker_3.id = 2
        marker_3.color.g = 0.5
        marker_3.color.a = 1.0
        marker_3.pose.position.x = point[0]
        marker_3.pose.position.y = point[1]
        marker_3.pose.position.z = point[2] + 0.15
        marker_4.text = text2
        marker_4.id = 3
        marker_4.color.r = 0.5
        marker_4.color.a = 1.0
        marker_4.pose.position.x = point_2[0]
        marker_4.pose.position.y = point_2[1]
        marker_4.pose.position.z = point_2[2] + 0.15

        marker_array.markers = [marker_1, marker_2, marker_3, marker_4]

        self._marker_pub.publish(marker_array)
        rospy.sleep(1)

    def _plan_joints(self, joints, test_name):
        # plan to joint target and determine success
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0:6] = joints[0:6]
        self.group.set_joint_value_target(group_variable_values)

        plan = self.group.plan()
        plan_time = "N/A"
        total_joint_rotation = "N/A"
        comp_time = "N/A"

        plan_success = self._check_plan_success(plan)
        if plan_success:
            plan_time = self._check_plan_time(plan)
            total_joint_rotation = self._check_plan_total_rotation(plan)
            while not self._comp_time:
                time.sleep(0.5)
            comp_time = self._comp_time.pop(0)
        self.planner_data.append([self.planner_id, test_name, str(plan_success), plan_time, total_joint_rotation,
                                  comp_time])

    @staticmethod
    def _check_plan_success(plan):
        if len(plan.joint_trajectory.points) > 0:
            return True
        else:
            return False

    @staticmethod
    def _check_plan_time(plan):
        # find duration of successful plan
        number_of_points = len(plan.joint_trajectory.points)
        time = plan.joint_trajectory.points[number_of_points - 1].time_from_start.to_sec()
        return time

    @staticmethod
    def _check_plan_total_rotation(plan):
        # find total joint rotation in successful trajectory
        angles = [0, 0, 0, 0, 0, 0]
        number_of_points = len(plan.joint_trajectory.points)
        for i in range(number_of_points - 1):
            angles_temp = [abs(x - y) for x, y in zip(plan.joint_trajectory.points[i + 1].positions,
                                                      plan.joint_trajectory.points[i].positions)]
            angles = [x + y for x, y in zip(angles, angles_temp)]

        total_angle_change = sum(angles)
        return total_angle_change

    def _check_computation_time(self, msg):
        # get computation time for successful plan to be found
        if msg.status.status == 3:
            self._comp_time.append(msg.result.planning_time)

    def _check_path_length(self, plan):
        # find distance travelled by end effector
        # not yet implemented
        return

    def test_goal_1(self):
        marker_position_1 = [1.25, 0, 0.3]
        marker_position_2 = [-0.15, 0.73, 0.36]
        self._add_markers(marker_position_1, "Start test \n sequence", marker_position_2, "Goal")

        # Start planning in a given joint position
        joints = [0.11, 0.00, -0.93, -2.22, -1.71, -1.68]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)
        joints = [1.55, -1.15, 2.01, 2.39, -1.55, -1.58]
        self._plan_joints(joints, "Test 1")

    def test_goal_2(self):
        self.group.clear_pose_targets()
        # Start planning in a given joint position
        joints = [2.0124379710802414, -0.6395552894291645, 1.8272080603542877, 2.0191359302159064,
                  -1.3904105764847612, -3.0564905988137565]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)
        joints = [2.6752085499210883, -0.8522038622523826, 1.201729214706459, 2.821729335878766,
                  -1.7238947911545262, -1.6146325266562542]
        self._plan_joints(joints, "Test 2")

        marker_position_1 = [-0.51, 0.65, 0.07]
        marker_position_2 = [-0.99, 0.33, 0.50]
        self._add_markers(marker_position_1, "Start", marker_position_2, "Goal")

    def test_goal_3(self):
        self.group.clear_pose_targets()
        # Start planning in a given joint position
        joints = [-1.4888676313634834, -2.032021937909764, 2.0806817862208606, -2.7766676701654585,
                  -1.5224358139380132, 3.1386020362120806]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)
        joints = [-0.3873561657737919, -1.4454237975422544, 1.7388717638919942, -3.0023296843235188,
                  -2.5803362501170866, -2.7518032611540217]
        self._plan_joints(joints, "Test 3")

        marker_position_1 = [0.20, -0.41, 0.72]
        marker_position_2 = [0.70, -0.19, 0.65]
        self._add_markers(marker_position_1, "Start", marker_position_2, "Goal")


def table_output(plan_data):
    # display test metrics in table
    row_titles = ["Planner", "Plan name", "Plan succeeded", "Time of plan", "Total angle change", "Computation time"]

    print(tabulate(plan_data, headers=row_titles, tablefmt='orgtbl'))


def main(num, list):
    if list == "stomp":
        planner_list = ["STOMP"]
    elif list == "sbpl":
        planner_list = ["AnytimeD*"]
    else:
        planner_list = ["BKPIECEkConfigDefault", "ESTkConfigDefault", "KPIECEkConfigDefault", "LBKPIECEkConfigDefault",
                        "PRMkConfigDefault", "RRTkConfigDefault", "SBLkConfigDefault", "PRMstarkConfigDefault",
                        "RRTstarkConfigDefault"]
    test_data = []

    for plan in planner_list:
        test_data += TestPlanners(num, plan).planner_data

    table_output(test_data)


if __name__ == "__main__":
    replanning = int(sys.argv[1])
    planner = sys.argv[2]
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        time.sleep(0.5)

    main(replanning, planner)
    rospy.spin()
