#!/usr/bin/env python

import rospy
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState
from visualization_msgs.msg import Marker
from tabulate import tabulate
from test_planners import TestPlanners
import time
from unittest import TestCase

PKG = "sr_moveit_planner_testsuite"


class TestPlannersComplex(TestPlanners):

    def __init__(self, planning_attempts, planner_id):
        # self.planner_id = str(sys.argv[1])
        # planning_attempts = int(sys.argv[2])
        group_id = "right_arm"
        rospy.init_node('moveit_test_planners', anonymous=True)
        self.planner_id = planner_id
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(group_id)
        rospy.sleep(1)

        self.group.set_planner_id(self.planner_id)
        self.group.set_num_planning_attempts(planning_attempts)

        # Start planning in a given joint position
        joints = [0.11, 0.00, -0.93, -2.22, -1.71, -1.68]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)

        self.planner_data = []

        self._marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)
        self.test_goal_complex_1()
        rospy.sleep(2)
        self.test_goal_complex_2()
        rospy.sleep(2)
        self.test_goal_complex_3()
        rospy.sleep(2)
        self.test_goal_complex_4()

    def test_goal_complex_1(self):
        joints = [-1.5239292524704837, -1.2894845382670848, -1.8913257438952191, -0.069273532016395,
                  1.466564282765517, 1.5430552349500921]

        marker_position = [0.15, 0.51, 0.80]
        self._plan_joints(joints, "Test 1")
        self._add_marker(marker_position, "goal")

    def test_goal_complex_2(self):
        # Start planning in a given joint position
        joints = [-1.5239292524704837, -1.2894845382670848, -1.8913257438952191, -0.069273532016395,
                  1.466564282765517, 1.5430552349500921]

        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)

        joints_2 = [-0.9705460107379753, -1.2274060909056506, -2.5798737542297077, 0.16233656301300264,
                    1.135619842704714, -2.8441037408209464]

        marker_position = [-0.04, 0.42, 0.41]
        self._plan_joints(joints_2, "Test 2")
        self._add_marker(marker_position, "goal")

    def test_goal_complex_3(self):
        # Start planning in a given joint position
        joints = [-0.9705460107379753, -1.2274060909056506, -2.5798737542297077, 0.16233656301300264,
                  1.135619842704714, -2.8441037408209464]

        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)

        joints_2 = [1.0661255455770708, -0.9074471336718001, -2.5576333130037856, 2.9757176414722126,
                    -1.6351170449654888, 0.2930372395567313]

        marker_position = [-0.23, -0.09, 0.28]
        self._plan_joints(joints_2, "Test 3")
        self._add_marker(marker_position, "goal")

    def test_goal_complex_4(self):
        # Start planning in a given joint position
        joints = [1.0661255455770708, -0.9074471336718001, -2.5576333130037856, 2.9757176414722126,
                  -1.6351170449654888, 0.2930372395567313]

        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)
        joints_2 = [-1.398753311402418, -1.022156133338257, 1.027548929442897, -1.8287371391328364,
                    -1.4912172680615894, -2.815605633361361]

        marker_position = [0.34, -0.98, 0.59]
        self._plan_joints(joints_2, "Test 4")
        self._add_marker(marker_position, "goal")


def table_output(plan_data):
    row_titles = ["Planner", "Plan name", "Plan succeeded", "Time of plan", "Total angle change"]

    print(tabulate(plan_data, headers=row_titles, tablefmt='orgtbl'))


def main(num, list):
    if list == "stomp":
        planner_list = ["STOMP"]
    elif list == "sbpl":
        planner_list = [""]
    else:
        planner_list = ["BKPIECEkConfigDefault", "ESTkConfigDefault", "KPIECEkConfigDefault", "LBKPIECEkConfigDefault",
                        "PRMkConfigDefault", "RRTkConfigDefault", "SBLkConfigDefault", "PRMstarkConfigDefault",
                        "RRTstarkConfigDefault"]
    test_data = []

    for plan in planner_list:
        test_data += TestPlannersComplex(num, plan).planner_data

    table_output(test_data)


if __name__ == "__main__":
    replanning = int(sys.argv[1])
    planner = sys.argv[2]
    # sleep to wait for point cloud
    time.sleep(30)
    # import rostest
    # while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
    #     time.sleep(0.5)
    # rostest.rosrun(PKG, "test_planners", TestPlanners)
    main(replanning, planner)
    rospy.spin()
