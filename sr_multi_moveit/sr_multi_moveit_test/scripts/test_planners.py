#!/usr/bin/env python

# A script to test planners, to be run on the command line with group name as the argument (left_arm or right_arm)

import rospy
import sys
import numpy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
import copy
import time
import argparse
import rosgraph


class TestPlanners(object):
    def __init__(self, group_id, planner):

        rospy.init_node('moveit_test_planners', anonymous=True)
        self.pass_list = []
        self.fail_list = []
        self.planner = planner
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(group_id)
        rospy.sleep(1)

        self.group.set_planner_id(self.planner)

        self.test_trajectories_empty_environment()
        self.test_waypoints()
        self.test_trajectories_with_walls_and_ground()

        for pass_test in self.pass_list:
            print(pass_test)

        for fail_test in self.fail_list:
            print(fail_test)

    def _add_walls_and_ground(self):
        # publish a scene
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()

        p.pose.position.x = 0
        p.pose.position.y = 0
        # offset such that the box is below ground (to prevent collision with
        # the robot itself)
        p.pose.position.z = -0.11
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        self.scene.add_box("ground", p, (3, 3, 0.1))

        p.pose.position.x = 0.4
        p.pose.position.y = 0.85
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.5
        p.pose.orientation.y = -0.5
        p.pose.orientation.z = 0.5
        p.pose.orientation.w = 0.5
        self.scene.add_box("wall_front", p, (0.8, 2, 0.01))

        p.pose.position.x = 1.33
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707388
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.706825
        self.scene.add_box("wall_right", p, (0.8, 2, 0.01))

        p.pose.position.x = -0.5
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707107
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.707107
        self.scene.add_box("wall_left", p, (0.8, 2, 0.01))

        # rospy.sleep(1)

    def _check_plan(self, plan):
        if len(plan.joint_trajectory.points) > 0:
            return True
        else:
            return False

    def _plan_joints(self, joints):
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0:6] = joints[0:6]
        self.group.set_joint_value_target(group_variable_values)

        plan = self.group.plan()
        return self._check_plan(plan)

    def test_trajectories_rotating_each_joint(self):
        # test_joint_values = [numpy.pi/2.0, numpy.pi-0.33, -numpy.pi/2]
        test_joint_values = [numpy.pi/2.0]
        joints = [0.0, 0.0, 0.0, -numpy.pi / 2.0, 0.0, 0.0]
        # Joint 4th is colliding with the hand
        # for joint in range(6):
        for joint in [0, 1, 2, 3, 5]:
            for value in test_joint_values:
                joints[joint] = value
            if not self._plan_joints(joints):
                self.fail_list.append("Failed: test_trajectories_rotating_each_joint, " + self.planner +
                                      ", joints:" + str(joints))
            else:
                self.pass_list.append("Passed: test_trajectories_rotating_each_joint, " + self.planner +
                                      ", joints:" + str(joints))

    def test_trajectories_empty_environment(self):
        # Up - Does not work with sbpl but it does with ompl
        joints = [
            0.0, -1.99, 2.19, 0.58, 0.0, -0.79, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # All joints up
        joints = [-1.67232, -2.39104, 0.264862,
                  0.43346, 2.44148, 2.48026, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # Down
        joints = [
            -0.000348431194526, 0.397651011661, 0.0766181197394, -
            0.600353691727, -0.000441966540076,
            0.12612019707, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # left
        joints = [
            0.146182953165, -2.6791929848, -
            0.602721109682, -3.00575848765, 0.146075718452,
            0.00420656698366, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # Front
        joints = [
            1.425279839, -0.110370375874, -
            1.52548746261, -1.50659865247, -1.42700242769,
            3.1415450794, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # Behind
        joints = [
            1.57542451065, 3.01734161219, 2.01043257686, -
            1.14647092839, 0.694689321451,
            -0.390769365032, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

        # Should fail because it is in self-collision
        joints = [
            -0.289797803762, 2.37263860495, 2.69118483159,  1.65486712181, 1.04235601797,
            -1.69730925867, 0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_empty_environment, " + self.planner +
                                  ", joints:" + str(joints))

    def test_waypoints(self):
        # Start planning in a given joint position
        joints = [
            -0.324590029242, -1.42602359749, -
            1.02523472017, -0.754761892979, 0.344227622185,
            -3.03250264451, 0.0, 0.0]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state(
        ).joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:8] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)

        waypoints = []

        initial_pose = self.group.get_current_pose().pose

        initial_pose.position.x = -0.301185959729
        initial_pose.position.y = 0.517069787724
        initial_pose.position.z = 1.20681710541
        initial_pose.orientation.x = 0.0207499700474
        initial_pose.orientation.y = -0.723943002716
        initial_pose.orientation.z = -0.689528413407
        initial_pose.orientation.w = 0.00515118111221

        # start with a specific position
        waypoints.append(initial_pose)

        # first move it down
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation = waypoints[0].orientation
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z - 0.20
        waypoints.append(copy.deepcopy(wpose))

        # second front
        wpose.position.y += 0.20
        waypoints.append(copy.deepcopy(wpose))

        # third side
        wpose.position.x -= 0.20
        waypoints.append(copy.deepcopy(wpose))

        # fourth return to initial pose
        wpose = waypoints[0]
        waypoints.append(copy.deepcopy(wpose))

        (plan3, fraction) = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        if not self._check_plan(plan3) and fraction > 0.8:
            self.fail_list.append("Failed: test_waypoints, " + self.planner)
        else:
            self.pass_list.append("Passed: test_waypoints, " + self.planner)

    def test_trajectories_with_walls_and_ground(self):
        self._add_walls_and_ground()

        # Should fail to plan: Goal is in collision with the wall_front
        joints = [
            0.302173213174, 0.192487443763, -
            1.94298265002, 1.74920382275, 0.302143499777, 0.00130280337897,
            0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))

        # Should fail to plan: Goal is in collision with the ground
        joints = [
            3.84825722288e-05, 0.643694953509, -
            1.14391175311, 1.09463824437, 0.000133883149666, -
            0.594498939239,
            0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed: test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))

        # Goal close to right corner
        joints = [
            0.354696232081, -0.982224980654, 0.908055961723, -
            1.92328051116, -1.3516255551, 2.8225061435,
            0.0, 0.0]
        if not self._plan_joints(joints):
            self.fail_list.append("Failed: test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))
        else:
            self.pass_list.append("Passed, test_trajectories_with_walls_and_ground, " + self.planner +
                                  ", joints:" + str(joints))

        self.scene.remove_world_object("ground")
        self.scene.remove_world_object("wall_left")
        self.scene.remove_world_object("wall_right")
        self.scene.remove_world_object("wall_front")


def main():
    parser = argparse.ArgumentParser(description='A script to test moveit planners. '
                                                 'Should be run after launching the UR10 arm and shadow hand: \n '
                                                 'roslaunch sr_robot_launch sr_right_ur10arm_hand.launch OR \n '
                                                 'roslaunch sr_robot_launch sr_left_ur10arm_hand.launch',
                                     add_help=True, usage='%(prog)s [group_id]',
                                     formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument(dest='group_id', help="Should be either right_arm or left_arm")
    parser.parse_args()

    group_id = str(sys.argv[1])

    try:
        rospy.get_master().getPid()
    except:
        print("Please launch robot.")
        sys.exit()

    # sleep to wait for robot description to be loaded
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        print("Waiting for robot description, has robot been launched?")
        time.sleep(0.5)

    planner_list = ["BKPIECEkConfigDefault", "ESTkConfigDefault", "KPIECEkConfigDefault", "LBKPIECEkConfigDefault",
                    "PRMkConfigDefault", "RRTkConfigDefault", "SBLkConfigDefault", "PRMstarkConfigDefault",
                    "RRTstarkConfigDefault"]
    for planner in planner_list:
        TestPlanners(group_id, planner)

if __name__ == "__main__":
    main()
