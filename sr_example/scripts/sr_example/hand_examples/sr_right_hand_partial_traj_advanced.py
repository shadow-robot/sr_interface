#!/usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# This example gives a more advanced use case for sending partial trajectories.
# A plot is generated to show the output of the desired and achieved positions and velocities.

import numpy as np

import rospy
from control_msgs.msg import JointTrajectoryControllerState,\
    FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


class PartialTrajListener():

    def __init__(self):
        self.start_time_goals = []
        self.start_time_goals_trajectory = []
        self.goal_joint_names = []
        self.trajectories = []
        self.start_goals = False
        self.finish_goals = False

        self.joints_time = []
        self.ffj3_actual = []
        self.ffj3_desired = []
        self.ffj3_error = []
        self.rfj3_actual = []
        self.rfj3_desired = []
        self.rfj3_error = []
        self.ffj3_vel_actual = []
        self.rfj3_vel_actual = []
        self.ffj3_vel_desired = []
        self.rfj3_vel_desired = []
        self.ffj3_vel_error = []
        self.rfj3_vel_error = []

        rospy.Subscriber("/rh_trajectory_controller/state",
                         JointTrajectoryControllerState, self.callback)
        rospy.Subscriber(
            "/rh_trajectory_controller/follow_joint_trajectory/result",
            FollowJointTrajectoryActionResult, self.callback_result)
        rospy.Subscriber(
            "/rh_trajectory_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal, self.callback_goal)

    def callback(self, state):
        self.joint_names = state.joint_names
        self.ffj3_index = self.joint_names.index("rh_FFJ3")
        self.rfj3_index = self.joint_names.index("rh_RFJ3")

        if self.start_goals and not self.finish_goals:
            self.joints_time.append(state.header.stamp.to_sec())
            self.ffj3_actual.append(state.actual.positions[self.ffj3_index])
            self.ffj3_desired.append(state.desired.positions[self.ffj3_index])
            self.ffj3_error.append(state.error.positions[self.ffj3_index])
            self.rfj3_actual.append(state.actual.positions[self.rfj3_index])
            self.rfj3_desired.append(state.desired.positions[self.rfj3_index])
            self.rfj3_error.append(state.error.positions[self.rfj3_index])

            self.ffj3_vel_actual.append(state.actual.velocities[self.ffj3_index])
            self.rfj3_vel_actual.append(state.actual.velocities[self.rfj3_index])
            self.ffj3_vel_desired.append(state.desired.velocities[self.ffj3_index])
            self.rfj3_vel_desired.append(state.desired.velocities[self.rfj3_index])
            self.ffj3_vel_error.append(state.error.velocities[self.ffj3_index])
            self.rfj3_vel_error.append(state.error.velocities[self.rfj3_index])

    def callback_result(self, result):
        print ("Trajectory Goal: " + result.status.goal_id.id +
               " finished with status: " + str(result.status.status))

    def callback_goal(self, goal):
        self.start_goals = True
        self.goal_joint_names.append(goal.goal.trajectory.joint_names)
        self.start_time_goals.append(goal.header.stamp.to_sec())
        self.start_time_goals_trajectory.append(goal.goal.trajectory.header.stamp.to_sec())
        self.trajectories.append(goal.goal.trajectory.points)

    def plot_settings(self, plt):
        ax = plt.gca()
        plt.grid(which='both', axis='both')
        plt.setp(ax.get_xticklabels(), fontsize=8)
        plt.setp(ax.get_yticklabels(), fontsize=8)
        plt.xlabel('Time (s)')
        ax.xaxis.label.set_size(10)
        ax.yaxis.label.set_size(10)

    def graph(self):
        time_zero = self.joints_time[0]
        time = np.array(self.joints_time) - time_zero

        plt.figure()

        # Plot goal trajectories waypoints
        time_ffj3_traj = []
        angle_ffj3_traj = []
        time_rfj3_traj = []
        angle_rfj3_traj = []
        for i, traj in enumerate(self.trajectories):
            ffj3_goal_index = self.goal_joint_names[i].index("rh_FFJ3") if "rh_FFJ3" in self.goal_joint_names[i] else -1
            rfj3_goal_index = self.goal_joint_names[i].index("rh_RFJ3") if "rh_RFJ3" in self.goal_joint_names[i] else -1

            for point in traj:
                if ffj3_goal_index > -1:
                    time_ffj3_traj.append(point.time_from_start.to_sec() +
                                          self.start_time_goals_trajectory[i] -
                                          time_zero)
                    angle_ffj3_traj.append(point.positions[ffj3_goal_index])
                if rfj3_goal_index > -1:
                    time_rfj3_traj.append(point.time_from_start.to_sec() +
                                          self.start_time_goals_trajectory[i] -
                                          time_zero)
                    angle_rfj3_traj.append(point.positions[rfj3_goal_index])

            if ffj3_goal_index > -1:
                plt.subplot(3, 2, 1)
                plt.plot(time_ffj3_traj, angle_ffj3_traj, 'o',
                         label="Traj " + str(i + 1))

            if rfj3_goal_index > -1:
                plt.subplot(3, 2, 2)
                plt.plot(time_rfj3_traj, angle_rfj3_traj, 'o',
                         label="Traj " + str(i + 1))

            time_ffj3_traj = []
            angle_ffj3_traj = []
            time_rfj3_traj = []
            angle_rfj3_traj = []

        # Plot trajectories
        plt.subplot(3, 2, 1)
        plt.plot(time, self.ffj3_actual, 'black', label="Actual traj")
        plt.plot(time, self.ffj3_desired, 'green', label="Desired traj")
        plt.ylabel('FFJ3 Actual position (rad)')
        self.plot_settings(plt)
        plt.ylim(ymax=2.2, ymin=-0.1)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 2)
        plt.plot(time, self.rfj3_actual, 'black', label="Actual traj")
        plt.plot(time, self.rfj3_desired, 'green', label="Desired traj")
        plt.ylabel('RFJ3 Actual position (rad)')
        self.plot_settings(plt)
        plt.ylim(ymax=2.2, ymin=-0.1)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 3)
        plt.plot(time, self.ffj3_vel_actual, 'black', label="Actual traj")
        plt.plot(time, self.ffj3_vel_desired, 'green', label="Desired traj")
        plt.ylabel('FFJ3 Actual velocity')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 4)
        plt.plot(time, self.rfj3_vel_actual, 'black', label="Actual traj")
        plt.plot(time, self.rfj3_vel_desired, 'green', label="Desired traj")
        plt.ylabel('RFJ3 Actual velocity')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 5)
        plt.plot(time, self.ffj3_vel_error, 'red', label="Error traj")
        plt.ylabel('FFJ3 Velocity Error')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)

        plt.subplot(3, 2, 6)
        plt.plot(time, self.rfj3_vel_error, 'red', label="Error traj")
        plt.ylabel('RFJ3 Velocity Error')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)

        plt.subplots_adjust(left=0.07, right=0.96, bottom=0.083, top=0.90)
        plt.show()


def construct_trajectory_point(posture, duration):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(posture[key])
    return trajectory_point

# 6 position goals are specified
open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
             'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
             'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp1 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.5235, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.5235, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp2 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.0472, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.0472, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp3 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.4, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.4, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp4 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.5, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp5 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

# Two partial trajectories are defined here for joint rh_FFJ3
grasp_partial_1 = {'rh_FFJ3': 1.06}
grasp_partial_2 = {'rh_FFJ3': 1.2}

if __name__ == '__main__':
    rospy.init_node("partial_traj_example_advanced", anonymous=True)
    rospy.sleep(1)  # Do not start at time zero

    listener = PartialTrajListener()
    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                     hand_serial=hand_serial)

    hand_mapping = hand_parameters.mapping[hand_serial]

    # Hand joints are detected
    joints = hand_finder.get_hand_joints()[hand_mapping]

    # Adjust poses according to the hand loaded
    open_hand_current = dict([(i, open_hand[i]) for i in joints if i in open_hand])
    grasp1_current = dict([(i, grasp1[i]) for i in joints if i in grasp1])
    grasp2_current = dict([(i, grasp2[i]) for i in joints if i in grasp2])
    grasp3_current = dict([(i, grasp3[i]) for i in joints if i in grasp3])
    grasp4_current = dict([(i, grasp4[i]) for i in joints if i in grasp4])
    grasp5_current = dict([(i, grasp5[i]) for i in joints if i in grasp5])

    start_time = rospy.Time.now()

    # Opening hand
    rospy.loginfo("Moving hand to open position")
    trajectory_start_time = 1.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(open_hand_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(open_hand_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)

    # Closing index and middle fingers. Trajectories are generated from grasp1 - grasp5 and run with hand_commander
    rospy.loginfo("Closing index and ring fingers")
    trajectory_start_time = 4.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp1_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(grasp1_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp2_current, 4.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp3_current, 6.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp4_current, 8.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp5_current, 10.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    # Interrupting trajectory of index using two partial trajectories
    rospy.loginfo("Moving index finger to partial trajectory goals")
    rospy.sleep(2)
    trajectory_start_time = 8.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp_partial_1.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(grasp_partial_1, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp_partial_2, 3.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    graphs_finished = False

    rate = rospy.Rate(10)

    # Do not exit until graphs closed
    while not rospy.is_shutdown():
        if len(listener.joints_time) > 5 and\
                (listener.joints_time[-1] - listener.joints_time[0]) > 15 and\
                not graphs_finished:
            listener.finish_goals = True
            listener.graph()
            graphs_finished = True
            break
        rate.sleep()
