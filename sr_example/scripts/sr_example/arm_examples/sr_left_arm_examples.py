#!/usr/bin/env python

# This example demonstrates some of the functions of the arm commander.
# The arm is moved through a sequence of goals generated via different functions in the commander.

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("basic_left_arm_example", anonymous=True)

arm_commander = SrArmCommander(name="left_arm", set_ground=False)

rospy.sleep(rospy.Duration(2))

# The arm commander generates a plan to a new pose before the pose is executed.
pose_1 = [0.5, -0.3, 1.2, 0, 1.57, 0]
print("Planning the move to the first pose:\n" + str(pose_1) + "\n")
arm_commander.plan_to_pose_target(pose_1)
print("Finished planning, moving the arm now.")
# Can only execute if a plan has been generated
arm_commander.execute()

rospy.sleep(6.0)

pose_2 = [0.5, 0.3, 1.2, 0, 1.57, 0]
print("Planning the move to the second pose:\n" + str(pose_2) + "\n")
arm_commander.plan_to_pose_target(pose_2)
print("Finished planning, moving the arm now.")
arm_commander.execute()

# Here a pose is provided and the arm commander moves the arm to it
pose_1 = [0.5, -0.3, 1.2, 0, 1.57, 0]
print("Moving arm to pose:\n" + str(pose_1) + "\n")
arm_commander.move_to_pose_target(pose_1, wait=True)

rospy.sleep(6.0)

pose_2 = [0.5, 0.3, 1.2, 0, 1.57, 0]
print("Moving arm to pose:\n" + str(pose_2) + "\n")
arm_commander.move_to_pose_target(pose_2, wait=True)

# The goal given here is in cartesian co-ordinates and is the goal for the end effector
position_1 = [0.5, -0.3, 1.2]
print("Moving arm to position:\n" + str(position_1) + "\n")
arm_commander.move_to_position_target(position_1)

rospy.sleep(rospy.Duration(5))

# Planning to a cartesian co-ordinate goal
position_2 = [0.5, 0.3, 1.2]
print("Planning the move to the second position:\n" + str(position_2) + "\n")
arm_commander.plan_to_position_target(position_2)
print("Finished planning, moving the arm now.")
arm_commander.execute()

rospy.sleep(rospy.Duration(5))

# Read the current joint positions
print("Arm joints position:\n" +
      str(arm_commander.get_joints_position()) + "\n")

joints_states_1 = {'la_shoulder_pan_joint': 0.43, 'la_elbow_joint': 2.12, 'la_wrist_1_joint': -1.71,
                   'la_wrist_2_joint': 1.48, 'la_shoulder_lift_joint': -2.58, 'la_wrist_3_joint': 1.62,
                   'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

print("Moving arm to joints position:\n" + str(joints_states_1) + "\n")

arm_commander.move_to_joint_value_target(joints_states_1)
rospy.sleep(rospy.Duration(5))

joints_states_2 = {'la_shoulder_pan_joint': 0.42, 'la_elbow_joint': 1.97, 'la_wrist_1_joint': -0.89,
                   'la_wrist_2_joint': -0.92, 'la_shoulder_lift_joint': -1.93, 'la_wrist_3_joint': 0.71,
                   'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

print("Moving arm to joints position:\n" + str(joints_states_2) + "\n")

arm_commander.move_to_joint_value_target(joints_states_2)
rospy.sleep(rospy.Duration(5))

print("Arm joints position:\n" + str(arm_commander.get_joints_position()) + "\n")

joints_states_3 = {'la_shoulder_pan_joint': 1.61, 'la_elbow_joint': 1.15, 'la_wrist_1_joint': -0.24,
                   'la_wrist_2_joint': 0.49, 'la_shoulder_lift_joint': -1.58, 'la_wrist_3_joint': 2.11,
                   'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

# A trajectory is generated from the 3 positions for each joint, specified in joint_states_1, joint_states_2
# and joint_states_3
print("Running joints trajectory")

joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = rospy.Time.now()
joint_trajectory.joint_names = list(joints_states_1.keys())
joint_trajectory.points = []
time_from_start = rospy.Duration(5)

for joints_states in [joints_states_1, joints_states_2, joints_states_3]:
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = time_from_start
    time_from_start = time_from_start + rospy.Duration(5)

    trajectory_point.positions = []
    trajectory_point.velocities = []
    trajectory_point.accelerations = []
    trajectory_point.effort = []
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(joints_states[key])
        trajectory_point.velocities.append(0.0)
        trajectory_point.accelerations.append(0.0)
        trajectory_point.effort.append(0.0)

    joint_trajectory.points.append(trajectory_point)

arm_commander.run_joint_trajectory(joint_trajectory)

rospy.sleep(rospy.Duration(15))

# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
named_target = "gamma"
print("Moving arm to named target: " + named_target)
arm_commander.move_to_named_target(named_target)

rospy.sleep(rospy.Duration(3))

# Current positions and velocities are read
print("Arm joints position:\n" +
      str(arm_commander.get_joints_position()) + "\n")
print("Arm joints velocities:\n" +
      str(arm_commander.get_joints_velocity()) + "\n")
