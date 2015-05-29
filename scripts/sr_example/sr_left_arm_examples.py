#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("basic_arm_examples", anonymous=True)

arm_commander = SrArmCommander(name="left_arm", set_ground=False)

rospy.sleep(rospy.Duration(2))

pose_1 = [0.5, -0.3, 1.2, 0, 1.57, 0]
print("Planning the move to the first pose\n" + str(pose_1) + "\n")
arm_commander.plan_to_pose_target(pose_1)
print("Finished planning, moving the arm now.")
arm_commander.execute()
 
rospy.sleep(6.0)
 
pose_2 = [0.5, 0.3, 1.2, 0, 1.57, 0]
print("Planning the move to the second pose\n" + str(pose_2) + "\n")
arm_commander.plan_to_pose_target(pose_2)
print("Finished planning, moving the arm now.")
arm_commander.execute()

pose_1 = [0.5, -0.3, 1.2, 0, 1.57, 0]
print("Moving arm to pose\n" + str(pose_1) + "\n")
arm_commander.move_to_pose_target(pose_1, wait=True)

rospy.sleep(6.0)

pose_2 = [0.5, 0.3, 1.2, 0, 1.57, 0]
print("Moving arm to pose\n" + str(pose_2) + "\n")
arm_commander.move_to_pose_target(pose_2, wait=True)

position_1 = [0.5, -0.3, 1.2]
print("Moving arm to position\n" + str(position_1) + "\n")
arm_commander.move_to_position_target(position_1)

rospy.sleep(rospy.Duration(5))

position_2 = [0.5, 0.3, 1.2]
print("Planning the move to the second position\n" + str(position_2) + "\n")
arm_commander.plan_to_position_target(position_2)
print("Finished planning, moving the arm now.")
arm_commander.execute()

rospy.sleep(rospy.Duration(5))

print("Arm joints position\n" + str(arm_commander.get_joints_position()) + "\n")

joints_states_1 = {'la_shoulder_pan_joint': 0.43221632746577665, 'la_elbow_joint': 2.118891128999479,
                   'la_wrist_1_joint': -1.711370650686752, 'la_wrist_2_joint': 1.4834244535003318,
                   'la_shoulder_lift_joint': -2.5813317754982474, 'la_wrist_3_joint': 1.6175960918705412}
print("Moving arm to joints state\n" + str(joints_states_1) + "\n")

arm_commander.move_to_joint_value_target(joints_states_1)

rospy.sleep(rospy.Duration(5))

joints_states_2 = {'la_shoulder_pan_joint': 0.4225743596855942, 'la_elbow_joint': 1.9732180863151747,
                   'la_wrist_1_joint': -0.8874321427449576, 'la_wrist_2_joint': -0.9214312892819567,
                   'la_shoulder_lift_joint': -1.9299519748391978, 'la_wrist_3_joint': 0.7143446787498702}

print("Moving arm to joints state\n" + str(joints_states_2) + "\n")
arm_commander.move_to_joint_value_target(joints_states_2)

rospy.sleep(rospy.Duration(5))

print("Arm joints position\n" + str(arm_commander.get_joints_position()) + "\n")
  
joints_states_3 = {'la_shoulder_pan_joint': 1.6113530596480121, 'la_elbow_joint': 1.1552231775506083,
                   'la_wrist_1_joint': -0.2393325455779891, 'la_wrist_2_joint': 0.4969532212998553,
                   'la_shoulder_lift_joint': -1.5826889903403423, 'la_wrist_3_joint': 2.1117520537195738}

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

named_target = "gamma"
print("Moving arm to named target " + named_target)
arm_commander.move_to_named_target(named_target)

rospy.sleep(rospy.Duration(3))

print("Arm joints position\n" + str(arm_commander.get_joints_position()) + "\n")
print("Arm joints velocities\n" + str(arm_commander.get_joints_velocity()) + "\n")
