#!/usr/bin/env python
import rospy
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == "__main__":
    rospy.init_node('test_hand_e_sim', anonymous=True)
    rospy.wait_for_message('/move_group/status', GoalStatusArray)
    robot_commander = SrRobotCommander(name="right_arm_and_hand")
    hand_arm_joints_goal = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                            'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                            'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                            'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                            'rh_LFJ5': 0.0, 'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0,
                            'rh_THJ5': 0.0, 'rh_WRJ1': 0.6, 'rh_WRJ2': 0.0,
                            'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 0.00,
                            'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                            'ra_shoulder_lift_joint': -0.58, 'ra_wrist_3_joint': 0.00,
                            'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                            'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}
                  
    robot_commander.move_to_joint_value_target_unsafe(hand_arm_joints_goal, 6.0, True)
    rospy.sleep(10)
    final_joint_values = robot_commander.get_current_state()
    print("sorted start states")
    print(sorted(hand_arm_joints_goal))
    print("sorted final states")
    print(sorted(final_joint_values))

    