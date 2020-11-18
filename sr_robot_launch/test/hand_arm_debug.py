#!/usr/bin/env python
import rospy
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == "__main__":
    rospy.init_node('test_hand_and_arm_sim', anonymous=True)
    rospy.wait_for_message('/move_group/status', GoalStatusArray)

#    launch_file = 'sr_right_ur10arm_hand.launch'
#    launch_file = 'sr_right_ur5arm_hand.launch'
    launch_file = 'sr_left_ur10arm_hand.launch'
#    launch_file = 'sr_left_ur5arm_hand.launch'
    

    if 'ur5' in launch_file or 'ur5e' in launch_file:
        hand_type = 'hand_lite'
    elif 'ur10' in launch_file or 'ur10e' in launch_file:
        hand_type = 'hand_e' 
    if 'right' in launch_file:
        robot_side = 'right'
        hand_id = 'rh'
        arm_id = 'ra'
    elif 'left' in launch_file:
        robot_side = 'left'
        hand_id = 'lh'
        arm_id = 'la'

    print('hand_type')
    print(hand_type)
    print('robot_side')
    print(robot_side)
    print('hand_id')
    print(hand_id)
    print('arm_id')
    print(arm_id)

    if robot_side == 'right':
        robot_commander = SrRobotCommander(name="right_arm_and_hand")
    elif robot_side == 'left':
        robot_commander = SrRobotCommander(name="left_arm_and_hand")

    
    