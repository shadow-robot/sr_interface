#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2024 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.


import rospy
import os
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter
from std_msgs.msg import Header

SQUEEZE_TYPE = 'tea'

rospy.init_node("manual_squeeze")

state_pub = rospy.Publisher("trial_state", Header, latch=True, queue_size=1)
hand_commander = SrHandCommander()
state_exporter = SrRobotStateExporter()
state_exporter.extract_list([f'pre_squeeze_{SQUEEZE_TYPE}', f'post_squeeze_{SQUEEZE_TYPE}'])

state_exporter.output_module(os.path.join(os.path.dirname(__file__), "exported_states.py"))

from exported_states import warehouse_states

trial_n = 1
while not rospy.is_shutdown():
    hand_commander.move_to_joint_value_target_unsafe(
        warehouse_states[f'pre_squeeze_{SQUEEZE_TYPE}'],
        wait=False
    )

    input("Press SPACE to start data collection...")
    msg = Header()
    msg.stamp = rospy.Time.now()
    msg.frame_id = f"trial_{trial_n}_start"
    state_pub.publish(msg)

    hand_commander.move_to_joint_value_target_unsafe(
        warehouse_states[f'post_squeeze_{SQUEEZE_TYPE}'],
        time=3.0
    )

    input("Press SPACE to end data collection...")
    msg = Header()
    msg.stamp = rospy.Time.now()
    msg.frame_id = f"trial_{trial_n}_end"
    state_pub.publish(msg)

    trial_n += 1
