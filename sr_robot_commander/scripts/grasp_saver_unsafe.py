#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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
from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe


# ***DISCLAIMER***
# This script provides a command line interface to save the current pose of the robot (made of an arm and hand).
# Poses can be composed of either hand joints/arm joints/all joints. This can be useful for making smooth
# movements where the hand and arm should move synchronously. However, there are no safeties! In particular,
# if a hand and arm aren't both present , its behaviour is undefined. Likewise if more than one hand/arm are
# present. There is also no checking to prevent overwriting existing database entries/otherwise damaging the
# robot_states database. It is included here as it is a useful tool, but it should be considered a work in
# progress and used with care.
#
# You have been warned :)
#
# To use this script:
#
# rosrun sr_robot_commander grasp_saver_unsafe.py NAME_TO_SAVE WHAT_TO_SAVE
#
# NAME_TO_SAVE is the name that will be used for the state in the database.
#
# N.B. IF A STATE ALREADY EXISTS WITH THAT NAME IT WILL BE OVERWRITTEN WITH NO WARNING!
#
# WHAT_TO_SAVE specifies which joints will be included. It can be "arm","hand" or "both".
#
# If WHAT_TO_SAVE is omitted, it defaults to "both".

if __name__ == "__main__":
    rospy.init_node("state_saver")

    name = rospy.get_param("~name")
    which = rospy.get_param("~which", "all")
    hand_h = rospy.get_param("~hand_h", False)
    save_target = rospy.get_param("~save_target", False)

    if which == "all":
        state_saver = SrStateSaverUnsafe(name + "_hand", "hand", hand_h=hand_h, save_target=save_target)
        state_saver = SrStateSaverUnsafe(name + "_arm", "arm", hand_h=hand_h, save_target=save_target)
        state_saver = SrStateSaverUnsafe(name + "_both", "both", hand_h=hand_h, save_target=save_target)
    else:
        state_saver = SrStateSaverUnsafe(name + "_" + which, which, hand_h=hand_h, save_target=save_target)
