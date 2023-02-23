#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
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

import argparse
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


if __name__ == "__main__":

    rospy.init_node("close_hand", anonymous=True)

    parser = argparse.ArgumentParser(description="Hand side")
    parser.add_argument("-s", "--side",
                        dest="side",
                        type=str,
                        required=True,
                        help="Please select hand side, can be 'right', 'left' or 'both'.",
                        default=None,
                        choices=["right", "left", "both"])

    args = parser.parse_args(rospy.myargv()[1:])

    if args.side == 'right':
        hand_name = 'right_hand'
    elif args.side == 'left':
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    trajectory = [
        {
            'name': 'open',
            'interpolate_time': 3.0,
            'pause_time': 2
        },
        {
            'name': 'fingers_pack_thumb_open',
            'interpolate_time': 3.0,
            'pause_time': 2
        },
        {
            'name': 'pack',
            'interpolate_time': 3.0,
            'pause_time': 2
        }
    ]

    commander_instance = SrHandCommander(hand_name)
    commander_instance.run_named_trajectory(trajectory)
