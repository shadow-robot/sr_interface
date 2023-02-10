#!/usr/bin/env python3
#
# Software License Agreement (BSD License)
# Copyright © 2011, 2022-2023 belongs to Shadow Robot Company Ltd.
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

"""
This program is a simple script that runs the hand through different given poses.

It is a good example on how to use the latching feature of the
ros publisher to make sure a data is received even if we don't stream
the data.

"""

import time
import rospy
from std_msgs.msg import Float64


class LatchingExample:
    # type of controller that is running
    controller_type = "_position_controller"

    def __init__(self):
        # Prefix of hand, can be lh for left or rh for right
        self.prefix = 'rh'
        self.keys = ['FFJ0', 'FFJ3', 'FFJ4',
                     'MFJ0', 'MFJ3', 'MFJ4',
                     'RFJ0', 'RFJ3', 'RFJ4',
                     'LFJ0', 'LFJ3', 'LFJ4', 'LFJ5',
                     'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5',
                     'WRJ1', 'WRJ2']

        self.keys_prefixed = [f"{self.prefix}_{joint}" for joint in self.keys]
        self.hand_publishers = self.create_hand_publishers(self.keys_prefixed)
        self.sleep_time = 3.0

    def run(self):
        """
        Runs the hand through different predefined position in a given order.
        """

        start = [0.00, 0.00, 0.00, 0.00,
                 0.00, 0.00, 0.00, 0.00,
                 0.00, 0.00, 0.00, 0.00,
                 0.00, 0.00, 0.00, 0.00, 0.00,
                 0.00, 0.00, 0.00, 0.00, 0.00,
                 0.00, 0.00]

        fist = [3.14, 1.57, 0.00,
                3.14, 1.57, 0.00,
                3.14, 1.57, 0.00,
                3.4, 1.57, 0.00, 0.00,
                1.33, 0.00, 1.15, 0.26,
                0.00, 0.00]

        victory = [0.00, 0.00, -0.35,
                   0.00, 0.00, 0.35,
                   3.14, 1.57, -0.17,
                   3.14, 1.57, -0.17, 0.00,
                   1.05, 0.00, 0.87, 0.61,
                   0.00, 0.00]

        wave_1 = [-0.35]
        wave_2 = [0.09]

        start_pose = dict(zip(self.keys_prefixed, start))
        fist_pose = dict(zip(self.keys_prefixed, fist))
        victory_pose = dict(zip(self.keys_prefixed, victory))
        wave_1_pose = dict(zip(self.keys_prefixed[-1:], wave_1))
        wave_2_pose = dict(zip(self.keys_prefixed[-1:], wave_2))

        # publish each pose and sleep for time 'sleep_time' between each command
        self.publish_pose(start_pose)
        time.sleep(self.sleep_time)

        self.publish_pose(fist_pose)
        time.sleep(self.sleep_time)

        self.publish_pose(start_pose)
        time.sleep(self.sleep_time)

        self.publish_pose(victory_pose)
        time.sleep(self.sleep_time)

        self.publish_pose(start_pose)
        time.sleep(self.sleep_time)

        for _ in range(4):
            self.publish_pose(wave_1_pose)
            time.sleep(self.sleep_time / 2)
            self.publish_pose(wave_2_pose)
            time.sleep(self.sleep_time / 2)

    def publish_pose(self, pose):
        """
        Publish a given pose.
        """
        for joint, pos in pose.items():
            self.hand_publishers[joint].publish(pos)

    def create_hand_publishers(self, keys_prefixed):
        """
        Creates a dictionary of publishers to send the targets to the controllers
        on /sh_??_??j?_mixed_position_velocity_controller/command
        or /sh_??_??j?_position_controller/command
        """
        hand_pub = {}

        for joint in keys_prefixed:
            # Here we initialize the publisher with the latch set to True.
            # this will ensure that the hand gets the message, even though we're
            # using the messages more as a service (we don't stream the data, we
            # just ask the hand to take a given position)
            hand_pub[joint] = rospy.Publisher(
                'sh_' + joint.lower() + self.controller_type + '/command', Float64,
                latch=True, queue_size=1)

        return hand_pub


def main():
    rospy.init_node('sr_latching_example', anonymous=True)
    example = LatchingExample()
    example.run()


if __name__ == '__main__':
    main()
