#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

"""
This program is a simple script that runs the hand through different given poses.

It is a good example on how to use the latching feature of the
ros publisher to make sure a data is received even if we don't stream
the data.

"""

import rospy

import time
from std_msgs.msg import Float64


class LatchingExample(object):
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

        self.keys_prefixed = ["{0}_{1}".format(self.prefix, joint)
                              for joint in self.keys]

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
            time.sleep(self.sleep_time/2)
            self.publish_pose(wave_2_pose)
            time.sleep(self.sleep_time/2)

    def publish_pose(self, pose):
        """
        Publish a given pose.
        """
        for joint, pos in pose.iteritems():
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
