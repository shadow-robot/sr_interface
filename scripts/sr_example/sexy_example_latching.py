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

# This program is a simple script used for demos. It runs the hand
# through different given poses.

# It is a good example on how to use the latching feature of the
# ros publisher to make sure a data is received even if we don't stream
# the data.

import rospy

import time
import math
from std_msgs.msg import Float64


class SexyExampleLatching(object):

    # type of controller that is running
    controller_type = "_position_controller"

    def __init__(self):

        self.hand_publishers = self.create_hand_publishers()

        self.sleep_time = 3.0

    def run(self):
        """
        Runs the hand through different predefined position in a given order.
        """
        start = {"FFJ0":0, "FFJ3":0, "FFJ4":0,
                 "MFJ0":0, "MFJ3":0, "MFJ4":0,
                 "RFJ0":0, "RFJ3":0, "RFJ4":0,
                 "LFJ0":0, "LFJ3":0, "LFJ4":0, "LFJ5":0,
                 "THJ1":0, "THJ2":0, "THJ3":0, "THJ4":0, "THJ5":0,
                 "WRJ1":0, "WRJ2":0 }

        fist = {"FFJ0":180, "FFJ3":90, "FFJ4":0,
                "MFJ0":180, "MFJ3":90, "MFJ4":0,
                "RFJ0":180, "RFJ3":90, "RFJ4":0,
                "LFJ0":180, "LFJ3":90, "LFJ4":0, "LFJ5":0,
                "THJ1":0, "THJ2":0, "THJ3":0, "THJ4":50, "THJ5":-50,
                "WRJ1":0, "WRJ2":0 }

        victory = {"FFJ0":0, "FFJ3":0, "FFJ4":-20,
                   "MFJ0":0, "MFJ3":0, "MFJ4":20,
                   "RFJ0":180, "RFJ3":90, "RFJ4":-10,
                   "LFJ0":180, "LFJ3":90, "LFJ4":-10, "LFJ5":0,
                   "THJ1":40, "THJ2":20, "THJ3":0, "THJ4":50, "THJ5":35,
                   "WRJ1":0, "WRJ2":0 }

        wave_1 = {"WRJ2":-20}
        wave_2 = {"WRJ2":5}

        self.publish_pose(start)
        time.sleep(self.sleep_time)

        self.publish_pose(fist)
        time.sleep(self.sleep_time)

        self.publish_pose(start)
        time.sleep(self.sleep_time)

        self.publish_pose(victory)
        time.sleep(self.sleep_time)

        self.publish_pose(start)
        time.sleep(self.sleep_time)

        for _ in range(4):
            self.publish_pose(wave_1)
            time.sleep(self.sleep_time)
            self.publish_pose(wave_2)
            time.sleep(self.sleep_time)

    def publish_pose(self, pose):
        """
        Publish a given pose.
        """
        for joint, pos in pose.iteritems():
            self.hand_publishers[joint].publish(math.radians(pos))

    def create_hand_publishers(self):
        """
        Creates a dictionary of publishers to send the targets to the controllers
        on /sh_??j?_mixed_position_velocity_controller/command
        or /sh_??j?_position_controller/command
        """
        hand_pub = {}

        for joint in ["FFJ0", "FFJ3", "FFJ4",
                      "MFJ0", "MFJ3", "MFJ4",
                      "RFJ0", "RFJ3", "RFJ4",
                      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                      "WRJ1", "WRJ2" ]:
            # Here we initialize the publisher with the latch set to True.
            # this will ensure that the hand gets the message, even though we're
            # using the messages more as a service (we don't stream the data, we
            # just ask the hand to take a given position)
            hand_pub[joint] = rospy.Publisher('sh_'+joint.lower() + self.controller_type + '/command', Float64, latch=True)

        return hand_pub

def main():
    rospy.init_node('sexy_example_latching', anonymous = True)
    sexy_example = SexyExampleLatching()
    sexy_example.run()

if __name__ == '__main__':
    main()
