#!/usr/bin/env python3

# Copyright 2018 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
import rospy
from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe

from unittest import TestCase

PKG = "sr_robot_commander"


class TestSrStateSaverExceptions(TestCase):
    """
    Tests the state saver raises exceptions when launched with no hardware present.
    """

    def setUp(self):
        rospy.init_node('test_state_saver_no_hardware', anonymous=True)

    def test_no_name(self):
        exception_raised = False
        exception_string = ''

        try:
            state_saver = SrStateSaverUnsafe('')
        except Exception as e:
            exception_raised = True
            exception_string = str(e)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Cannot save with empty name.")

    def test_with_hand(self):
        exception_raised = False
        exception_string = ''

        try:
            state_saver = SrStateSaverUnsafe("test_name", "hand")
        except Exception as e:
            exception_raised = True
            exception_string = str(e)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Unable to construct robot model. Please make \
            sure all needed information is on the parameter server.")

    def test_with_arm(self):
        exception_raised = False
        exception_string = ''

        try:
            state_saver = SrStateSaverUnsafe("test_name", "arm")
        except Exception as e:
            exception_raised = True
            exception_string = str(e)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Group right_arm not found.")

    def test_with_both(self):
        exception_raised = False
        exception_string = ''

        try:
            state_saver = SrStateSaverUnsafe("test_name", "both")
        except Exception as e:
            exception_raised = True
            exception_string = str(e)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Unable to construct robot model. Please make \
            sure all needed information is on the parameter server.")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_state_saver_no_hardware", TestSrStateSaverExceptions)
