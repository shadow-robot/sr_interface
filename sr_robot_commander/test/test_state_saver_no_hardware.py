#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2018, 2022-2023 belongs to Shadow Robot Company Ltd.
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

from unittest import TestCase
import rospy
from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe

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
            SrStateSaverUnsafe('')
        except Exception as exception:
            exception_raised = True
            exception_string = str(exception)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Cannot save with empty name.")

    def test_with_hand(self):
        exception_raised = False
        exception_string = ''

        try:
            SrStateSaverUnsafe("test_name", "hand")
        except Exception as exception:
            exception_raised = True
            exception_string = str(exception)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Unable to construct robot model. Please make"
                         " sure all needed information is on the parameter server.")

    def test_with_arm(self):
        exception_raised = False
        exception_string = ''

        try:
            SrStateSaverUnsafe("test_name", "arm")
        except Exception as exception:
            exception_raised = True
            exception_string = str(exception)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Group right_arm not found.")

    def test_with_both(self):
        exception_raised = False
        exception_string = ''

        try:
            SrStateSaverUnsafe("test_name", "both")
        except Exception as exception:
            exception_raised = True
            exception_string = str(exception)

        self.assertTrue(exception_raised)
        self.assertEqual(exception_string, "Unable to construct robot model. Please make"
                         " sure all needed information is on the parameter server.")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_state_saver_no_hardware", TestSrStateSaverExceptions)
