#!/usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
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

# Example to demonstrate moving to stored/names targets. Both arm and hand movements executed.
# Available named targets can be viewed in MoveIt, on the planning tab.

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("grab_bottle_example", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

print("Executing moves:")

# start a bit higher
# arm_commander.move_to_named_target("gamma", True)

# approach
print("Approach bottle")
hand_commander.move_to_named_target("open_bottle", False)
arm_commander.move_to_named_target("approach_bottle", True)

# grab
print("Hold bottle")
arm_commander.move_to_named_target("hold_bottle", True)
hand_commander.move_to_named_target("hold_bottle", True)

# pour
print("Pour")
arm_commander.move_to_named_target("pour_bottle", True)

# release
print("Release bottle")
arm_commander.move_to_named_target("hold_bottle", True)
hand_commander.move_to_named_target("open_bottle", True)

# and move back
print("Return")
arm_commander.move_to_named_target("approach_bottle", True)

# victory
print("Peace")
arm_commander.move_to_named_target("victory")
hand_commander.move_to_named_target("victory")
