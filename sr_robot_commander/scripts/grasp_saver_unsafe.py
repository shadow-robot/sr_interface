#!/usr/bin/env python

from sys import argv

import rospy

from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe


"""
***DISCLAIMER***

This script provides a command line interface to save the current pose of the robot (made of an arm and hand).
Poses can be composed of either hand joints/arm joints/all joints. This can be useful for making smooth
movements where the hand and arm should move synchronously. However, there are no safeties! In particular,
if a hand and arm aren't both present , its behaviour is undefined. Likewise if more than one hand/arm are
present. There is also no checking to prevent overwriting existing database entries/otherwise damaging the
robot_states database. It is included here as it is a useful tool, but it should be considered a work in
progress and used with care.

You have been warned :)


To use this script:

rosrun sr_robot_commander grasp_saver_unsafe.py NAME_TO_SAVE WHAT_TO_SAVE

NAME_TO_SAVE is the name that will be used for the state in the database.

N.B. IF A STATE ALREADY EXISTS WITH THAT NAME IT WILL BE OVERWRITTEN WITH NO WARNING!

WHAT_TO_SAVE specifies which joints will be included. It can be "arm","hand" or "both".

If WHAT_TO_SAVE is omitted, it defaults to "both".
"""

if "__main__" == __name__:
    rospy.init_node("state_saver")
    if len(argv) <= 1 or "" == argv[1]:
        rospy.logerr("You didn't enter a name.")
        exit(-1)

    which = 'all'

    if len(argv) > 2:
        which = argv[2]

    if which == "all":
        gs = SrStateSaverUnsafe(argv[1] + "_hand", "hand")
        gs = SrStateSaverUnsafe(argv[1] + "_arm", "arm")
        gs = SrStateSaverUnsafe(argv[1] + "_both", "both")
    else:
        gs = SrStateSaverUnsafe(argv[1] + "_" + which, which)
