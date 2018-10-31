#!/usr/bin/env python

# This example demonstrates how you can send a trajectory created from named poses.

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_pose_exporter import SrRobotPoseExporter

# It's assumed that a module containing poses and named named exported_poses.py has already
# been exported and is found somewhere on the path (e.g. in the same directory as this script).
# Let's assume it contains two poses, 'pose_1' and 'pose_2'

from exported_poses import warehouse_poses

"""
Now we have a dictionary of poses called warehouse_poses, e.g.
warehouse_poses = {
  'pose_1': {
     'joint_0': 0.00,
     'joint_1': 0.00
  },
  'pose_2': {
     'joint_0': 0.00,
     'joint_1': 0.00
  }
}
"""


# Define a hand commander, so we have something to do with the poses we extracted.
hand_commander = SrHandCommander()

# You could use the poses directly:
hand_commander.move_to_named_target(warehouse_poses['pose_1'])


# You could translate all the named poses in a trajectory:

trajectory = [
    {
        'name': 'pose_1',
        'interpolate_time': 3.0
    },
    {
        'name': 'pose_2',
        'interpolate_time': 3.0,
        'pause_time': 2
    }
]

pose_exporter = SrRobotPoseExporter(warehouse_poses))
converted_trajectory = pose_exporter.convert_trajectory(trajectory)

hand_commander.run_named_trajectory(converted_trajectory)


# Or we could repopulate the warehouse with the exported poses:

pose_exporter = SrRobotPoseExporter(warehouse_poses))
pose_exporter.repopulate_warehouse()
