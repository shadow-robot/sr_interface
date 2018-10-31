#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_robot_pose_exporter import SrRobotPoseExporter

"""
The following example shows how to use the warehouse pose exporter to save
robot states from the warehouse as plain text python files. Poses can be
extracted one by one, as a list of pose names, or all poses from a named
trajectory can be extracted together.

Once poses have been extracted, the resulting dictionary can be exported to
 a specified file location. The generated file can then be used as a modules
for importing into subsequent scripts.
"""


# Below is a named trajectory, of the sort used by SrRobotCommander. It is
# assumed that the poses "pose_1" and "pose_2" exist in the warehouse.

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

pose_extractor = SrRobotPoseExtractor()

# The following three snippets will produce the same effect, i.e. to extract
# two poses, named "pose_1" and "pose_2"

# Extract one at a time:
pose_extractor.extract_one_state("pose_1")
pose_extractor.extract_one_state("pose_2")

# Extract a list:
pose_extractor.extract_list(['pose_1', 'pose_2'])

# Extract all poses in a trajectory:
pose_extractor.extract_from_trajetory(trajectory)

# Alternatively, you could extract all poses from the warehouse:
pose_extractor.extract_all()

# Once the poses you requrie are extracted, they can be exported to a module:
pose_extractor.output_module("/tmp/exported_poses.py")
