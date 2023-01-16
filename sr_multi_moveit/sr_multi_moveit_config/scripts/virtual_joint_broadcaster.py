#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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


import rospy
import tf
from urdf_parser_py.urdf import URDF


def publish_world_to_base_transform():
    while not rospy.has_param('/robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")
    urdf_str = rospy.get_param('/robot_description')
    robot = URDF.from_xml_string(urdf_str)
    robot_root = robot.get_root()

    if robot_root is not None and robot_root != "world":
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                                      rospy.Time.now(), robot_root, "world")
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('virtual_joint_broadcaster', anonymous=True)
    # Publish world to base transform
    try:
        publish_world_to_base_transform()
    except rospy.ROSInterruptException:
        pass
