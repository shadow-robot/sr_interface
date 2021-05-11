#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.# Software License Agreement (BSD License)

from __future__ import absolute_import
import rospy
import tf
from urdf_parser_py.urdf import URDF


def publish_world_to_base_transform():
    while not rospy.has_param('/robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")
    urdf_str = rospy.get_param('/robot_description')
    robot_urdf = URDF.from_xml_string(urdf_str)
    robot = URDF.from_xml_string(urdf_str)
    robot_root = robot.get_root()

    if robot_root is not None and robot_root != "world":
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            br = tf.TransformBroadcaster()
            br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(), robot_root, "world")
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('virtual_joint_broadcaster', anonymous=True)
    # Publish world to base transform
    try:
        publish_world_to_base_transform()
    except rospy.ROSInterruptException:
        pass
