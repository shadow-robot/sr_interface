#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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
# POSSIBILITY OF SUCH DAMAGE.


import roslib
import rospy
import tf
from geometry_msgs.msg import Vector3

roslib.load_manifest('hand_kinematics')

if __name__ == '__main__':
    rospy.init_node('tf_tippos_publisher')
    listener = tf.TransformListener()
    tip_pos_pub = []
    tip_pos_pub.append(rospy.Publisher('fftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('mftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('rftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('lftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('thtip/position/', Vector3, queue_size=10))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(
                '/rh_palm', '/rh_fftip', rospy.Time.now(), rospy.Duration(1.0))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_fftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[0].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_mftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[1].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_rftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[2].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_lftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[3].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_thtip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[4].publish(Vector3(trans[0], trans[1], trans[2]))
        except rospy.ROSInterruptException:
            continue

        rate.sleep()
