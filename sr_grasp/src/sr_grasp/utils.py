#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2014, 2022-2023 belongs to Shadow Robot Company Ltd.
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
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint

_sr_joint_names = [
    'FFJ1', 'FFJ2', 'FFJ3', 'FFJ4',
    'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5',
    'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4',
    'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4',
    'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5',
    'WRJ1', 'WRJ2']

_sr_joint_names_j0 = [
    'LFJ0', 'LFJ3', 'LFJ4', 'LFJ5',
    'RFJ0', 'RFJ3', 'RFJ4',
    'MFJ0', 'MFJ3', 'MFJ4',
    'FFJ0', 'FFJ3', 'FFJ4',
    'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5',
    'WRJ1', 'WRJ2']


def _fix_j0(joints):
    """
    Convert joints targets using J1 and J2 to J0, which the controllers use.
    Useful if using joint_states to get grasp joint positions.
    """
    for finger in ['FFJ', 'LFJ', 'MFJ', 'RFJ']:
        if finger + '1' in joints and finger + '2' in joints:
            joints[finger + '0'] = joints[finger + '1'] + joints[finger + '2']
            del joints[finger + '1']
            del joints[finger + '2']


def mk_grasp(joints, pre_joints=None, fix_j0=False):
    """
    Generate a moveit_msgs/Grasp from a set of joint angles given as a dict
    of joint_name -> position.
    """
    if pre_joints is None:
        pre_joints = {}

    sr_joint_names = _sr_joint_names
    if fix_j0:
        _fix_j0(joints)
        _fix_j0(pre_joints)
        sr_joint_names = _sr_joint_names_j0
    now = rospy.Time.now()
    grasp = Grasp()
    grasp.grasp_quality = 0.001
    grasp.grasp_pose.header.frame_id = "forearm"
    grasp.grasp_pose.header.stamp = now
    grasp.grasp_pose.pose.position.x = 0.01
    grasp.grasp_pose.pose.position.y = -0.045
    grasp.grasp_pose.pose.position.z = 0.321
    grasp.grasp_pose.pose.orientation.x = 0
    grasp.grasp_pose.pose.orientation.y = 0
    grasp.grasp_pose.pose.orientation.z = 0
    grasp.grasp_pose.pose.orientation.w = 0
    # pre-grasp (just zero all for now)
    grasp.pre_grasp_posture.header.stamp = now
    grasp.pre_grasp_posture.joint_names = sr_joint_names
    jtp = JointTrajectoryPoint()
    for jname in sr_joint_names:
        if jname in pre_joints:
            jtp.positions.append(pre_joints[jname])
        else:
            jtp.positions.append(0.0)
    grasp.pre_grasp_posture.points.append(jtp)
    # grasp
    grasp.grasp_posture.header.stamp = now
    grasp.grasp_posture.joint_names = sr_joint_names
    jtp = JointTrajectoryPoint()
    for jname in sr_joint_names:
        if jname in joints:
            jtp.positions.append(joints[jname])
        else:
            jtp.positions.append(0.0)
    grasp.grasp_posture.points.append(jtp)
    return grasp
