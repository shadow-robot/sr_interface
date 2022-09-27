#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022 belongs to Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy

from sr_robot_msgs.srv import PlanTrajectoryFromList as PlanFromList
from sr_robot_msgs.srv import PlanTrajectoryFromPrefix as PlanFromPrefix
from sr_robot_msgs.srv import ExecutePlannedTrajectory as ExecutePlan
from sr_robot_msgs.srv import PlanNamedTrajectory as PlanNamed
from sr_robot_msgs.srv import ListNamedTrajectories as ListNamed
from functools import partial
from builtins import map
from copy import deepcopy


class WaypointNamedServices():
    def __init__(self):

        rospy.init_node('waypoint_named_services')

        rospy.loginfo("Waiting for planning services...")
        rospy.wait_for_service('plan_trajectory_from_list')
        rospy.wait_for_service('plan_trajectory_from_prefix')
        rospy.wait_for_service('execute_planned_trajectory')

        self.__from_list = rospy.ServiceProxy(
            'plan_trajectory_from_list', PlanFromList)
        self.__from_prefix = rospy.ServiceProxy(
            'plan_trajectory_from_prefix', PlanFromPrefix)
        self.__execute_plan = rospy.ServiceProxy(
            'execute_planned_trajectory', PlanFromPrefix)
        rospy.loginfo("Service proxies connected.")

        self.service_mapping = rospy.get_param("~service_mapping")
        self.define_services()

    def __plan_named_trajectory(self, req):
        mapping = [
            x for x in self.service_mapping if x["name"] ==
            req.name]
        if len(mapping) > 1:
            rospy.logfatal("Trajectory name is not unique")
        elif len(mapping) is 0:
            rospy.logfatal("Trajectory name does not exist")
        else:
            m = mapping[0]

            if "list" not in m.keys() and "prefix" not in m.keys():
                rospy.logfatal("Service must specify either prefix " +
                               "or list for choosing waypoints")
            elif "list" in m.keys() and "prefix" in m.keys():
                rospy.logfatal("Service mapping has both list and " +
                               "prefix specified. Can only be one.")
            else:
                if "list" in m.keys():
                    return self.__from_list(m["list"]).success
                else:
                    return self.__from_prefix(m["prefix"]).success

        return False

    def __list_named_trajectories(self, req):
        return [map(lambda x: x["name"], self.service_mapping)]

    def define_services(self):
        plan_service_name = rospy.get_param("~plan_named_trajectory_service")
        list_service_name = rospy.get_param("~list_named_trajectories_service")

        self.__plan_server = rospy.Service(plan_service_name, PlanNamed,
                                           self.__plan_named_trajectory)
        self.__list_server = rospy.Service(list_service_name, ListNamed,
                                           self.__list_named_trajectories)


if __name__ == "__main__":
    sf = WaypointNamedServices()
    rospy.spin()
