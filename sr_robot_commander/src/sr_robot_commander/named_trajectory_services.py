#!/usr/bin/env python3
# Copyright 2019, 2022 Shadow Robot Company Ltd.
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

from builtins import map
import rospy

from sr_robot_msgs.srv import PlanTrajectoryFromList as PlanFromList
from sr_robot_msgs.srv import PlanTrajectoryFromPrefix as PlanFromPrefix
from sr_robot_msgs.srv import PlanNamedTrajectory as PlanNamed
from sr_robot_msgs.srv import ListNamedTrajectories as ListNamed



class WaypointNamedServices:
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
        self.__execute_plan = rospy.ServiceProxy(          # pylint: disable=W0238
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
        elif len(mapping) == 0:
            rospy.logfatal("Trajectory name does not exist")
        else:
            mapping_val = mapping[0]

            if "list" not in mapping_val.keys() and "prefix" not in mapping_val.keys():
                rospy.logfatal("Service must specify either prefix " +
                               "or list for choosing waypoints")
            elif "list" in mapping_val.keys() and "prefix" in mapping_val.keys():
                rospy.logfatal("Service mapping has both list and " +
                               "prefix specified. Can only be one.")
            else:
                if "list" in mapping_val.keys():
                    return self.__from_list(mapping_val["list"]).success
                return self.__from_prefix(mapping_val["prefix"]).success

        return False

    def __list_named_trajectories(self, _):
        return [map(lambda x: x["name"], self.service_mapping)]

    def define_services(self):
        plan_service_name = rospy.get_param("~plan_named_trajectory_service")
        list_service_name = rospy.get_param("~list_named_trajectories_service")

        self.__plan_server = rospy.Service(plan_service_name, PlanNamed,   # pylint: disable=W0238
                                           self.__plan_named_trajectory)
        self.__list_server = rospy.Service(list_service_name, ListNamed,   # pylint: disable=W0238
                                           self.__list_named_trajectories)


if __name__ == "__main__":
    sf = WaypointNamedServices()
    rospy.spin()
