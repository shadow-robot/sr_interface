/*
* Copyright 2021 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "sr_error_reporter/UnderactuationErrorReporter.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "error_reporter");
  ros::NodeHandle node_handle;
  UnderactuationErrorReporter underactuation_error_reporter(node_handle);
  ros::spin();
  return 0;
}
