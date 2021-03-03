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

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "std_msgs/String.h"
#include "sr_error_reporter/UnderactuationError.h"
#include "sr_error_reporter/UnderactuationErrorReporter.hpp"

UnderactuationErrorReporter::UnderactuationErrorReporter(ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
{
  joints_subscriber_ = node_handle.subscribe("/joint_states", 1,
    &UnderactuationErrorReporter::joints_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subscriber_left_ = node_handle.subscribe("/lh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subscriber_right_ = node_handle.subscribe("/rh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_state_.reset(new robot_state::RobotState(robot_model_loader_->getModel()));
  robot_state_->setToDefaultValues();
}

ros::Publisher UnderactuationErrorReporter::get_or_create_publisher(std::string link_name)
{
  auto iterator = error_publishers_.find(link_name);
  if (iterator != error_publishers_.end())
  {
    return iterator->second;
  }
  ros::Publisher publisher = node_handle_.advertise<sr_error_reporter::UnderactuationError>(
    "/underactuation_error/" + link_name, 1);
  error_publishers_[link_name] = publisher;
  return publisher;
}

void UnderactuationErrorReporter::update_kinematic_model(
  std::map<std::string, double>& joint_positions,
  std::map<std::string, geometry_msgs::Transform>& transforms)
{
  robot_state_->setToDefaultValues();
  for (auto& side : sides_)
  {
    for (auto& finger : include_fingers_)
    {
      auto j1 = joint_positions.find(side + finger.first + "distal");
      if (j1 == joint_positions.end())
      {
        continue;
      }
      auto j2 = joint_positions.find(side + finger.first + "middle");
      if (j2 == joint_positions.end())
      {
        continue;
      }
      std::vector<double> positions = {0, 0, j2->second, j1->second};
      robot_state_->setJointGroupPositions(side + finger.second, positions);

      std::string link_name = side + finger.first + "tip";
      // Forward kinemetics
      const Eigen::Affine3d &end_effector_state = robot_state_->getGlobalLinkTransform(link_name);
      geometry_msgs::Transform transform;
      tf::transformEigenToMsg(end_effector_state, transform);
      transforms[link_name] = transform;
    }
  }
}

void UnderactuationErrorReporter::publish_error()
{
  for (auto& actual : actual_tip_transforms_)
  {
    std::string link_name = actual.first;
    auto desired = desired_tip_transforms_.find(link_name);
    if (desired != desired_tip_transforms_.end())
    {
      double x = actual.second.translation.x - desired->second.translation.x;
      double y = actual.second.translation.y - desired->second.translation.y;
      double z = actual.second.translation.z - desired->second.translation.z;
      double error = std::sqrt(x * x + y * y + z * z);

      sr_error_reporter::UnderactuationError underactuation_error;
      underactuation_error.header.stamp = ros::Time::now();
      underactuation_error.error = error;
      get_or_create_publisher(link_name).publish(underactuation_error);
    }
  }
}

void UnderactuationErrorReporter::update_joint_position(
  std::map<std::string, double>& joint_positions,
  std::string name,
  double position)
{
  // Expect 7 characters for example: rh_LFJ1
  if (name.size() < 7)
  {
    return;
  }

  std::string finger_name = name.substr(3, 2);
  std::transform(finger_name.begin(), finger_name.end(), finger_name.begin(), [](unsigned char c)
  {
    return std::tolower(c);
  });  // NOLINT Our lint doesn't seem to support lambdas
  if (include_fingers_.find(finger_name) == include_fingers_.end())
  {
    return;
  }

  int joint_index = std::atoi(name.substr(6, 1).c_str());
  if (joint_index > 0 && joint_index <= 4)
  {
    std::string side = name.substr(0, 3);
    sides_.insert(side);
    joint_positions[side + finger_name + joint_names_[joint_index]] = position;
  }
}

void UnderactuationErrorReporter::joints_callback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++)
  {
    update_joint_position(actual_joint_angles_, msg->name[i], msg->position[i]);
  }
  update_kinematic_model(actual_joint_angles_, actual_tip_transforms_);
  publish_error();
}

void UnderactuationErrorReporter::trajectory_callback(const trajectory_msgs::JointTrajectory& msg)
{
  for (int i = 0; i < msg.joint_names.size(); i++)
  {
    if (msg.points.size() > 0)
    {
      update_joint_position(desired_joint_angles_, msg.joint_names[i], msg.points[0].positions[i]);
    }
  }
  update_kinematic_model(desired_joint_angles_, desired_tip_transforms_);
  publish_error();
}
