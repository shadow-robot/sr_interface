/*
* Software License Agreement (BSD License)
* Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
*      may be used to endorse or promote products derived from this software without
*      specific prior written permission.
*
* This software is provided by Shadow Robot Company Ltd "as is" and any express
* or implied warranties, including, but not limited to, the implied warranties of
* merchantability and fitness for a particular purpose are disclaimed. In no event
* shall the copyright holder be liable for any direct, indirect, incidental, special,
* exemplary, or consequential damages (including, but not limited to, procurement of
* substitute goods or services; loss of use, data, or profits; or business interruption)
* however caused and on any theory of liability, whether in contract, strict liability,
* or tort (including negligence or otherwise) arising in any way out of the use of this
* software, even if advised of the possibility of such damage.
*/

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "std_msgs/String.h"
#include "sr_error_reporter/UnderactuationError.h"
#include "sr_error_reporter/UnderactuationErrorReporter.hpp"
#include "sr_utilities_common/wait_for_param.h"

UnderactuationErrorReporter::UnderactuationErrorReporter(ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
{
  trajectory_subscriber_left_ = node_handle.subscribe("/lh_trajectory_controller/state", 1,
    &UnderactuationErrorReporter::trajectory_callback_left, this, ros::TransportHints().tcpNoDelay());
  trajectory_subscriber_right_ = node_handle.subscribe("/rh_trajectory_controller/state", 1,
    &UnderactuationErrorReporter::trajectory_callback_right, this, ros::TransportHints().tcpNoDelay());

  if (!wait_for_param(node_handle, "robot_description"))
  {
    ROS_ERROR("UnderactuationErrorReporter did't find robot_description on parameter server");
  }

  if (wait_for_param(node_handle, "robot_description_semantic"))
  {
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_state_.reset(new robot_state::RobotState(robot_model_loader_->getModel()));
  }
  else
  {
    ROS_ERROR("UnderactuationErrorReporter did't find robot_description_semantic on parameter server");
  }
}

ros::Publisher UnderactuationErrorReporter::get_or_create_publisher(
  std::string side,
  std::string finger_name)
{
  std::string key = side + finger_name;
  auto iterator = error_publishers_.find(key);
  if (iterator != error_publishers_.end())
  {
    return iterator->second;
  }
  ros::Publisher publisher = node_handle_.advertise<sr_error_reporter::UnderactuationError>(
    "/sh_" + side + finger_name + "j0_position_controller/underactuation_cartesian_error", 1);
  error_publishers_[key] = publisher;
  return publisher;
}

void UnderactuationErrorReporter::update_kinematic_model(
  std::string side,
  std::map<std::string, double>& joint_positions,
  std::map<std::string, geometry_msgs::Transform>& transforms)
{
  robot_state_->setToDefaultValues();
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
    // Set all joint angles to 0
    std::vector<double> positions = {0, 0};
    if (finger.first == "lf")
    {
      // Little finger has 5 joints instead of 4
      positions.push_back(0);
    }
    // except for underactuated joins which we want to measure
    positions.push_back(j2->second);
    positions.push_back(j1->second);
    robot_state_->setJointGroupPositions(side + finger.second, positions);

    std::string link_name = side + finger.first + "tip";
    // Forward kinemetics
    const Eigen::Affine3d &end_effector_state = robot_state_->getGlobalLinkTransform(link_name);
    geometry_msgs::Transform transform;
    tf::transformEigenToMsg(end_effector_state, transform);
    transforms[link_name] = transform;
  }
}

void UnderactuationErrorReporter::publish_error(
  std::string side,
  std::map<std::string, geometry_msgs::Transform> actual_tip_transforms,
  std::map<std::string, geometry_msgs::Transform> desired_tip_transforms)
{
  for (auto& actual : actual_tip_transforms)
  {
    std::string link_name = actual.first;
    auto desired = desired_tip_transforms.find(link_name);
    if (desired != desired_tip_transforms.end())
    {
      double x = actual.second.translation.x - desired->second.translation.x;
      double y = actual.second.translation.y - desired->second.translation.y;
      double z = actual.second.translation.z - desired->second.translation.z;
      double error = std::sqrt(x * x + y * y + z * z);

      sr_error_reporter::UnderactuationError underactuation_error;
      underactuation_error.header.stamp = ros::Time::now();
      underactuation_error.error = error;
      std::string finger_name = link_name.substr(3, 2);
      get_or_create_publisher(side, finger_name)
        .publish(underactuation_error);
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
    joint_positions[side + finger_name + joint_names_[joint_index]] = position;
  }
}

void UnderactuationErrorReporter::trajectory_callback_left(
  const control_msgs::JointTrajectoryControllerState& msg)
{
  handle_trajectory_message("lh_", msg);
}

void UnderactuationErrorReporter::trajectory_callback_right(
  const control_msgs::JointTrajectoryControllerState& msg)
{
  handle_trajectory_message("rh_", msg);
}

void UnderactuationErrorReporter::handle_trajectory_message(
  std::string side,
  const control_msgs::JointTrajectoryControllerState& msg)
{
  std::map<std::string, double> actual_joint_angles;
  std::map<std::string, double> desired_joint_angles;
  for (int i = 0; i < msg.joint_names.size(); i++)
  {
    update_joint_position(actual_joint_angles, msg.joint_names[i], msg.actual.positions[i]);
    update_joint_position(desired_joint_angles, msg.joint_names[i], msg.desired.positions[i]);
  }
  std::map<std::string, geometry_msgs::Transform> actual_tip_transforms;
  std::map<std::string, geometry_msgs::Transform> desired_tip_transforms;
  update_kinematic_model(side, actual_joint_angles, actual_tip_transforms);
  update_kinematic_model(side, desired_joint_angles, desired_tip_transforms);
  publish_error(side, actual_tip_transforms, desired_tip_transforms);
}
