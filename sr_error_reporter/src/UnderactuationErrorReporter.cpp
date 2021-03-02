/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <eigen_conversions/eigen_msg.h>
#include "sr_error_reporter/UnderactuationError.h"
#include "std_msgs/String.h"

#include "UnderactuationErrorReporter.hpp"

UnderactuationErrorReporter::UnderactuationErrorReporter(ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
{
  joints_subsriber_ = node_handle.subscribe("/joint_states", 1,
    &UnderactuationErrorReporter::joints_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subsriber_left_ = node_handle.subscribe("/lh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subsriber_right_ = node_handle.subscribe("/rh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_state_.reset(new robot_state::RobotState(robot_model_loader_->getModel()));
  robot_state_->setToDefaultValues();
}

void UnderactuationErrorReporter::add_side(std::string side)
{
  auto result = sides_.insert(side);
  /*if (result.second)
  {
    for (auto& finger : include_fingers_)
    {
      std::string link_name = side + finger.first + "tip";
      std::string topic = "/underactuation_error/" + link_name;
      error_publishers_[link_name] = node_handle_.advertise<sr_error_reporter::UnderactuationError>(topic, 1);
    }
  }*/
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
      auto j2 = joint_positions.find(side + finger.first + "middle");
      std::vector<double> positions = {0, 0, j2->second, j1->second};
      robot_state_->setJointGroupPositions(side + finger.second, positions);
    }
  }
  for (auto& side : sides_)
  {
    for (auto& finger : include_fingers_)
    {
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
  ROS_ERROR_STREAM("Actual:");
  for (auto& joint : actual_joint_angles_)
  {
    ROS_ERROR_STREAM("\t" << joint.first << ": " << joint.second);
  }
  ROS_ERROR_STREAM("Desired:");
  for (auto& joint : desired_joint_angles_)
  {
    ROS_ERROR_STREAM("\t" << joint.first << ": " << joint.second);
  }
  for (auto& actual : actual_tip_transforms_) {
    std::string link_name = actual.first;
    auto desired = desired_tip_transforms_.find(link_name);
    if (desired != desired_tip_transforms_.end()) {
      double x = actual.second.translation.x - desired->second.translation.x;
      double y = actual.second.translation.y - desired->second.translation.y;
      double z = actual.second.translation.z - desired->second.translation.z;
      double error = std::sqrt(x * x + y * y + z * z);
      ROS_ERROR_STREAM("Error for " << link_name << ": " << error);
      auto publisher = error_publishers_.find(link_name);
      if (publisher == error_publishers_.end())
      {
        error_publishers_[link_name] = node_handle_.advertise<sr_error_reporter::UnderactuationError>(
          "/underactuation_error/" + link_name, 1);
        publisher = error_publishers_.find(link_name);
      }
      sr_error_reporter::UnderactuationError underactuation_error;
      underactuation_error.header.stamp = ros::Time::now();
      underactuation_error.error = error;
      publisher->second.publish(underactuation_error);
    }
  }
}

void UnderactuationErrorReporter::update_joint_position(
  std::map<std::string, double>& joint_positions,
  std::string name,
  double position)
{
  if (name.size() < 7)
  {
    return;
  }

  std::string finger_name = name.substr(3, 2);
  std::transform(finger_name.begin(), finger_name.end(), finger_name.begin(), [](unsigned char c)
  {
    return std::tolower(c);
  });
  if (include_fingers_.find(finger_name) == include_fingers_.end())
  {
    return;
  }

  int joint_index = std::atoi(name.substr(6, 1).c_str());
  if (joint_index > 0 && joint_index <= 4)
  {
    std::string side = name.substr(0, 3);
    add_side(side);
    joint_positions[side + finger_name + joint_names_[joint_index]] = position;
  }
}

void UnderactuationErrorReporter::joints_callback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++) {
    update_joint_position(actual_joint_angles_, msg->name[i], msg->position[i]);
  }
  update_kinematic_model(actual_joint_angles_, actual_tip_transforms_);
  publish_error();
}

void UnderactuationErrorReporter::trajectory_callback(const trajectory_msgs::JointTrajectory& msg) {
  for (int i = 0; i < msg.joint_names.size(); i++) {
    if (msg.points.size() > 0) {
      update_joint_position(desired_joint_angles_, msg.joint_names[i], msg.points[0].positions[i]);
    }
  }
  update_kinematic_model(desired_joint_angles_, desired_tip_transforms_);
  publish_error();
}
