/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <eigen_conversions/eigen_msg.h>

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
  ROS_ERROR_STREAM("callback added");

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_state_.reset(new robot_state::RobotState(robot_model_loader_->getModel()));
  robot_state_->setToDefaultValues();

  /*if (kinematic_model_ == NULL)
  {
    ROS_ERROR("No robot description found");
  }
  else
  {
    ROS_ERROR("ROBOT FOUND!");
    for (auto& group_name : kinematic_model_->getJointModelGroupNames()) {
      ROS_ERROR_STREAM("Group name: " << group_name);
      const robot_model::JointModelGroup *group = kinematic_model_->getJointModelGroup(group_name);
      for (auto& link_name : group->getLinkModelNames()) {
        ROS_ERROR_STREAM("\tLink name: " << link_name);
      }
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
      std::vector<double> positions = {0, 0, 0.3, 0.4};
      ROS_ERROR_STREAM("setJointGroupPositions START");
      robot_state_->setJointGroupPositions("rh_first_finger", positions);
      ROS_ERROR_STREAM("setJointGroupPositions END");
    }
  }
  for (auto& side : sides_)
  {
    for (auto& finger : include_fingers_)
    {
      std::string link_name = side + finger + "tip";
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
  // TODO: calculate and publish error
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
    sides_.insert(side);
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
