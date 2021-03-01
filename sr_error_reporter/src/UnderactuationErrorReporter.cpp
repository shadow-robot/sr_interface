/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include "UnderactuationErrorReporter.hpp"

UnderactuationErrorReporter::UnderactuationErrorReporter(ros::NodeHandle& node_handle)
  : node_handle(node_handle)
{
  joints_subsriber = node_handle.subscribe("/joint_states", 1,
    &UnderactuationErrorReporter::joints_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subsriber_left = node_handle.subscribe("/lh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());
  trajectory_subsriber_right = node_handle.subscribe("/rh_trajectory_controller/command", 1,
    &UnderactuationErrorReporter::trajectory_callback, this, ros::TransportHints().tcpNoDelay());
  ROS_ERROR_STREAM("callback added");

  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  moveit::core::RobotModelPtr kinematic_model_ = robot_model_loader_->getModel();

  if (kinematic_model_ == NULL)
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
  }
}

void UnderactuationErrorReporter::publish_error()
{
  ROS_ERROR_STREAM("Actual:");
  for (auto& joint : actual_joint_angles)
  {
    ROS_ERROR_STREAM("\t" << joint.first << ": " << joint.second);
  }
  ROS_ERROR_STREAM("Desired:");
  for (auto& joint : desired_joint_angles)
  {
    ROS_ERROR_STREAM("\t" << joint.first << ": " << joint.second);
  }
  // TODO: publish error
}

void UnderactuationErrorReporter::update_joint_position(
  std::map<std::string, double>& joint_positions,
  std::string name,
  double position)
{
  if (name.size() >= 7)
  {
    std::string side = name.substr(0, 3);
    std::string finger_name = name.substr(3, 2);
    std::transform(finger_name.begin(), finger_name.end(), finger_name.begin(), [](unsigned char c)
    {
      return std::tolower(c);
    });
    // TODO: filter out thumb and wrist
    int joint_index = std::atoi(name.substr(6, 1).c_str()) - 1;
    if (joint_index >= 0 && joint_index <= 4)
    {
      joint_positions[side + finger_name + joint_names[joint_index]] = position;
    }
  }
}

void UnderactuationErrorReporter::joints_callback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++) {
    update_joint_position(actual_joint_angles, msg->name[i], msg->position[i]);
  }
  publish_error();
}

void UnderactuationErrorReporter::trajectory_callback(const trajectory_msgs::JointTrajectory& msg) {
  for (int i = 0; i < msg.joint_names.size(); i++) {
    update_joint_position(desired_joint_angles, msg.joint_names[i], msg.points[0].positions[i]);
  }
  publish_error();
}
