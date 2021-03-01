/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_
#define SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_

#include <map>
#include <set>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

class UnderactuationErrorReporter
{
 public:
  explicit UnderactuationErrorReporter(ros::NodeHandle& node_handle);
 private:
  const ros::NodeHandle& node_handle;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  ros::Subscriber joints_subsriber, trajectory_subsriber_left, trajectory_subsriber_right;

  /**
   * MoveIt link name to radians. From /joint_states topic.
   */
  std::map<std::string, double> actual_joint_angles;

  /**
   * MoveIt link name to radians. From /[lh|rh]_trajectory_controller/command topic.
   */
  std::map<std::string, double> desired_joint_angles;

  std::set<std::string> include_fingers =
  {
    "FF",
    "MF",
    "RF",
    "LF"
  };

  std::vector<std::string> joint_names =
  {
    "tip",
    "distal",
    "middle",
    "proximal",
    "knuckle"
  };

  void publish_error();
  void update_joint_position(
    std::map<std::string, double>& joint_positions,
    std::string name,
    double position);
  void joints_callback(const sensor_msgs::JointStateConstPtr& msg);
  void trajectory_callback(const trajectory_msgs::JointTrajectory& msg);
};

#endif  // SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_
