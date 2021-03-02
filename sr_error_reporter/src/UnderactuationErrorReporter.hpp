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

#include <geometry_msgs/Transform.h>
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
  const ros::NodeHandle& node_handle_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotStatePtr robot_state_;
  ros::Subscriber joints_subsriber_, trajectory_subsriber_left_, trajectory_subsriber_right_;

  /**
   * MoveIt link name to radians. From /joint_states topic.
   */
  std::map<std::string, double> actual_joint_angles_;

  /**
   * MoveIt link name to radians. From /[lh|rh]_trajectory_controller/command topic.
   */
  std::map<std::string, double> desired_joint_angles_;

  std::map<std::string, geometry_msgs::Transform> actual_tip_transforms_;
  std::map<std::string, geometry_msgs::Transform> desired_tip_transforms_;

  std::set<std::string> sides_;

  std::map<std::string, std::string> include_fingers_ =
  {
    {
      "ff",
      "first_finger"
    }
    ,
    {
      "mf",
      "middle_finger"
    }
    ,
    {
      "rf",
      "ring_finger"
    }
    ,
    {
      "lf",
      "little_finger"
    }
  };

  std::vector<std::string> joint_names_ =
  {
    "tip",
    "distal",
    "middle",
    "proximal",
    "knuckle"
  };

  void update_kinematic_model(
    std::map<std::string, double>& joint_positions,
    std::map<std::string, geometry_msgs::Transform>& transforms);

  void publish_error();

  /**
   * Update given joint position map with new value if joint name
   * if for underactuated finger.
   *
   * @param joint_positions Map of link name to position for updating.
   * @param name For example "rh_FFJ2".
   * @param position New position.
   */
  void update_joint_position(
    std::map<std::string, double>& joint_positions,
    std::string name,
    double position);

  void joints_callback(const sensor_msgs::JointStateConstPtr& msg);
  void trajectory_callback(const trajectory_msgs::JointTrajectory& msg);
};

#endif  // SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_
