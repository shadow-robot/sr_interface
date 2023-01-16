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

#ifndef SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_
#define SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Transform.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>

class UnderactuationErrorReporter
{
 public:
  explicit UnderactuationErrorReporter(ros::NodeHandle& node_handle);
 private:
  ros::NodeHandle& node_handle_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotStatePtr robot_state_;
  ros::Subscriber trajectory_subscriber_left_, trajectory_subscriber_right_;
  std::map<std::string, ros::Publisher> error_publishers_;

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

  ros::Publisher get_or_create_publisher(
    std::string side,
    std::string finger_name);

  void update_kinematic_model(
    std::string side,
    std::map<std::string, double>& joint_positions,
    std::map<std::string, geometry_msgs::Transform>& transforms);

  void publish_error(
    std::string side,
    std::map<std::string, geometry_msgs::Transform> actual_tip_transforms,
    std::map<std::string, geometry_msgs::Transform> desired_tip_transforms);

  /**
   * Update given joint position map with new value if joint name
   * is for underactuated finger.
   *
   * @param joint_positions Map of link name to position for updating.
   * @param name For example "rh_FFJ2".
   * @param position New position.
   */
  void update_joint_position(
    std::map<std::string, double>& joint_positions,
    std::string name,
    double position);

  void trajectory_callback_left(
    const control_msgs::JointTrajectoryControllerState& msg);
  void trajectory_callback_right(
    const control_msgs::JointTrajectoryControllerState& msg);
  void handle_trajectory_message(
    std::string side,
    const control_msgs::JointTrajectoryControllerState& msg);
};

#endif  // SR_INTERFACE_SR_ERROR_REPORTER_SRC_UNDERACTUATIONERRORREPORTER_HPP_
