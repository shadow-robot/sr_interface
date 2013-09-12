/**
 * @file   link_joints.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul  8 16:57:22 2010
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief This is an example to show how to get data from the hand,
 * read the position for a specific joint and send this as the target to
 * another joint.
 *
 * To test this program, just start the hand, rviz visualizer, the control GUI
 * and this example (in 4 different consoles):
 * \verbatim
 roslaunch sr_hand srh_motor.launch
 roslaunch sr_hand rviz_motor.launch
 rosrun sr_control_gui __init__.py
 rosrun sr_hand link_joints
 \endverbatim
 * If you move the joint slider for FFJ3, then MFJ3 will move as well.
 *
 *
 *
 */

#include <ros/ros.h>
#include <string>

//messages
#include <std_msgs/Float64.h>
#include <sr_robot_msgs/JointControllerState.h>

/// the name of the parent joint
std::string parent_name = "ffj3";
/// the name of the child joint to link to the parent
std::string child_name  = "mfj3";
/// the type of controller that will be running
std::string controller_type = "_mixed_position_velocity_controller";

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub;
//a ros publisher (will be instantiated later on)
ros::Publisher pub;

/**
 * The callback function is called each time a message is received on the
 * controller
 *
 * @param msg message of type sr_hand::joints_data
 */
void callback(const sr_robot_msgs::JointControllerStateConstPtr& msg)
{
  //publish the message
  std_msgs::Float64 command;
  command.data = msg->set_point;
  pub.publish(command);

  return;
}

/**
 * The main: initialise a ros node, a subscriber and a publisher
 *
 * @param argc
 * @param argv
 *
 * @return 0 on success
 */
int main(int argc, char** argv)
{
  //init the ros node
  ros::init(argc, argv, "link_joints_example");
  ros::NodeHandle node;

  /**
   * init the subscriber and subscribe to the
   * parent joint controller topic using the callback function
   * callback()
   */
  sub = node.subscribe("sh_" + parent_name + controller_type + "/state", 2,  callback);

  /**
   * init the publisher on the child joint controller command topic
   * publishing messages of the type std_msgs::Float64.
   */
  pub = node.advertise<std_msgs::Float64>("sh_" + child_name + controller_type + "/command", 2);

  //subscribe until interrupted.
  while( ros::ok() )
    ros::spin();

  return 0;
}
