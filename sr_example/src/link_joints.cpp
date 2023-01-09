/**
* @file   link_joints.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu Jul  8 16:57:22 2010
*
* Software License Agreement (BSD License)
* Copyright © 2011, 2022-2023 belongs to Shadow Robot Company Ltd.
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
*
* This example demonstrates how two joints can have their positions 'linked' by
* having a child joint subscribing to the parent joint's controller state topic.
* The messages are then published to the child joint's controller.
*
* The hand should be launched but the trajectory controllers should
* not be running as they overwrite position commands. To check if they are, a call
* to rosservice can be made with the following command:
* >rosservice call /controller_manager/list_controllers
* To stop the trajectory controllers, open the gui (type rqt) and select position control in
* plugins > Shadow Robot > Change controllers
* NOTE: If the joint sliders plugin is open during this change of controllers, it will
* need to be reloaded.
*
* A position can then be published to the parent joint or the joint could be moved by
* using the joint sliders in the gui, plugins > Shadow Robot > joint slider
*
* If you move the joint slider for FFJ3, then MFJ3 will move as well.
*/

#include <ros/ros.h>
#include <string>

// messages
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

/// the name of the parent joint
const char* parent_name = "rh_ffj3";
/// the name of the child joint to link to the parent
const char* child_name = "rh_mfj3";
/// Controller that controls joint position
const char* controller_type = "_position_controller";

// a ros subscriber (will be instantiated later on)
ros::Subscriber sub;
// a ros publisher (will be instantiated later on)
ros::Publisher pub;

/**
* The callback function is called each time a message is received on the
* controller
*
* @param msg message of type sr_hand::joints_data
*/
void callback(const control_msgs::JointControllerStateConstPtr &msg)
{
  // publish the message
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
int main(int argc, char **argv)
{
  // init the ros node
  ros::init(argc, argv, "link_joints_example");
  ros::NodeHandle node;

  /**
   * init the subscriber and subscribe to the
   * parent joint controller topic using the callback function
   * callback()
   */
  sub = node.subscribe(std::string("sh_") + parent_name + controller_type + "/state", 2, callback);

  /**
   * init the publisher on the child joint controller command topic
   * publishing messages of the type std_msgs::Float64.
   */
  pub = node.advertise<std_msgs::Float64>(std::string("sh_") + child_name + controller_type + "/command", 2);

  // subscribe until interrupted.
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
