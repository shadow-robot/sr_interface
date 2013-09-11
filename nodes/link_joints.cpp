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
#include <sr_robot_msgs/joints_data.h>
#include <sr_robot_msgs/joint.h>
#include <sr_robot_msgs/sendupdate.h>

/// the name of the parent joint
std::string parent_name = "FFJ3";
/// the name of the child joint to link to the parent
std::string child_name  = "MFJ3";

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub;
//a ros publisher (will be instantiated later on)
ros::Publisher pub;

/**
 * The callback function is called each time a message is received on the
 * topic /srh/shadowhand_data
 *
 * @param msg message of type sr_hand::joints_data
 */
void callback(const sr_robot_msgs::joints_dataConstPtr& msg)
{
  //loop on all the sendupdate messages received (if > 0)
  int msg_length = msg->joints_list_length;
  if( msg_length == 0)
    {
      ROS_WARN("Received empty message.");
      return;
    }

  //OK, not empty => read the data
  for(unsigned short index_msg=0; index_msg < msg_length; ++index_msg)
    {
      //get the sensor name
      std::string sensor_name = msg->joints_list[index_msg].joint_name;

      /**
       * if it's the parent joint, read the target, and send it to the
       * child.
       */
      if(sensor_name.compare(parent_name) == 0)
	{
	  //get the position (to be set as the target of the child joint)
	  float target = msg->joints_list[index_msg].joint_position;

	  //form a sendupdate msg.
	  sr_robot_msgs::sendupdate msg;
	  std::vector<sr_robot_msgs::joint> jointVector;

	  //fill the message
	  sr_robot_msgs::joint joint;
	  joint.joint_name = child_name;
	  joint.joint_target = target;
	  jointVector.push_back(joint);

	  msg.sendupdate_length = jointVector.size();
	  msg.sendupdate_list = jointVector;

	  //publish the message
	  pub.publish(msg);

	  return;
	}
    }

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
   * init the subscriber and subscribe to the topic
   * /srh/shadowhand_data, using the callback function
   * callback()
   */
  sub = node.subscribe("/srh/shadowhand_data", 2,  callback);

  /**
   * init the publisher on the topic /srh/sendupdate
   * publishing messages of the type sr_robot_msgs::sendupdate.
   */
  pub = node.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);

  //subscribe until interrupted.
  while( ros::ok() )
    ros::spin();

  return 0;
}
