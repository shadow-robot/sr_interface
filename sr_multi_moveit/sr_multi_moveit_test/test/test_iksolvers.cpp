/**
* @file   test_iksolvers.cpp
* @author Shadow Software Team <software@shadowrobot.com>, Contact <contact@shadowrobot.com>
*
* Software License Agreement (BSD License)
* Copyright © 2015, 2022-2023 belongs to Shadow Robot Company Ltd.
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <ctime>

class TestIKsolvers
{
  bool found_ik;
  moveit::planning_interface::MoveGroupInterface group;

  public:
  TestIKsolvers();
  void AddSceneObjects(void);
  bool TestRandomState(void);
};

TestIKsolvers::TestIKsolvers():group("right_arm")
{
}

bool TestIKsolvers::TestRandomState(void)
{
  robot_state::RobotStatePtr test_state = group.getCurrentState();

  group.setStartState(*group.getCurrentState());
  geometry_msgs::PoseStamped random_pose =  group.getRandomPose();

  found_ik = test_state->setFromIK(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),
      random_pose.pose, 10, 0.1);

  if (found_ik)
  {
    return true;
  }
  else
  {
    ROS_INFO("Did not find IK solution");
    ROS_INFO_STREAM("Pose:"<< random_pose.pose);
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing iksolvers");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int num_succeeded_positions = 0;
  double maximum = 0.0;
  double minimum = 100;

  double elapsed_secs;

  clock_t global_begin = clock();

  TestIKsolvers *testIKsolvers = new TestIKsolvers();
  int total_random_queries = 10000;
  for (std::size_t i = 0; i < total_random_queries; i++)
  {
    clock_t begin = clock();
    bool succeed = testIKsolvers->TestRandomState();
    if (succeed)
    {
      num_succeeded_positions++;
    }
    clock_t end = clock();
    elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    maximum = (elapsed_secs > maximum)?elapsed_secs:maximum;
    minimum = (elapsed_secs < minimum)?elapsed_secs:minimum;
  }
  clock_t global_end = clock();
  double global_elapsed_secs = static_cast<double>(global_end - global_begin) / CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Number of valid ik plans " << num_succeeded_positions << "/" << total_random_queries);
  ROS_INFO_STREAM("Total time taken " << global_elapsed_secs);
  ROS_INFO_STREAM("Average time per test " << global_elapsed_secs/total_random_queries);
  ROS_INFO_STREAM("Maximum time per test " << maximum);

  ros::shutdown();
  return 0;
}

