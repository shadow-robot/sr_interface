/**
 * @file   test_iksolvers.cpp
 * @author Shadow Software Team <software@shadowrobot.com>, Contact <contact@shadowrobot.com>
 *
 *
 * Copyright 2015 Shadow Robot Company Ltd.
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

