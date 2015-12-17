#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <eigen_conversions/eigen_msg.h>

class TestIKsolvers
{
  bool found_ik;
  moveit::planning_interface::MoveGroup group;

  public:
  TestIKsolvers();
    ~TestIKsolvers();
  void AddSceneObjects(void);
  bool TestRandomState(void);
};

TestIKsolvers::TestIKsolvers():group("right_arm")
{
}

TestIKsolvers::~TestIKsolvers()
{
  ros::shutdown();
}

bool TestIKsolvers::TestRandomState(void)
{
  robot_state::RobotStatePtr test_state = group.getCurrentState();

  group.setStartState(*group.getCurrentState());
  geometry_msgs::PoseStamped random_pose =  group.getRandomPose();
  //ROS_INFO_STREAM("Pose:"<< random_pose.pose);

  found_ik = test_state->setFromIK(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),
      random_pose.pose, 10, 0.1);

  //ROS_INFO_STREAM("IK solution " << (found_ik ? "found" : "not found"));

  if (found_ik)
  {
    /*moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setJointValueTarget(*test_state);
    bool success = group.plan(my_plan);
    ROS_INFO("Visualizing plan  %s", success?"SUCCEDED":"FAILED");*/

    return true;

  }
  else
  {
    ROS_INFO("Did not find IK solution");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing iksolvers");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int num_succeeded_positions = 0;

  TestIKsolvers *testIKsolvers = new TestIKsolvers();
  int total_random_queries = 10000;
  for (std::size_t i = 0; i < total_random_queries; i++)
  {
    bool succeed = testIKsolvers->TestRandomState();
    if (succeed)
    {
      num_succeeded_positions ++;
    }
  }
  ROS_INFO_STREAM("Number of valid ik plans " << num_succeeded_positions << "/" << total_random_queries);

  ros::waitForShutdown();
  return 0;
}

