#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Adding walls and ground
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  geometry_msgs::Pose box_pose;

  collision_object.id = "ground";
  primitive.dimensions[0] = 3.0;
  primitive.dimensions[1] = 3.0;
  primitive.dimensions[2] = 0.1;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z =  -0.11;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  collision_object.id = "wall_front";
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.01;
  box_pose.position.x =  0.4;
  box_pose.position.y = 0.85;
  box_pose.position.z =  0.4;
  box_pose.orientation.x = 0.5;
  box_pose.orientation.y = -0.5;
  box_pose.orientation.z = 0.5;
  box_pose.orientation.w = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  collision_object.id = "wall_right";
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 1.8;
  primitive.dimensions[2] = 0.01;
  box_pose.position.x =  1.33;
  box_pose.position.y = 0.4;
  box_pose.position.z =  0.4;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = -0.707388;
  box_pose.orientation.z = 0.0;
  box_pose.orientation.w = 0.706825;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  collision_object.id = "wall_left";
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 1.6;
  primitive.dimensions[2] = 0.01;
  box_pose.position.x =  -0.5;
  box_pose.position.y = 0.4;
  box_pose.position.z =  0.4;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = -0.707107;
  box_pose.orientation.z = 0.0;
  box_pose.orientation.w = 0.707107;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan  %s", success?"":"FAILED");
  sleep(5.0);

  // Collision checking
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  bool in_collision;
  robot_state::RobotState random_state = planning_scene.getCurrentState();

  //robot_state::RobotState& random_state = planning_scene.getCurrentStateNonConst();
  random_state = planning_scene.getCurrentState();
  random_state.setToRandomPositions();

  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, random_state);
  ROS_INFO_STREAM("Random state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " collision");
  // Alternative
  random_state = planning_scene.getCurrentState();
  random_state.setToRandomPositions();
  in_collision = planning_scene.isStateColliding(random_state);
  ROS_INFO_STREAM("Random state is " << (in_collision ? "in collision" : "not in collision"));

  group.setStartState(*group.getCurrentState());
  group.setRandomTarget();
  success = group.plan(my_plan);
  ROS_INFO("Visualizing plan  %s", success?"":"FAILED");
  sleep(5.0);

  ros::shutdown();
  return 0;
}
