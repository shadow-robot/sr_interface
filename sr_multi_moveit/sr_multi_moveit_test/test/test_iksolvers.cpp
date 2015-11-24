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

    sleep(5.0);

    std::vector<double> group_variable_values;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success1 = group.plan(my_plan);
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    // Now, let's modify one of the joints, plan to the new joint
    // space goal and visualize the plan.
    ROS_INFO("Visualizing plan 2");
    group_variable_values[0] = -2.82755;
    group_variable_values[0] = 3.07975;
    group_variable_values[0] = -1.45497;
    group_variable_values[0] = -1.69038;
    group_variable_values[0] = -1.43346;
    group_variable_values[0] = -1.86694;
    group.setJointValueTarget(group_variable_values);
    success1 = group.plan(my_plan);
    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success1?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);

    // Adding walls and ground
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    geometry_msgs::Pose box_pose;

    collision_object.id = "ground";
    primitive.dimensions[0] = 3.0;
    primitive.dimensions[1] = 3.0;
    primitive.dimensions[2] = 0.01;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z =  -0.11;

    collision_object.primitives.clear();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.clear();
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

    collision_object.primitives.clear();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.clear();
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    collision_object.header.frame_id = group.getPlanningFrame();
    collision_object.id = "wall_right";
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.01;
    box_pose.position.x =  1.33;
    box_pose.position.y = 0.4;
    box_pose.position.z =  0.4;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = -0.707388;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.706825;

    collision_object.primitives.clear();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.clear();
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    collision_object.id = "wall_left";
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.01;
    box_pose.position.x =  -0.5;
    box_pose.position.y = 0.4;
    box_pose.position.z =  0.4;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = -0.707107;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.707107;

    collision_object.primitives.clear();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.clear();
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    sleep(2.0);
    group.setPlanningTime(10.0);

    // Collision checking
    bool in_collision;
    bool success;
    int total_joint_queries = 10;
    int total_queries_not_in_collision;
    int total_successful_plans;
    //moveit::planning_interface::MoveGroup::Plan my_plan;


    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    /*for (std::size_t i = 0; i < total_joint_queries; i++)
    {
        robot_state::RobotState random_state = planning_scene.getCurrentState();
        random_state = planning_scene.getCurrentState();


        random_state.setToRandomPositions();
        const double* joint_values;
        joint_values = random_state.getJointPositions("ra_shoulder_pan_joint");
        ROS_INFO_STREAM("joint0 is " << (joint_values[0]));
        joint_values = random_state.getJointPositions("ra_shoulder_lift_joint");
        ROS_INFO_STREAM("joint1 is " << (joint_values[0]));
        joint_values = random_state.getJointPositions("ra_elbow_joint");
        ROS_INFO_STREAM("joint2 is " << (joint_values[0]));
        joint_values = random_state.getJointPositions("ra_wrist_1_joint");
        ROS_INFO_STREAM("joint3 is " << (joint_values[0]));
        joint_values = random_state.getJointPositions("ra_wrist_2_joint");
        ROS_INFO_STREAM("joint4 is " << (joint_values[0]));
        joint_values = random_state.getJointPositions("ra_wrist_3_joint");
        ROS_INFO_STREAM("joint5 is " << (joint_values[0]));

        in_collision = planning_scene.isStateColliding(random_state);
        ROS_INFO_STREAM("Random state is " << (in_collision ? "in collision" : "not in collision"));

        group.setStartState(*group.getCurrentState());
        group.setRandomTarget();
        success = group.plan(my_plan);
        ROS_INFO("Visualizing plan  %s", success?"SUCCEDED":"FAILED");
        sleep(2.0);
    }*/

    /* Sleep so we have time to see the object in RViz */
    //sleep(2.0);

    ros::shutdown();
    return 0;
}

