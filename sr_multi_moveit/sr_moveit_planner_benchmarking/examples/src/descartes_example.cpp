#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <string>
#include <vector>

typedef std::vector <descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

/**
 * Generates a completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d &pose);

/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d &pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
        toROSJointTrajectory(const TrajectoryVec &trajectory, const descartes_core::RobotModel &model,
                             const std::vector <std::string> &joint_names, double time_delay);

/**
 * Publisher to display planned trajectory in rviz
 */
ros::Publisher pub;

/**
 * Sends a ROS trajectory to planned path topic
 */
bool displayTrajectory(const trajectory_msgs::JointTrajectory &trajectory);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory);

int main(int argc, char **argv) {
    ros::init(argc, argv, "descartes_example");
    ros::NodeHandle nh;
    pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 100);
    // Required for communication with moveit components
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 1. Define sequence of points
    TrajectoryVec points;
    Eigen::Affine3d pose;
    Eigen::Matrix3d m;
    pose = Eigen::Translation3d(1.18456524463, 0.254669798111, 0.0114505933477);
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    pose.linear() = m;
    descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pose);
    points.push_back(pt);

    pose = Eigen::Translation3d(0.576479494895, 0.256191712104, 0.548940634794);
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    pose.linear() = m;
    pt = makeCartesianPoint(pose);
    points.push_back(pt);

    pose = Eigen::Translation3d(0.188919122546, 0.256139968662, 0.549052970929);
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    pose.linear() = m;
    pt = makeCartesianPoint(pose);
    points.push_back(pt);

    pose = Eigen::Translation3d(0.068956487197, 0.627524836336, 0.388852054994);
    m = Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    pose.linear() = m;
    pt = makeCartesianPoint(pose);
    points.push_back(pt);

    // 2. Create a robot model and initialize it
    descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);

    // Name of description on parameter server.
    const std::string robot_description = "robot_description";
    const std::string group_name = "right_arm";
    const std::string world_frame = "/world";
    // tool center point frame (name of link associated with tool)
    const std::string tcp_frame = "ra_ee_link";

    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame)) {
        ROS_INFO("Could not initialize robot model");
        return -1;
    }
    // check for collisions in planning scene
    model->setCheckCollisions(true);
    bool coll = model->getCheckCollisions();
    ROS_INFO_STREAM("Descartes robot enabled collision checks: " << coll);

    // 3. Create a planner and initialize it with our robot model, planner can be either DensePlanner or SparsePlanner
    descartes_planner::DensePlanner planner;
    planner.initialize(model);

    // 4. Feed the trajectory to the planner
    if (!planner.planPath(points)) {
        ROS_ERROR("Could not solve for a valid path");
        return -2;
    }

    TrajectoryVec result;
    if (!planner.getPath(result)) {
        ROS_ERROR("Could not retrieve path");
        return -3;
    }

    // 5. Translate the result into a type that ROS understands
    // Define joint names
    std::vector <std::string> names = {"ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint",
                                       "ra_wrist_1_joint", "ra_wrist_2_joint", "ra_wrist_3_joint", "rh_WRJ1",
                                       "rh_WRJ2"};
    // Generate a ROS joint trajectory with the result path, robot model, given joint names,
    // a certain time delta between each trajectory point
    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);
    // 6. Send the ROS trajectory to the robot for execution
    displayTrajectory(joint_solution);

    ROS_INFO("Done!");
    return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d &pose) {
    using descartes_core::TrajectoryPtPtr;
    using descartes_trajectory::CartTrajectoryPt;
    using descartes_trajectory::TolerancedFrame;

    return TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose)));
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d &pose) {
    using descartes_core::TrajectoryPtPtr;
    using descartes_trajectory::AxialSymmetricPt;
    return TrajectoryPtPtr(new AxialSymmetricPt(pose, M_PI / 2.0 - 0.0001, AxialSymmetricPt::Z_AXIS));
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec &trajectory,
                     const descartes_core::RobotModel &model,
                     const std::vector <std::string> &joint_names,
                     double time_delay) {
    trajectory_msgs::JointTrajectory result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = "/world";
    result.joint_names = joint_names;

    // For keeping track of time-so-far in the trajectory
    double time_offset = 0.0;
    for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it) {
        // Find nominal joint solution at this point
        std::vector<double> joints;
        it->get()->getNominalJointPose(std::vector<double>(), model, joints);
        // Adding zeros for the hand wrist angles
        joints.push_back(0);
        joints.push_back(0);

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = joints;
        pt.velocities.resize(joints.size(), 0.0);
        pt.accelerations.resize(joints.size(), 0.0);
        pt.effort.resize(joints.size(), 0.0);
        pt.time_from_start = ros::Duration(time_offset);
        time_offset += time_delay;

        result.points.push_back(pt);
    }

    return result;
}

bool displayTrajectory(const trajectory_msgs::JointTrajectory &trajectory) {
    // display the planned trajectory in rviz
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectoryPoint pt;
    robot_trajectory.joint_trajectory = trajectory;
    display_trajectory.trajectory.push_back(robot_trajectory);
    pub.publish(display_trajectory);
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory) {
    // Create a Follow Joint Trajectory Action Client
    actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction>
            ac("/ra_trajectory_controller/follow_joint_trajectory", true);
    if (!ac.waitForServer(ros::Duration(2.0))) {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);

    ac.sendGoal(goal);

    if (ac.waitForResult(
            goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5))) {
        ROS_INFO("Action server reported successful execution");
        return true;
    } else {
        ROS_WARN("Action server could not execute trajectory");
        return false;
    }
}
