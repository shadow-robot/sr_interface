/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */
/* Modified by Guillaume WALCK for Shadow Hands*/

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

// HandKinematics
#include <hand_kinematics/hand_kinematics_plugin.h>
// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

#define IK_NEAR 3e-3
#define IK_NEAR_TRANSLATE 1e-5

class MyTest
{
public:
  bool initialize()
  {
    double search_discretization;
    ros::NodeHandle nh("~");
    kinematics_loader_.reset(
            new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));

    std::string plugin_name;
    if (!nh.getParam("plugin_name", plugin_name))
    {
      ROS_ERROR("No plugin name found on parameter server");
      EXPECT_TRUE(0);
      return false;
    }
    ROS_INFO("Plugin name: %s", plugin_name.c_str());
    try
    {
      kinematics_solver_ = kinematics_loader_->createUniqueInstance(plugin_name);
    }
    catch (pluginlib::PluginlibException &ex)  // handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
      EXPECT_TRUE(0);
      return false;
    }
    std::string root_name, tip_name, finger_group_name;
    ros::WallTime start_time = ros::WallTime::now();
    bool done = true;
    while ((ros::WallTime::now() - start_time).toSec() <= 5.0)
    {
      if (!nh.getParam("root_name", root_name))
      {
        ROS_ERROR("No root name found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      if (!nh.getParam("tip_name", tip_name))
      {
        ROS_ERROR("No tip name found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      if (!nh.getParam("search_discretization", search_discretization))
      {
        ROS_ERROR("No search discretization found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      if (!nh.getParam("group_name", finger_group_name))
      {
        ROS_ERROR("No group name given, using fingers");
        finger_group_name = "fingers";
        continue;
      }


      done = true;
    }

    if (!done)
    {
      return false;
    }

    if (kinematics_solver_->initialize("robot_description", finger_group_name, root_name, tip_name,
                                       search_discretization))
    {
      return true;
    }
    else
    {
      EXPECT_TRUE(0);
      return false;
    }
  };

  void joint_state_callback(const geometry_msgs::Pose &ik_pose,
                            const std::vector<double> &joint_state,
                            moveit_msgs::MoveItErrorCodes &error_code)
  {
    std::vector<std::string> link_names;
    link_names.push_back(kinematics_solver_->getTipFrame());
    std::vector<geometry_msgs::Pose> solutions;
    solutions.resize(1);
    if (!kinematics_solver_->getPositionFK(link_names, joint_state, solutions))
    {
      error_code.val = error_code.PLANNING_FAILED;
    }
    if (solutions[0].position.z > 0.0)
    {
      error_code.val = error_code.SUCCESS;
    }
    else
    {
      error_code.val = error_code.PLANNING_FAILED;
    }
  };

  kinematics::KinematicsBasePtr kinematics_solver_;
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
};

MyTest my_test;

TEST(HandIKPlugin, initialize)
{
  ASSERT_TRUE(my_test.initialize());
  // Test getting chain information
  std::string root_name = my_test.kinematics_solver_->getBaseFrame();
  EXPECT_TRUE(root_name == std::string("rh_palm"));
  std::string tool_name = my_test.kinematics_solver_->getTipFrame();
  EXPECT_TRUE(tool_name.find("tip") != std::string::npos);
  std::vector<std::string> joint_names = my_test.kinematics_solver_->getJointNames();

  if (tool_name.find("fftip") != std::string::npos)
  {
    EXPECT_EQ(static_cast<int>(joint_names.size()), 4);
    EXPECT_TRUE(joint_names[0].find("FFJ4") != std::string::npos);
    EXPECT_TRUE(joint_names[1].find("FFJ3") != std::string::npos);
    EXPECT_TRUE(joint_names[2].find("FFJ2") != std::string::npos);
    EXPECT_TRUE(joint_names[3].find("FFJ1") != std::string::npos);
  }
  if (tool_name.find("mftip") != std::string::npos)
  {
    EXPECT_EQ(static_cast<int>(joint_names.size()), 4);
    EXPECT_TRUE(joint_names[0].find("MFJ4") != std::string::npos);
    EXPECT_TRUE(joint_names[1].find("MFJ3") != std::string::npos);
    EXPECT_TRUE(joint_names[2].find("MFJ2") != std::string::npos);
    EXPECT_TRUE(joint_names[3].find("MFJ1") != std::string::npos);
  }
  if (tool_name.find("rftip") != std::string::npos)
  {
    EXPECT_EQ(static_cast<int>(joint_names.size()), 4);
    EXPECT_TRUE(joint_names[0].find("RFJ4") != std::string::npos);
    EXPECT_TRUE(joint_names[1].find("RFJ3") != std::string::npos);
    EXPECT_TRUE(joint_names[2].find("RFJ2") != std::string::npos);
    EXPECT_TRUE(joint_names[3].find("RFJ1") != std::string::npos);
  }
  if (tool_name.find("lftip") != std::string::npos)
  {
    EXPECT_EQ(static_cast<int>(joint_names.size()), 5);
    EXPECT_TRUE(joint_names[0].find("LFJ5") != std::string::npos);
    EXPECT_TRUE(joint_names[1].find("LFJ4") != std::string::npos);
    EXPECT_TRUE(joint_names[2].find("LFJ3") != std::string::npos);
    EXPECT_TRUE(joint_names[3].find("LFJ2") != std::string::npos);
    EXPECT_TRUE(joint_names[4].find("LFJ1") != std::string::npos);
  }
  if (tool_name.find("thtip") != std::string::npos)
  {
    EXPECT_EQ(static_cast<int>(joint_names.size()), 5);
    EXPECT_TRUE(joint_names[0].find("THJ5") != std::string::npos);
    EXPECT_TRUE(joint_names[1].find("THJ4") != std::string::npos);
    EXPECT_TRUE(joint_names[2].find("THJ3") != std::string::npos);
    EXPECT_TRUE(joint_names[3].find("THJ2") != std::string::npos);
    EXPECT_TRUE(joint_names[4].find("THJ1") != std::string::npos);
  }
}

TEST(HandIKPlugin, getFK)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const srdf::ModelSharedPtr &srdf = rdf_loader_.getSRDF();
  const urdf::ModelInterfaceSharedPtr &urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
          my_test.kinematics_solver_->getGroupName());

  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver_->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);

  ros::NodeHandle nh("~");
  int number_fk_tests;
  nh.param("number_fk_tests", number_fk_tests, 100);

  for (unsigned int i = 0; i < (unsigned int) number_fk_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);

    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);

    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);
    bool result_fk = my_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
  }
}

TEST(HandIKPlugin, searchIK)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const srdf::ModelSharedPtr &srdf_model = rdf_loader_.getSRDF();
  const urdf::ModelInterfaceSharedPtr &urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
          my_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver_->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);

  ros::NodeHandle nh("~");
  int number_ik_tests;
  nh.param("number_ik_tests", number_ik_tests, 10);
  unsigned int success = 0;

  std::vector<std::string> joint_names = my_test.kinematics_solver_->getJointNames();

  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < (unsigned int) number_ik_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);

    // make the coupling 1:1 in random values
    if (joint_names[0].find("TH") == std::string::npos && joint_names[0].find("LF") == std::string::npos)
    {
      fk_values[3] = fk_values[2];
    }
    else if (joint_names[0].find("LF") != std::string::npos)
    {
      fk_values[4] = fk_values[3];
    }

    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);
    bool result_fk = my_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);

    bool result = my_test.kinematics_solver_->searchPositionIK(poses[0], seed, timeout, solution, error_code);
    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);
    if (result)
    {
      success++;
      result = my_test.kinematics_solver_->getPositionIK(poses[0], solution, solution, error_code);
      EXPECT_TRUE(result);
    }
    else
    {
      if (fk_names[0].find("TH") != std::string::npos)
        ROS_DEBUG("fk values: %f %f %f %f %f", fk_values[0], fk_values[1], fk_values[2], fk_values[3], fk_values[4]);
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = my_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses);

    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
  }
  ROS_INFO("Success Rate: %f", static_cast<double>(success / number_ik_tests));
  bool success_count = (success > 0.99 * number_ik_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}

TEST(HandIKPlugin, searchIKWithCallbacks)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const srdf::ModelSharedPtr &srdf = rdf_loader_.getSRDF();
  const urdf::ModelInterfaceSharedPtr &urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
          my_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver_->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);

  ros::NodeHandle nh("~");
  int number_ik_tests;
  nh.param("number_ik_tests_with_callbacks", number_ik_tests, 10);
  unsigned int success = 0;
  unsigned int num_actual_tests = 0;
  std::vector<std::string> joint_names = my_test.kinematics_solver_->getJointNames();
  for (unsigned int i = 0; i < (unsigned int) number_ik_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver_->getJointNames().size(), 0.0);

    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);

    // make the coupling 1:1 in random values
    if (joint_names[0].find("TH") == std::string::npos && joint_names[0].find("LF") == std::string::npos)
    {
      fk_values[3] = fk_values[2];
    }
    else if (joint_names[0].find("LF") != std::string::npos)
    {
      fk_values[4] = fk_values[3];
    }

    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);
    bool result_fk = my_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    if (poses[0].position.z < 0.0)
    {
      continue;
    }

    num_actual_tests++;
    bool result = my_test.kinematics_solver_->searchPositionIK(poses[0], seed, timeout, solution,
                                                               boost::bind(&MyTest::joint_state_callback, &my_test, _1,
                                                                           _2, _3), error_code);

    if (result)
    {
      success++;
      std::vector<geometry_msgs::Pose> new_poses;
      new_poses.resize(1);
      result_fk = my_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses);

      EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
      EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
      EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    }

    if (!ros::ok())
    {
      break;
    }
  }
  ROS_INFO("Success with callbacks (%%): %f", static_cast<double>(success / num_actual_tests) * 100.0);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "right_hand_kinematics");
  return RUN_ALL_TESTS();
}
