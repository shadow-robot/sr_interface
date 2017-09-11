/**
 * @file   test_end_effector.cpp
 * @author Guillaume Walck
 *
 * Copyright 2015 Shadow Robot Company Ltd.
*/



/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <moveit/profiler/profiler.h>
#include <ros/package.h>
#include <ros/ros.h>

class LoadRobotModel : public testing::Test
{
protected:
  virtual void SetUp()
  {
    ros::NodeHandle nh("~");

    srdf_model.reset(new srdf::Model());

    std::string xml_string, robot_description_, robot_description_semantic_;

    // wait for move_group to be ready
    ros::WallDuration(8.0).sleep();
    ros::spinOnce();
    if (0)
    {
        ROS_ERROR("Move group not available within 30 seconds, did you start the demo ?");
        return;
    }
    if (!nh.searchParam("robot_description", robot_description_) || !nh.getParam(robot_description_, xml_string))
    {
      ROS_ERROR("Robot model parameter not found! Did you remap '%s' ?", robot_description_.c_str());
      return;
    }
    urdf_model = urdf::parseURDF(xml_string);

    if (!nh.searchParam("robot_description_semantic", robot_description_semantic_) ||
        !nh.getParam(robot_description_semantic_, xml_string))
    {
      ROS_ERROR("Robot model semantic parameter not found! Did you remap '%s' ?", robot_description_semantic_.c_str());
      return;
    }
    srdf_model->initString(*urdf_model, xml_string);

    robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  };

  virtual void TearDown()
  {
  }

protected:
  boost::shared_ptr<urdf::ModelInterface> urdf_model;
  boost::shared_ptr<srdf::Model> srdf_model;
  moveit::core::RobotModelConstPtr robot_model;
};

TEST_F(LoadRobotModel, InitOK)
{
  ASSERT_EQ(urdf_model->getName(), "ur10srh");
  ASSERT_EQ(srdf_model->getName(), "ur10srh");
}

TEST_F(LoadRobotModel, Model)
{
  const std::vector<const moveit::core::JointModelGroup*> &jmgs = robot_model->getJointModelGroups();
  for (std::size_t i = 0 ; i < jmgs.size() ; ++i)
  {
    const std::vector< std::string > &ee_names =  jmgs[i]->getAttachedEndEffectorNames();
    if (jmgs[i]->isChain())
    {
      ASSERT_GT(ee_names.size(), 0) << jmgs[i]->getName();
      for (std::size_t j = 0 ; j < ee_names.size() ; ++j)
      {
        ASSERT_GT(ee_names[j].size(), 0) << jmgs[i]->getName();
      }
    }
  }
  moveit::tools::Profiler::Status();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "end_effector_tester");
    return RUN_ALL_TESTS();
}




