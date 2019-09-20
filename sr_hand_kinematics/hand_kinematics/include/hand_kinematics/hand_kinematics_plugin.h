/*********************************************************************
*
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
*
* Author: Sachin Chitta
*        Modified by Guillaume WALCK (UPMC) 2012
* 
*********************************************************************/

#ifndef HAND_KINEMATICS_HAND_KINEMATICS_PLUGIN_H
#define HAND_KINEMATICS_HAND_KINEMATICS_PLUGIN_H

#include <ros/ros.h>
#include <angles/angles.h>

#include <hand_kinematics/hand_kinematics_utils.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <kdl/jntarray.hpp>
#include <kdl_coupling/chain_coupling.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_coupling/chainiksolverpos_nr_jl_coupling.hpp>
#include <kdl_coupling/chainiksolvervel_wdls_coupling.hpp>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>
#include <string>
#include <vector>

#include <tf_conversions/tf_kdl.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <boost/shared_ptr.hpp>
#include <moveit/kinematics_base/kinematics_base.h>

namespace hand_kinematics
{
class HandKinematicsPlugin :
      public kinematics::KinematicsBase
{
public:
/** @class
*  @brief Plugin-able interface to the Shadow hand kinematics
*/
HandKinematicsPlugin();

/**
*  @brief Specifies if the node is active or not
*  @return True if the node is active, false otherwise.
*/
bool isActive();

/**
* @brief Given a desired pose of the end-effector, compute the joint angles to reach it
* @return True if a valid solution was found, false otherwise
*/
virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                           const std::vector<double> &ik_seed_state,
                           std::vector<double> &solution,
                           moveit_msgs::MoveItErrorCodes &error_code,
                           const kinematics::KinematicsQueryOptions &options =
                             kinematics::KinematicsQueryOptions()) const;

virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              std::vector<double> &solution,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options =
                                kinematics::KinematicsQueryOptions()) const;


virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              const std::vector<double> &consistency_limits,
                              std::vector<double> &solution,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options =
                                kinematics::KinematicsQueryOptions()) const;


/**
* @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
* This particular method is intended for "searching" for a solutions by stepping through the redundancy
* (or other numerical routines).
*/
virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              std::vector<double> &solution,
                              const IKCallbackFn &solution_callback,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options =
                                kinematics::KinematicsQueryOptions()) const;

/**
* @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
* This particular method is intended for "searching" for a solutions by stepping through the redundancy
* (or other numerical routines).
*/
virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              const std::vector<double> &consistency_limits,
                              std::vector<double> &solution,
                              const IKCallbackFn &solution_callback,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options =
                                kinematics::KinematicsQueryOptions()) const;


/**
* @brief Given a set of joint angles and a set of links, compute their pose
* @param link_names - set of links for which poses are to be computed
* @param joint_angles - the contains the joint angles
* @param poses - the response contains pose information for all the requested links
* @return True if a valid solution was found, false otherwise
*/
virtual bool getPositionFK(const std::vector<std::string> &link_names,
                           const std::vector<double> &joint_angles,
                           std::vector<geometry_msgs::Pose> &poses) const;


/**
* @brief  Initialization function for the kinematics
* @return True if initialization was successful, false otherwise
*/
virtual bool initialize(const std::string &robot_description,
                        const std::string &group_name,
                        const std::string &base_frame,
                        const std::string &tip_frame,
                        double search_discretization);


/**
* @brief Return all the joint names in the order they are used internally
*/
const std::vector<std::string> &getJointNames() const;

/**
* @brief Return all the link names in the order they are represented internally
*/
const std::vector<std::string> &getLinkNames() const;

protected:
bool active_;
urdf::Model robot_model_;
double search_discretization_;
ros::NodeHandle node_handle_, root_handle_;

KDL::ChainFkSolverPos_recursive *fk_solver;
KDL::ChainIkSolverPos_NR_JL_coupling *ik_solver_pos;
KDL::ChainIkSolverVel_wdls_coupling *ik_solver_vel;
moveit_msgs::KinematicSolverInfo solver_info_;

std::string finger_base_name;
int dimension_;
boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;

KDL::Chain_coupling kdl_chain_;

KDL::JntArray joint_min_, joint_max_;

moveit_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;

/**
* @brief This method generates a random joint array vector between the joint limits so that local minima in IK can be avoided.
* @param Joint vector to be initialized with random values.
*/
void generateRandomJntSeed(KDL::JntArray &jnt_pos_in) const;
};
}  // namespace hand_kinematics

#endif  // HAND_KINEMATICS_HAND_KINEMATICS_PLUGIN_H
