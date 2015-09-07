/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */
/* Modified by Guillaume Walck for Shadow Hands */

#include <hand_kinematics/hand_kinematics_utils.h>

namespace hand_kinematics
{
  static const double IK_DEFAULT_TIMEOUT = 10.0;

  bool loadRobotModel(ros::NodeHandle node_handle, urdf::Model &robot_model, std::string &root_name,
                      std::string &tip_name, std::string &xml_string)
  {
    std::string urdf_xml, full_urdf_xml;
    node_handle.param("urdf_xml", urdf_xml, std::string("robot_description"));
    node_handle.searchParam(urdf_xml, full_urdf_xml);
    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (node_handle.getParam(full_urdf_xml, result))
    {
      xml.Parse(result.c_str());
    }
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
      return false;
    }
    xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if (!root || !root_element)
    {
      ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
      exit(1);
    }
    robot_model.initXml(root);

    if (root_name.find("palm") == std::string::npos)
    {
      ROS_FATAL("HANDIK: Current solver can only resolve to root frame = palm");
      return false;
    }

    if (tip_name.find("tip") == std::string::npos)
    {
      ROS_FATAL("Current solver can only resolve to one of the tip frames");
      return false;
    }
    if (tip_name.find("fftip") == std::string::npos && tip_name.find("mftip") == std::string::npos &&
        tip_name.find("rftip") == std::string::npos && tip_name.find("lftip") == std::string::npos &&
        tip_name.find("thtip") == std::string::npos)
    {
      ROS_FATAL("Name of distal frame does not match any finger");
      return false;
    }

    return true;
  }

  bool getKDLChain(const std::string &xml_string, const std::string &root_name, const std::string &tip_name,
                   KDL::Chain &kdl_chain)
  {
    // create robot chain from root to tip
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml_string, tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, kdl_chain))
    {
      ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
      return false;
    }
    return true;
  }

  bool getKDLTree(const std::string &xml_string, const std::string &root_name, const std::string &tip_name,
                  KDL::Tree &kdl_tree)
  {
    // create robot chain from root to tip
    if (!kdl_parser::treeFromString(xml_string, kdl_tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    return true;
  }


  void getKDLChainInfo(const KDL::Chain &chain,
                       moveit_msgs::KinematicSolverInfo &chain_info)
  {
    int i = 0;  // segment number
    while (i < (int) chain.getNrOfSegments())
    {
      chain_info.link_names.push_back(chain.getSegment(i).getName());
      i++;
    }
  }

  int getKDLSegmentIndex(const KDL::Chain &chain,
                         const std::string &name)
  {
    int i = 0;  // segment number
    while (i < (int) chain.getNrOfSegments())
    {
      if (chain.getSegment(i).getName() == name)
      {
        return i + 1;
      }
      i++;
    }
    return -1;
  }


  bool checkJointNames(const std::vector<std::string> &joint_names,
                       const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for (unsigned int i = 0; i < chain_info.joint_names.size(); i++)
    {
      int index = -1;
      for (unsigned int j = 0; j < joint_names.size(); j++)
      {
        if (chain_info.joint_names[i] == joint_names[j])
        {
          index = j;
          break;
        }
      }
      if (index < 0)
      {
        ROS_ERROR("Joint state does not contain joint state for %s.", chain_info.joint_names[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool checkLinkNames(const std::vector<std::string> &link_names,
                      const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if (link_names.empty())
    {
      return false;
    }
    for (unsigned int i = 0; i < link_names.size(); i++)
    {
      if (!checkLinkName(link_names[i], chain_info))
      {
        return false;
      }
    }
    return true;
  }

  bool checkLinkName(const std::string &link_name,
                     const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for (unsigned int i = 0; i < chain_info.link_names.size(); i++)
    {
      if (link_name == chain_info.link_names[i])
      {
        return true;
      }
    }
    return false;
  }

  bool checkRobotState(moveit_msgs::RobotState &robot_state,
                       const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if ((int) robot_state.joint_state.position.size() != (int) robot_state.joint_state.name.size())
    {
      ROS_ERROR(
              "Number of joints in robot_state.joint_state does not match number of positions in robot_state.joint_state");
      return false;
    }
    if (!checkJointNames(robot_state.joint_state.name, chain_info))
    {
      ROS_ERROR("Robot state must contain joint state for every joint in the kinematic chain");
      return false;
    }
    return true;
  }

  bool checkFKService(moveit_msgs::GetPositionFK::Request &request,
                      moveit_msgs::GetPositionFK::Response &response,
                      const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if (!checkLinkNames(request.fk_link_names, chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for.");
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if (!checkRobotState(request.robot_state, chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    return true;
  }

  bool checkIKService(moveit_msgs::GetPositionIK::Request &request,
                      moveit_msgs::GetPositionIK::Response &response,
                      const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    if (!checkLinkName(request.ik_request.ik_link_name, chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for.");
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if (!checkRobotState(request.ik_request.robot_state, chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    if (request.ik_request.timeout <= ros::Duration(0.0))
    {
      response.error_code.val = response.error_code.TIMED_OUT;
      return false;
    }
    return true;
  }

  int getJointIndex(const std::string &name, const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for (unsigned int i = 0; i < chain_info.joint_names.size(); i++)
    {
      if (chain_info.joint_names[i] == name)
      {
        return i;
      }
    }
    return -1;
  }

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg,
                              KDL::Frame &pose_kdl,
                              const std::string &root_frame,
                              tf::TransformListener &tf)
  {
    geometry_msgs::PoseStamped pose_stamped;
    if (!convertPoseToRootFrame(pose_msg, pose_stamped, root_frame, tf))
    {
      return false;
    }
    tf::poseMsgToKDL(pose_stamped.pose, pose_kdl);
    return true;
  }


  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg,
                              geometry_msgs::PoseStamped &pose_msg_out,
                              const std::string &root_frame,
                              tf::TransformListener &tf)
  {
    geometry_msgs::PoseStamped pose_msg_in = pose_msg;
    ROS_DEBUG("Request:\nframe_id: %s\nPosition: %f %f %f\n:Orientation: %f %f %f %f\n",
              pose_msg_in.header.frame_id.c_str(),
              pose_msg_in.pose.position.x,
              pose_msg_in.pose.position.y,
              pose_msg_in.pose.position.z,
              pose_msg_in.pose.orientation.x,
              pose_msg_in.pose.orientation.y,
              pose_msg_in.pose.orientation.z,
              pose_msg_in.pose.orientation.w);
    pose_msg_out = pose_msg;
    tf::Stamped<tf::Pose> pose_stamped;
    poseStampedMsgToTF(pose_msg_in, pose_stamped);

    if (!tf.canTransform(root_frame, pose_stamped.frame_id_, pose_stamped.stamp_))
    {
      std::string err;
      if (tf.getLatestCommonTime(pose_stamped.frame_id_, root_frame, pose_stamped.stamp_, &err) != tf::NO_ERROR)
      {
        ROS_ERROR("hand_ik:: Cannot transform from '%s' to '%s'. TF said: %s", pose_stamped.frame_id_.c_str(),
                  root_frame.c_str(), err.c_str());
        return false;
      }
    }
    try
    {
      tf.transformPose(root_frame, pose_stamped, pose_stamped);
    }
    catch (...)
    {
      ROS_ERROR("hand_ik:: Cannot transform from '%s' to '%s'", pose_stamped.frame_id_.c_str(), root_frame.c_str());
      return false;
    }
    tf::poseStampedTFToMsg(pose_stamped, pose_msg_out);
    return true;
  }

  Eigen::MatrixXd updateCouplingFF(const KDL::JntArray &q)
  {
    Eigen::MatrixXd cm(4, 3);
    for (unsigned int i = 0; i < 4; i++)
    {
      for (unsigned int j = 0; j < 3; j++)
      {
        cm(i, j) = 0.0;
      }
    }

    cm(0, 0) = 1.0;  // J4
    cm(1, 1) = 1.0;  // J3
    cm(2, 2) = 1.0;  // J2
    cm(3, 2) = 1.0;  // J1

    return cm;
  }

  Eigen::MatrixXd updateCouplingMF(const KDL::JntArray &q)
  {
    Eigen::MatrixXd cm(4, 3);
    for (unsigned int i = 0; i < 4; i++)
    {
      for (unsigned int j = 0; j < 3; j++)
      {
        cm(i, j) = 0.0;
      }
    }

    cm(0, 0) = 1.0;  // J4
    cm(1, 1) = 1.0;  // J3
    cm(2, 2) = 1.0;  // J2
    cm(3, 2) = 1.0;  // J1

    return cm;
  }

  Eigen::MatrixXd updateCouplingRF(const KDL::JntArray &q)
  {
    Eigen::MatrixXd cm(4, 3);
    for (unsigned int i = 0; i < 4; i++)
    {
      for (unsigned int j = 0; j < 3; j++)
      {
        cm(i, j) = 0.0;
      }
    }

    cm(0, 0) = 1.0;  // J4
    cm(1, 1) = 1.0;  // J3
    cm(2, 2) = 1.0;  // J2
    cm(3, 2) = 1.0;  // J1

    return cm;
  }

  Eigen::MatrixXd updateCouplingLF(const KDL::JntArray &q)
  {
    Eigen::MatrixXd cm(5, 4);
    for (unsigned int i = 0; i < 5; i++)
    {
      for (unsigned int j = 0; j < 4; j++)
      {
        cm(i, j) = 0.0;
      }
    }

    cm(0, 0) = 1.0;  // J5
    cm(1, 1) = 1.0;  // J4
    cm(2, 2) = 1.0;  // J3
    cm(3, 3) = 1.0;  // J2
    cm(4, 3) = 1.0;  // J1

    return cm;
  }

  Eigen::MatrixXd updateCouplingTH(const KDL::JntArray &q)
  {
    // There is no coupling in the thumb. So the coupling matrix is the identity matrix
    return Eigen::MatrixXd::Identity(5, 5);
  }

  bool init_ik(urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name,
               KDL::JntArray &joint_min, KDL::JntArray &joint_max, moveit_msgs::KinematicSolverInfo &info)
  {
    unsigned int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    urdf::Vector3 length;

    while (link && link->name != root_name)
    {
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint)
      {
        ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
        return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG("adding joint: [%s]", joint->name.c_str());
        num_joints++;
      }
      link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints)
    {
      joint = robot_model.getJoint(link->parent_joint->name);
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG("getting bounds for joint: [%s]", joint->name.c_str());

        float lower, upper;
        int hasLimits;
        if (joint->type != urdf::Joint::CONTINUOUS)
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
          hasLimits = 1;
        }
        else
        {
          lower = -M_PI;
          upper = M_PI;
          hasLimits = 0;
        }
        int index = num_joints - i - 1;
        joint_min.data[index] = lower;
        joint_max.data[index] = upper;
        info.joint_names[index] = joint->name;
        info.link_names[index] = link->name;
        info.limits[index].joint_name = joint->name;
        info.limits[index].has_position_limits = hasLimits;
        info.limits[index].min_position = lower;
        info.limits[index].max_position = upper;
        i++;
      }
      link = robot_model.getLink(link->getParent()->name);
    }
    return true;

  }
}
