// bsd license blah blah
// majority of this code comes from package urdf_tool/arm_kinematics
// written by David Lu!!
//
// Modified by Juan A. Corrales, ISIR, UPMC
// -Added support for coupled joints of Shadow Hand. Now, the coupling is a fixed 1:1 value
// but the updateCoupling functions can be modified in order to implement dynamic coupling
// (which can be modified depending on the values of the joint values of the finger).
// -Use of WDLS velocity IK method in order to solve IK only for 3D position. 
// -IK is solved at the fingertip frame, which should be defined in the URDF/Xacro file of the
//  Shadow Hand.

// Modified by Guillaume WALCK (UPMC) 2012 
// - Turned it into a kinematics_plugin

#include <cstring>
#include <hand_kinematics/hand_kinematics_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>

#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/MoveItErrorCodes.h>


using std::string;

static const std::string IK_WITH_COLLISION_SERVICE = "get_constraint_aware_ik";
static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

#define IK_EPS	1e-5


namespace hand_kinematics {
static const double IK_DEFAULT_TIMEOUT = 10.0;
//register the plugin
PLUGINLIB_DECLARE_CLASS(hand_kinematics,HandKinematicsPlugin, hand_kinematics::HandKinematicsPlugin, kinematics::KinematicsBase)

  HandKinematicsPlugin::HandKinematicsPlugin():active_(false){}

	bool HandKinematicsPlugin::isActive()
  {
    if(active_)
      return true;
    return false;
  }

	bool HandKinematicsPlugin::initialize(const std::string& robot_description,
																					const std::string& group_name,
																					const std::string& base_frame,
																					const std::string& tip_frame,
																					double search_discretization)
	{
		setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
		urdf::Model robot_model;
		std::string xml_string;
		ros::NodeHandle private_handle("~/"+group_name);

		while(!loadRobotModel(private_handle,robot_model,base_frame_,tip_frame_,xml_string) && private_handle.ok())
		{
			ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
			ros::Duration(0.5).sleep();
		}

		ROS_DEBUG("Loading KDL Tree");
		if(!getKDLChain(xml_string,base_frame_,tip_frame_,kdl_chain_))
		{
			active_ = false;
			ROS_ERROR("Could not load kdl tree");
		}

		
		// Define coupling matrix for fingers ff, mf, rf: their first two joints (J1 and J2) are coupled while J3 and J4 are independent.
		// The rows of coupling matrix correspond to all joints (unlocked ones) while the columns correspond to independent joints (not coupled).
		if(tip_frame_.find("fftip")!=string::npos)
		{
			// Assign update function for dynamic coupling
			kdl_chain_.setUpdateCouplingFunction(updateCouplingFF);	
			dimension_=4;
		}
		if(tip_frame_.find("mftip")!=string::npos)
		{
			// Assign update function for dynamic coupling
			kdl_chain_.setUpdateCouplingFunction(updateCouplingMF);	
			dimension_=4;
		}
		if(tip_frame_.find("rftip")!=string::npos)
		{
			// Assign update function for dynamic coupling
			kdl_chain_.setUpdateCouplingFunction(updateCouplingRF);	
			dimension_=4;
		}
		if(tip_frame_.find("lftip")!=string::npos)
		{
			// Assign update function for dynamic coupling
			kdl_chain_.setUpdateCouplingFunction(updateCouplingLF);	
			dimension_=5;
		}
		if(tip_frame_.find("thtip")!=string::npos)
		{
			// Assign update function for thumb: identity matrix is used since there is no coupling
			kdl_chain_.setUpdateCouplingFunction(updateCouplingTH);
			dimension_=5;
		}

		Eigen::MatrixXd Mx(6,6); // Task space weighting matrix: We will only consider translation components.
		for(unsigned int i=0; i < 6; i++)
		{
			for(unsigned int j=0; j < 6; j++)
			{
				Mx(i,j)= 0.0;
			}
		}
		// Control only position of the fingertip. Discard error in orientation
		Mx(0,0)= 1.0; // coordinate X
		Mx(1,1)= 1.0; // coordinate Y
		Mx(2,2)= 1.0; // coordinate Z
		Mx(3,3)= 0.0; // rotation X
		Mx(4,4)= 0.0; // rotation Y
		Mx(5,5)= 0.0; // rotation Z
			
		ROS_INFO("CHAIN--> Joints:%d, Ind. Joints:%d, Segments:%d",kdl_chain_.getNrOfJoints(),kdl_chain_.getNrOfIndJoints(),kdl_chain_.getNrOfSegments());
		// Get Solver Parameters
		int maxIterations;
		double epsilon, lambda;

		private_handle.param("maxIterations", maxIterations, 1000);
		private_handle.param("epsilon", epsilon, 1e-2);
		private_handle.param("lambda", lambda, 0.01);
		ROS_DEBUG("IK Solver, maxIterations: %d, epsilon: %f, lambda: %f",maxIterations, epsilon, lambda);
		
		init_ik(robot_model,base_frame_,tip_frame_, joint_min_,joint_max_,ik_solver_info_);
		// Build Solvers
		ROS_DEBUG("Advertising services");
		fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain_); 
		ik_solver_vel= new KDL::ChainIkSolverVel_wdls_coupling(kdl_chain_,epsilon,maxIterations);
		ik_solver_vel->setLambda(lambda);
		ik_solver_vel->setWeightTS(Mx);
		ik_solver_pos= new KDL::ChainIkSolverPos_NR_JL_coupling(kdl_chain_,joint_min_,joint_max_,*fk_solver, *ik_solver_vel, maxIterations, epsilon);
    
		//jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		//pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model, base_frame_,tip_frame_, search_discretization_,free_angle_));
		
		active_ = true;
		return active_;
	}


	bool HandKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
																						 const std::vector<double> &ik_seed_state,
																						 std::vector<double> &solution,
																						 moveit_msgs::MoveItErrorCodes &error_code,
																						 const kinematics::KinematicsQueryOptions &options) const
	{
		if(!active_)
		{
			ROS_ERROR("kinematics not active");
			error_code.val = error_code.NO_IK_SOLUTION;
			return false;
		}

		KDL::Frame pose_desired;
		tf::poseMsgToKDL(ik_pose, pose_desired);

		//Do the IK
		KDL::JntArray jnt_pos_in;
		KDL::JntArray jnt_pos_out;
		jnt_pos_in.resize(dimension_);
		for(int i=0; i < dimension_; i++)
		{
			jnt_pos_in(i) = ik_seed_state[i];
		}
		int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in,
																									 pose_desired,
																									 jnt_pos_out);

		if(ik_valid >= 0)
		{
			solution.resize(dimension_);
			for(int i=0; i < dimension_; i++)
			{
				solution[i] = jnt_pos_out(i);
			}
			error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
			ROS_DEBUG("IK Success");
			return true;
		}
		else
		{
			ROS_DEBUG("An IK solution could not be found");
			error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
			return false;
		}
	}

  bool HandKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
	{
		static IKCallbackFn solution_callback = 0;
		static std::vector<double> consistency_limits;
		return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
	}  
    
  bool HandKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
	{
		static IKCallbackFn solution_callback = 0;
		return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
	}  
	
	bool HandKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
	{
		static std::vector<double> consistency_limits;
		return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
	}
    
  
	bool HandKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
		if(!active_)
		{
			ROS_ERROR("kinematics not active");
			error_code.val = error_code.FAILURE;
			return false;
		}
		if(!consistency_limits.empty() && consistency_limits.size() != (size_t)dimension_)
		{
			ROS_ERROR("Consistency limits should be of size: %d",dimension_);
			error_code.val = error_code.FAILURE;
			return false;
		}

		KDL::Frame pose_desired;
		tf::poseMsgToKDL(ik_pose, pose_desired);

		//Do the IK
		KDL::JntArray jnt_pos_in;
		KDL::JntArray jnt_pos_out;
		jnt_pos_in.resize(dimension_);
		for(int i=0; i < dimension_; i++)
		{
			jnt_pos_in(i) = ik_seed_state[i];
		}

		int ik_valid;
		if(consistency_limits.empty())
		{
			ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in,
																									 pose_desired,
																									 jnt_pos_out);
																									
		}
		else
		{
			ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in,
																									 pose_desired,
																									 jnt_pos_out);
		}

		if(ik_valid >= 0)
		{
			solution.resize(dimension_);
			for(int i=0; i < dimension_; i++)
			{
				solution[i] = jnt_pos_out(i);
			}
			error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
			ROS_DEBUG("IK Success");
			return true;
		}
		else
		{
			ROS_DEBUG("An IK solution could not be found");
			error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
			return false;
		}
	}
	
	bool HandKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
	{
		if(!active_)
		{
			ROS_ERROR("kinematics not active");
			return false;
		}

		KDL::Frame p_out;
		KDL::JntArray jnt_pos_in;
		geometry_msgs::PoseStamped pose;
		tf::Stamped<tf::Pose> tf_pose;

		jnt_pos_in.resize(dimension_);
		for(int i=0; i < dimension_; i++)
		{
			jnt_pos_in(i) = joint_angles[i];
			// ROS_DEBUG("Joint angle: %d %f",i,joint_angles[i]);
		}

		poses.resize(link_names.size());

		bool valid = true;

    for(unsigned int i=0; i < poses.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",hand_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
      if(fk_solver->JntToCart(jnt_pos_in,p_out,hand_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >=0)
      {
        tf::PoseKDLToMsg(p_out,poses[i]);
        /*
           tf_pose.frame_id_ = root_name;
            tf_pose.stamp_ = ros::Time();
            tf::PoseKDLToTF(p_out,tf_pose);
            try {
                tf_listener.transformPose(request.header.frame_id,tf_pose,tf_pose);
            } catch (...) {
                ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
                return false;
            }
            tf::poseStampedTFToMsg(tf_pose,pose);
            response.pose_stamped[i] = pose;
        */
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
        valid = false;
      }
    }
    return valid;
  }

  std::string HandKinematicsPlugin::getBaseFrame()
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
    }
    return base_frame_;
  }

  std::string HandKinematicsPlugin::getToolFrame()
  {
    if(!active_ || ik_solver_info_.link_names.empty())
    {
      ROS_ERROR("kinematics not active");
    }
    return ik_solver_info_.link_names[0];
  }

  const std::vector<std::string> &HandKinematicsPlugin::getJointNames() const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
    }
    return ik_solver_info_.joint_names;
  }

  const std::vector<std::string> &HandKinematicsPlugin::getLinkNames() const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
    }
    return fk_solver_info_.link_names;
  }


}//end namespace 


