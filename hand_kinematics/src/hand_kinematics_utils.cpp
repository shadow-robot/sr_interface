#include <hand_kinematics/hand_kinematics_utils.h>

namespace hand_kinematics
{
	bool loadRobotModel(ros::NodeHandle node_handle, urdf::Model &robot_model, std::string &root_name, std::string &tip_name, std::string &xml_string)
  {
    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
    node_handle.searchParam(urdf_xml,full_urdf_xml);
    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (node_handle.getParam(full_urdf_xml, result))
      xml.Parse(result.c_str());
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
    if (!node_handle.getParam("root_name", root_name)){
      ROS_FATAL("HANDIK: No root name found on parameter server");
      return false;
    }
    
   	if(root_name!="palm") {
			ROS_FATAL("HANDIK: Current solver can only resolve to root frame = palm");
			return false;
		}
	
    if (!node_handle.getParam("tip_name", tip_name)){
      ROS_FATAL("HANDIK: No tip name found on parameter server");
      return false;
    }
    
    if(tip_name.find("tip")==std::string::npos) {
			ROS_FATAL("Current solver can only resolve to one of the tip frames");
			return false;
		}
		if(tip_name.find("fftip")==std::string::npos && tip_name.find("mftip")==std::string::npos && tip_name.find("rftip")==std::string::npos && tip_name.find("lftip")==std::string::npos && tip_name.find("thtip")==std::string::npos){
			ROS_FATAL("Name of distal frame does not match any finger");
			return false;	
		}
        
    return true;
  }

	bool getKDLChain(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
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
      ROS_ERROR("Could not initialize chain object");
      return false;
    }
    return true;
  }

  bool getKDLTree(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Tree &kdl_tree)
  {
    // create robot chain from root to tip
    if (!kdl_parser::treeFromString(xml_string, kdl_tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    return true;
  }

	
	int getJointIndex(const std::string &name, const kinematics_msgs::KinematicSolverInfo &chain_info)
	{
		for (unsigned int i=0; i < chain_info.joint_names.size(); i++) {
        if (chain_info.joint_names[i] == name)
            return i;
    }
    return -1;
	}

	void getKDLChainInfo(const KDL::Chain &chain,
                       kinematics_msgs::KinematicSolverInfo &chain_info)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      chain_info.link_names.push_back(chain.getSegment(i).getName());
      i++;
    }
  }

	int getKDLSegmentIndex(const KDL::Chain &chain,
                         const std::string &name)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      if(chain.getSegment(i).getName() == name)
      {
        return i+1;
      }
      i++;
    }
    return -1;
  }


	Eigen::MatrixXd updateCouplingFF(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(4,3);
		for (unsigned int i =0; i<4; i++)
			for (unsigned int j=0; j<3; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J4
		cm(1,1) = 1.0; // J3
		cm(2,2) = 1.0; // J2
		cm(3,2) = 1.0; // J1

		return cm;
	}
	Eigen::MatrixXd updateCouplingMF(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(4,3);
		for (unsigned int i =0; i<4; i++)
			for (unsigned int j=0; j<3; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J4
		cm(1,1) = 1.0; // J3
		cm(2,2) = 1.0; // J2
		cm(3,2) = 1.0; // J1

		return cm;
	}
	Eigen::MatrixXd updateCouplingRF(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(4,3);
		for (unsigned int i =0; i<4; i++)
			for (unsigned int j=0; j<3; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J4
		cm(1,1) = 1.0; // J3
		cm(2,2) = 1.0; // J2
		cm(3,2) = 1.0; // J1

		return cm;
	}
	Eigen::MatrixXd updateCouplingLF(const KDL::JntArray& q)
	{
		Eigen::MatrixXd cm(5,4);
		for (unsigned int i =0; i<5; i++)
				for (unsigned int j=0; j<4; j++)
							cm(i,j) = 0.0;

		cm(0,0) = 1.0; // J5
		cm(1,1) = 1.0; // J4
		cm(2,2) = 1.0; // J3
		cm(3,3) = 1.0; // J2
		cm(4,3) = 1.0; // J1

		return cm;
	}
	Eigen::MatrixXd updateCouplingTH(const KDL::JntArray& q)
	{
		// There is no coupling in the thumb. So the coupling matrix is the identity matrix
		return Eigen::MatrixXd::Identity(5,5);
	}

  bool init_ik(urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name, KDL::JntArray &joint_min, KDL::JntArray &joint_max, kinematics_msgs::KinematicSolverInfo &info )
  {
    unsigned int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    urdf::Vector3 length;

    while (link && link->name != root_name) 
    {
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint) {
          ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
          return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
          ROS_DEBUG( "adding joint: [%s]", joint->name.c_str() );
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
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_DEBUG( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;
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

/*
  	if (!readJoints(robot_model)) {
				ROS_FATAL("Could not read information about the joints");
				return false;
		}
  
  */



}
