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

//#include <cstdlib>
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_coupling/chain_coupling.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_coupling/chainiksolverpos_nr_jl_coupling.hpp>
#include <kdl_coupling/chainiksolvervel_wdls_coupling.hpp>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>
#include <string>
#include <vector>
using std::string;

static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

#define IK_EPS	1e-5

Eigen::MatrixXd updateCouplingFF(const KDL::JntArray& q)
{
  Eigen::MatrixXd cm(4,3);
  for (unsigned int i =0; i<4; i++)
    for (unsigned int j=0; j<3; j++)
      cm(i,j) = 0.0;

  cm(0,0) = 1.0;  // J4
  cm(1,1) = 1.0;  // J3
  cm(2,2) = 1.0;  // J2
  cm(3,2) = 1.0;  // J1

  return cm;
}
Eigen::MatrixXd updateCouplingMF(const KDL::JntArray& q)
{
  Eigen::MatrixXd cm(4,3);
  for (unsigned int i =0; i<4; i++)
    for (unsigned int j=0; j<3; j++)
      cm(i,j) = 0.0;

  cm(0,0) = 1.0;  // J4
  cm(1,1) = 1.0;  // J3
  cm(2,2) = 1.0;  // J2
  cm(3,2) = 1.0;  // J1

  return cm;
}
Eigen::MatrixXd updateCouplingRF(const KDL::JntArray& q)
{
  Eigen::MatrixXd cm(4,3);
  for (unsigned int i =0; i<4; i++)
    for (unsigned int j=0; j<3; j++)
      cm(i,j) = 0.0;

  cm(0,0) = 1.0;  // J4
  cm(1,1) = 1.0;  // J3
  cm(2,2) = 1.0;  // J2
  cm(3,2) = 1.0;  // J1

  return cm;
}
Eigen::MatrixXd updateCouplingLF(const KDL::JntArray& q)
{
  Eigen::MatrixXd cm(5,4);
  for (unsigned int i =0; i<5; i++)
    for (unsigned int j=0; j<4; j++)
      cm(i,j) = 0.0;

  cm(0,0) = 1.0;  // J5
  cm(1,1) = 1.0;  // J4
  cm(2,2) = 1.0;  // J3
  cm(3,3) = 1.0;  // J2
  cm(4,3) = 1.0;  // J1

  return cm;
}
Eigen::MatrixXd updateCouplingTH(const KDL::JntArray& q)
{
  // There is no coupling in the thumb. So the coupling matrix is the identity matrix
  return Eigen::MatrixXd::Identity(5,5);
}

class Kinematics {
public:
  Kinematics();
  bool init();

private:
  ros::NodeHandle nh, nh_private;
  std::string root_name, finger_base_name, tip_name;
  KDL::JntArray joint_min, joint_max;
  KDL::Chain_coupling chain;
  unsigned int num_joints;

  KDL::ChainFkSolverPos_recursive* fk_solver;
  KDL::ChainIkSolverPos_NR_JL_coupling *ik_solver_pos;
  KDL::ChainIkSolverVel_wdls_coupling *ik_solver_vel;

  ros::ServiceServer ik_service,ik_solver_info_service;
  ros::ServiceServer fk_service,fk_solver_info_service;

  tf::TransformListener tf_listener;

  moveit_msgs::KinematicSolverInfo info;

  bool loadModel(const std::string xml);
  bool readJoints(urdf::Model &robot_model);
  int getJointIndex(const std::string &name);
  int getKDLSegmentIndex(const std::string &name);

  /**
   * @brief This is the basic IK service method that will compute and return an IK solution.
   * @param A request message. See service definition for GetPositionIK for more information on this message.
   * @param The response message. See service definition for GetPositionIK for more information on this message.
   */
  bool getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                     moveit_msgs::GetPositionIK::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                       moveit_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                       moveit_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This method generates a random joint array vector between the joint limits so that local minima in IK can be avoided.
   * @param Joint vector to be initialized with random values.
   */
  void generateRandomJntSeed(KDL::JntArray &jnt_pos_in);
  /**
   * @brief This is the basic forward kinematics service that will return information about the kinematics node.
   * @param A request message. See service definition for GetPositionFK for more information on this message.
   * @param The response message. See service definition for GetPositionFK for more information on this message.
   */
  bool getPositionFK(moveit_msgs::GetPositionFK::Request &request,
                     moveit_msgs::GetPositionFK::Response &response);
};



Kinematics::Kinematics(): nh_private ("~") {
}

bool Kinematics::init() {
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
  nh.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get Root and Tip From Parameter Service
  if (!nh_private.getParam("root_name", root_name)) {
    ROS_FATAL("GenericIK: No root name found on parameter server");
    return false;
  }
  if(root_name.find("palm")==string::npos) {
    ROS_FATAL("Current solver can only resolve to root frame = palm");
    return false;
  }

  if (!nh_private.getParam("tip_name", tip_name)) {
    ROS_FATAL("GenericIK: No tip name found on parameter server");
    return false;
  }

  if(tip_name.find("tip")==string::npos) {
    ROS_FATAL("Current solver can only resolve to one of the tip frames");
    return false;
  }
  if(tip_name.find("fftip")==string::npos && tip_name.find("mftip")==string::npos && tip_name.find("rftip")==string::npos && tip_name.find("lftip")==string::npos && tip_name.find("thtip")==string::npos){
    ROS_FATAL("Name of distal frame does not match any finger");
    return false;
  }

  // Load and Read Models
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Define coupling matrix for fingers ff, mf, rf: their first two joints (J1 and J2) are coupled while J3 and J4 are independent.
  // The rows of coupling matrix correspond to all joints (unlocked ones) while the columns correspond to independent joints (not coupled).
  if(tip_name.find("fftip")!=string::npos)
  {
    // Assign update function for dynamic coupling
    chain.setUpdateCouplingFunction(updateCouplingFF);
  }
  if(tip_name.find("mftip")!=string::npos)
  {
    // Assign update function for dynamic coupling
    chain.setUpdateCouplingFunction(updateCouplingMF);
  }
  if(tip_name.find("rftip")!=string::npos)
  {
    // Assign update function for dynamic coupling
    chain.setUpdateCouplingFunction(updateCouplingRF);
  }
  if(tip_name.find("lftip")!=string::npos)
  {
    // Assign update function for dynamic coupling
    chain.setUpdateCouplingFunction(updateCouplingLF);
  }
  if(tip_name.find("thtip")!=string::npos)
  {
    // Assign update function for thumb: identity matrix is used since there is no coupling
    chain.setUpdateCouplingFunction(updateCouplingTH);
  }

  Eigen::MatrixXd Mx(6,6);  // Task space weighting matrix: We will only consider translation components.
  for(unsigned int i=0; i < 6; i++)
  {
    for(unsigned int j=0; j < 6; j++)
    {
      Mx(i,j)= 0.0;
    }
  }
  // Control only position of the fingertip. Discard error in orientation
  Mx(0,0)= 1.0;  // coordinate X
  Mx(1,1)= 1.0;  // coordinate Y
  Mx(2,2)= 1.0;  // coordinate Z
  Mx(3,3)= 0.0;  // rotation X
  Mx(4,4)= 0.0;  // rotation Y
  Mx(5,5)= 0.0;  // rotation Z

  ROS_DEBUG("CHAIN--> Joints:%d, Ind. Joints:%d, Segments:%d",chain.getNrOfJoints(),chain.getNrOfIndJoints(),chain.getNrOfSegments());
  // Get Solver Parameters
  int maxIterations;
  double epsilon, lambda;

	if (!nh_private.getParam("maxIterations", maxIterations))
	{
		maxIterations= 1000;
		ROS_WARN("No maxIterations on param server, using %d as default",maxIterations);
	}

	if (!nh_private.getParam("epsilon", epsilon))
	{
		epsilon= 1e-2;
		ROS_WARN("No epsilon on param server, using %f as default",epsilon);
	}

	if (!nh_private.getParam("lambda", lambda))
	{
		lambda= 0.01;
		ROS_WARN("No lambda on param server, using %f as default",lambda);
	}

  ROS_DEBUG("IK Solver, maxIterations: %d, epsilon: %f, lambda: %f",maxIterations, epsilon, lambda);

  // Build Solvers
  fk_solver = new KDL::ChainFkSolverPos_recursive(chain);  // keep the standard arm_kinematics fk_solver
  ik_solver_vel= new KDL::ChainIkSolverVel_wdls_coupling(chain,epsilon,maxIterations);
  ik_solver_vel->setLambda(lambda);
  ik_solver_vel->setWeightTS(Mx);
  ik_solver_pos= new KDL::ChainIkSolverPos_NR_JL_coupling(chain,joint_min,joint_max,*fk_solver, *ik_solver_vel, maxIterations, epsilon);

  ROS_INFO("Advertising services");
  fk_service = nh_private.advertiseService(FK_SERVICE,&Kinematics::getPositionFK,this);
  ik_service = nh_private.advertiseService(IK_SERVICE,&Kinematics::getPositionIK,this);
  ik_solver_info_service = nh_private.advertiseService(IK_INFO_SERVICE,&Kinematics::getIKSolverInfo,this);
  fk_solver_info_service = nh_private.advertiseService(FK_INFO_SERVICE,&Kinematics::getFKSolverInfo,this);

  return true;
}

bool Kinematics::loadModel(const std::string xml) {
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, chain)) {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model)) {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }

  return true;
}

bool Kinematics::readJoints(urdf::Model &robot_model) {
  num_joints = 0;
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  boost::shared_ptr<const urdf::Joint> joint;

  urdf::Vector3 length;

  while (link && link->name != root_name) {
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


int Kinematics::getJointIndex(const std::string &name) {
  for (unsigned int i=0; i < info.joint_names.size(); i++) {
    if (info.joint_names[i] == name)
      return i;
  }
  return -1;
}

int Kinematics::getKDLSegmentIndex(const std::string &name) {
  int i=0;
  while (i < (int)chain.getNrOfSegments()) {
    if (chain.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

void Kinematics::generateRandomJntSeed(KDL::JntArray &jnt_pos_in)
{
  for(unsigned int i=0; i < num_joints; i++)
  {
    double min= info.limits[i].min_position;
    double max= info.limits[i].max_position;
    double r= min + ((double)rand()) / RAND_MAX *(max-min);
    jnt_pos_in(i)= r;
  }
}
bool Kinematics::getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                               moveit_msgs::GetPositionIK::Response &response) {

  if((request.ik_request.ik_link_name.find("fftip")==std::string::npos) && (request.ik_request.ik_link_name.find("mftip")==std::string::npos)
     && (request.ik_request.ik_link_name.find("rftip")==std::string::npos) && (request.ik_request.ik_link_name.find("lftip")==std::string::npos) && (request.ik_request.ik_link_name.find("thtip")==std::string::npos))
  {
    ROS_ERROR("Only IK at fingertip frame can be computed\n");
    return false;
  }

  geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
  tf::Stamped<tf::Pose> transform;
  tf::Stamped<tf::Pose> transform_root;
  tf::poseStampedMsgToTF( pose_msg_in, transform );

 // Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(num_joints);
  for (unsigned int i=0; i < num_joints; i++) {
    int tmp_index = getJointIndex(request.ik_request.robot_state.joint_state.name[i]);
    if (tmp_index >=0) {
      jnt_pos_in(tmp_index) = request.ik_request.robot_state.joint_state.position[i];
    } else {
      ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.robot_state.joint_state.name[i].c_str());
    }
  }
 // Convert F to our root_frame
  try {
    tf_listener.transformPose(root_name, transform, transform_root);
  } catch (...) {
    ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  KDL::Frame F_dest;
  tf::transformTFToKDL(transform_root, F_dest);
  int ik_valid= -1;
  for(int i=0; i < 10 && ik_valid < 0; i++)
  {
		if(request.ik_request.ik_link_name.find("thtip")!=std::string::npos || request.ik_request.ik_link_name.find("lftip")!=std::string::npos)
			ROS_DEBUG("IK Seed: %f, %f, %f, %f, %f",jnt_pos_in(0),jnt_pos_in(1),jnt_pos_in(2),jnt_pos_in(3),jnt_pos_in(4));
		else
			ROS_DEBUG("IK Seed: %f, %f, %f, %f",jnt_pos_in(0),jnt_pos_in(1),jnt_pos_in(2),jnt_pos_in(3));
    ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);
    generateRandomJntSeed(jnt_pos_in);
    // maintain 1:1 coupling
    if(request.ik_request.ik_link_name.find("thtip")==std::string::npos && request.ik_request.ik_link_name.find("lftip")==std::string::npos)
    {
			jnt_pos_in(3)=jnt_pos_in(2);
		}
		else if(request.ik_request.ik_link_name.find("lftip")!=std::string::npos )
			jnt_pos_in(4)=jnt_pos_in(3);
		if(i>0)
			ROS_DEBUG("IK Recalculation step: %d",i);
  }
  if (ik_valid >= 0) {
    response.solution.joint_state.name = info.joint_names;
    response.solution.joint_state.position.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
      response.solution.joint_state.position[i] = jnt_pos_out(i);
      ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
    }
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  } else {
    ROS_DEBUG("An IK solution could not be found");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return true;
  }
}

bool Kinematics::getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response) {
  response.kinematic_solver_info = info;
  return true;
}

bool Kinematics::getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response) {
  response.kinematic_solver_info = info;
  return true;
}

bool Kinematics::getPositionFK(moveit_msgs::GetPositionFK::Request &request,
                               moveit_msgs::GetPositionFK::Response &response) {

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(num_joints);
  for (unsigned int i=0; i < num_joints; i++) {
    int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
    if (tmp_index >=0)
      jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
  }

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());

  bool valid = true;
  for (unsigned int i=0; i < request.fk_link_names.size(); i++) {
    int segmentIndex = Kinematics::getKDLSegmentIndex(request.fk_link_names[i]);
    ROS_DEBUG("End effector index: %d",segmentIndex);
    ROS_DEBUG("Chain indices: %d",chain.getNrOfSegments());

    int fk_valid= fk_solver->JntToCart(jnt_pos_in,p_out,segmentIndex);

    if (fk_valid >=0) {
      tf_pose.frame_id_ = root_name;
      tf_pose.stamp_ = ros::Time();
      tf::poseKDLToTF(p_out,tf_pose);
      try {
        tf_listener.transformPose(request.header.frame_id,tf_pose,tf_pose);
      } catch (...) {
        ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return false;
      }
      tf::poseStampedTFToMsg(tf_pose,pose);
      response.pose_stamped[i] = pose;
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    } else {
      ROS_ERROR("Could not compute FK for %s",request.fk_link_names[i].c_str());
      response.error_code.val = response.error_code.FAILURE;
      valid = false;
    }
  }
  return valid;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hand_kinematics_coupling");
  Kinematics k;
  if (k.init()<0) {
    ROS_ERROR("Could not initialize kinematics node");
    return -1;
  }

  ros::spin();
  return 0;
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
