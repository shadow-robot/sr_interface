/* upmc_fkik_test.cpp
 *
 * test the forward/inverse kinematics for the Shadow hand with
 * J1/J2 finger-couplings, as developed by UPMC.

 * 08.08.2012 - new file
 *
 * (C) 2012 fnh
 */


#include <map>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
// #include <std_msgs/Float64.h>
// #include <pr2_mechanism_msgs/ListControllers.h>



/*
command line access to the fk/ik services:

roscd hand_kinematics
roslaunch launch/hand_kinematics_standalone.launch &
rosservice call /ff_kinematics/get_fk "{header: {frame_id: "palm"}, fk_link_names: ["fftip"], robot_state: {joint_state: {header: { seq: 57313,  stamp: {secs: 1311162659,nsecs: 346940994},  frame_id: ''}, name: ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4'],
 position: [0.3, 0.3, 0.5, 0.2]} } }"

rosservice call /ff_kinematics/get_ik "{
ik_request:{
ik_link_name: "fftip",
pose_stamped:{
 header:{ seq: 0, stamp: {secs: 1311164025, nsecs: 700270985} , frame_id: "palm"},
 pose: {
      position: {
        x: 0.01935,
        y: -0.0627,
        z: 0.1623},
      orientation:{
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 0.1}
}
         },
robot_state: {
joint_state: {
header: { seq: 57313,  stamp: {secs: 1311162659,nsecs: 346940994},  frame_id: ''}, name: ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4'],
position: [0.78,0.78,0.78,0.0] } } },
timeout: 1000}"
*/


#define DEG2RAD  (M_PI / 180.0)
#define RAD2DEG  (180.0 / M_PI)

// the maximum error (in radians) that we tolerate to count IK as a success
#define MAX_DELTA 0.01


double rand_range(double min_n, double max_n) {
  return (double)rand() / RAND_MAX * (max_n - min_n) + min_n;
}


void random_test_finger_fkik( ros::NodeHandle nh, std::string PREFIX, std::string prefix, int n_tests ,bool verbose=false) {
  // create joint-names
  std::string tipName = prefix + "tip";
  bool isLittleFinger = (prefix == "lf"); 
  bool isThumb = (prefix == "th"); 

  std::vector<std::string> jointNames;
  jointNames.push_back( PREFIX + "J1" ); 
  jointNames.push_back( PREFIX + "J2" );
  jointNames.push_back( PREFIX + "J3" );
  jointNames.push_back( PREFIX + "J4" );
  if (isLittleFinger || isThumb) {
    jointNames.push_back( PREFIX + "J5" );
  }

  // we also want a map from joint-name to index, because IK returns
  // the joints in a different order
  // 
  std::map<std::string,int> jointIndexMap; 
  for( unsigned int j=0; j < jointNames.size(); j++ ) {
    jointIndexMap[jointNames[j]] = j;
  }

  // check that the FK/IK services are available for the finger
 // 
  ROS_INFO( "waiting for FK/IK service for finger %s", prefix.c_str() );
  ros::service::waitForService(prefix+"_kinematics/get_fk_solver_info");
  ros::service::waitForService(prefix+"_kinematics/get_fk");
  ros::service::waitForService(prefix+"_kinematics/get_ik_solver_info");
  ros::service::waitForService(prefix+"_kinematics/get_ik");

  // create the service clients; note that we reuse them throughout
  // the whole test for a given finger
 // 
  ros::ServiceClient fk = nh.serviceClient<moveit_msgs::GetPositionFK>(prefix+"_kinematics/get_fk", true);
  ros::ServiceClient ik = nh.serviceClient<moveit_msgs::GetPositionIK>(prefix+"_kinematics/get_ik", true);

  moveit_msgs::GetPositionFK  fkdata;

  // generate n_tests random positions, check that pos(fk) == ik(joints)
  // hardcoded joint limits: J1=J2=J3: 0..90 degrees, J4 -25..25 degrees
  // 
  double j1j2 = 0;
  int n_matched = 0;
  int n_iksolved = 0;
  std::vector<double> jj;
  for( int i=0; i < n_tests; i++ ) {
    jj.resize(0);
    if (isThumb) {  // min-degrees 0 -30 -15 0 -60  max-degrees 90 30 15 75 60
      jj.push_back( rand_range(   0*DEG2RAD,  90*DEG2RAD ) );  // J1
      jj.push_back( rand_range( -30*DEG2RAD,  30*DEG2RAD ) );  // J1
      jj.push_back( rand_range( -15*DEG2RAD,  15*DEG2RAD ) );  // J1
      jj.push_back( rand_range(   0*DEG2RAD,  75*DEG2RAD ) );  // J1
      jj.push_back( rand_range( -60*DEG2RAD,  60*DEG2RAD ) );  // J1
    }
    else {
      jj.push_back( j1j2= rand_range( 0*DEG2RAD, 90*DEG2RAD ) );  // J1
      jj.push_back( j1j2 );    // we need J2==J1 to enforce the joint-coupling
      jj.push_back( rand_range(   0*DEG2RAD, 90*DEG2RAD ) );  // J3
      jj.push_back( rand_range( -25*DEG2RAD, 25*DEG2RAD ));  // J4 abduction
      if (isLittleFinger) {
        jj.push_back( rand_range( 0*DEG2RAD, 40*DEG2RAD ) );  // LFJ5
      }
    }

    // call fk to get the fingertip <prefix+tip> position
   // 
    fkdata.request.header.frame_id = "palm";
    fkdata.request.header.stamp = ros::Time::now();
    fkdata.request.fk_link_names.resize( 1 );
    fkdata.request.fk_link_names.push_back( tipName );
    fkdata.request.robot_state.joint_state.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.frame_id = "";
    fkdata.request.robot_state.joint_state.name.resize( jointNames.size() );
    fkdata.request.robot_state.joint_state.position.resize( jointNames.size() );

    if(verbose)
      ROS_INFO( "FK req fk_link_names[0] is %s", tipName.c_str() );
    for( unsigned int j=0; j < jointNames.size(); j++ ) {
      fkdata.request.robot_state.joint_state.name[j] = jointNames[j];
      fkdata.request.robot_state.joint_state.position[j] = jj[j];
      if(verbose)
        ROS_INFO( "FK req %d joint %d %s   radians %6.2f", i, j, jointNames[j].c_str(), jj[j] );
    }
    fk.call( fkdata );  // (fkeq, fkres );

    // arm_navigation_msgs::ArmNavigationErrorCodes status = fkdata.response.error_code.val;
    int status = fkdata.response.error_code.val;
    if(verbose)
      ROS_INFO( "FK returned status %d", (int) status );

    std::vector<geometry_msgs::PoseStamped> pp = fkdata.response.pose_stamped;
    geometry_msgs::Pose pose = pp[0].pose;  // position.xyz orientation.xyzw
    // string[] fk_link_names
    if (status == fkdata.response.error_code.SUCCESS) {
      if(verbose)
        ROS_INFO( "FK pose is %8.4f %8.4f %8.4f", 
                  pose.position.x,
                  pose.position.y,
                  pose.position.z );
    }
    else {
      continue;  // no use trying IK without FK success
    }

    // now try IK to reconstruct FK angles
    // 
    moveit_msgs::GetPositionIK::Request  ikreq;
    moveit_msgs::GetPositionIK::Response ikres;

    ikreq.ik_request.ik_link_name = tipName;
    ikreq.ik_request.pose_stamped.header.frame_id = "palm";
    ikreq.ik_request.pose_stamped.pose.position.x = pose.position.x;
    ikreq.ik_request.pose_stamped.pose.position.y = pose.position.y;
    ikreq.ik_request.pose_stamped.pose.position.z = pose.position.z;

   // pos is not relevant with the used ik_solver but set it anyway to identity
    ikreq.ik_request.pose_stamped.pose.orientation.x = 0.0;
    ikreq.ik_request.pose_stamped.pose.orientation.y = 0.0;
    ikreq.ik_request.pose_stamped.pose.orientation.z = 0.0;
    ikreq.ik_request.pose_stamped.pose.orientation.w = 1.0;

    ikreq.ik_request.robot_state.joint_state.position.resize( jointNames.size() );
    ikreq.ik_request.robot_state.joint_state.name.resize( jointNames.size() );
    for( unsigned int j=0; j < jointNames.size(); j++ ) {
      ikreq.ik_request.robot_state.joint_state.name[j] = jointNames[j];
    }

    ik.call( ikreq, ikres );
    if (ikres.error_code.val == ikres.error_code.SUCCESS) {
      n_iksolved++;
      if(verbose)
        ROS_INFO( "IK found a solution: " );
      for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++) {
        if(verbose)
          ROS_INFO("Joint: %s %f", ikres.solution.joint_state.name[j].c_str(),
                                 ikres.solution.joint_state.position[j] );
      }
      bool ok = true;
      for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++) {
        std::string name = ikres.solution.joint_state.name[j];
        int           ix = jointIndexMap[ name ];
        double         x = ikres.solution.joint_state.position[j];
        if (fabs( x - jj[ix] ) > MAX_DELTA) {
          if(verbose)
            ROS_INFO( "FK/IK mismatch for joint %d %s,  %6.4f  %6.4f", j, name.c_str(), x, jj[ix] );
          ok = false;
        }
      }
      if (ok) {
        n_matched++;
      }
      if(verbose)
        ROS_INFO( "FK/IK comparison %s", (ok ? "matched" : "failed" ));
    }

    // call fk again but this time with the joint-angles from IK 
   // 
    fkdata.request.header.frame_id = "palm";
    fkdata.request.header.stamp = ros::Time::now();
    fkdata.request.fk_link_names.resize( 1 );
    fkdata.request.fk_link_names.push_back( tipName );
    fkdata.request.robot_state.joint_state.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.frame_id = "";
    fkdata.request.robot_state.joint_state.name.resize( jointNames.size() );
    fkdata.request.robot_state.joint_state.position.resize( jointNames.size() );
    if(verbose)
      ROS_INFO( "FK req fk_link_names[0] is %s", tipName.c_str() );
    for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++ ) {
      std::string name = ikres.solution.joint_state.name[j];
      int           ix = jointIndexMap[ name ];
      double         x = ikres.solution.joint_state.position[j];

      fkdata.request.robot_state.joint_state.name[ix] = jointNames[ix];
      fkdata.request.robot_state.joint_state.position[ix] = x;
      if(verbose)
        ROS_INFO( "FK req %d joint %d %s   radians %6.2f", i, j, jointNames[ix].c_str(), x );
    }
    fk.call( fkdata );  // (fkeq, fkres );

    status = fkdata.response.error_code.val;
    if(verbose)
      ROS_INFO( "FK returned status %d", (int) status );

    std::vector<geometry_msgs::PoseStamped> ppp = fkdata.response.pose_stamped;
    geometry_msgs::Pose pppose = ppp[0].pose;  // position.xyz orientation.xyzw
    // string[] fk_link_names
    if (status == fkdata.response.error_code.SUCCESS) {
      if(verbose)
        ROS_INFO( "FK pose is %8.4f %8.4f %8.4f",
                  pppose.position.x,
                  pppose.position.y,
                  pppose.position.z );
    }
    if(verbose)
      ROS_INFO( "\n" );
  }

  ROS_WARN( "-#- tested %d random positions, %d IK solved, %d matched", n_tests, n_iksolved, n_matched );
}



/**
 * interface and selftest (fk==ik) of the forward/inverse kinematics solver
 * for the Shadow hand with J1/J2 finger-couplings, for the Shadow hand thumb,
 * and support for little-finger palm-arch (LFJ5).
 * The solvers start at the "palm" frame and work towards idealized 
 * "fftip" "mftip" "rftip" "lftip" and "thtip" frames, which are approximations
 * to the core of the fingertips (assuming spherical fingertip shape).
 * Actual contact points on the fingertips are therefore about one-half
 * finger radius outside of the positions specified by the *tip frames.
 */
int main(int argc, char **argv)
{
  ros::init (argc, argv, "upmc_fkik_test" );
  ros::NodeHandle nh;
  bool verbose=false;
  if( argc >1)
  {
    char argument =  (char)(argv[1][0]);
    if(argument=='v')
      verbose=true;
    else
      verbose=false;
  }
  
  int seed = 0;
  while( seed == 0 ) {
    seed = ros::Time::now().sec;  // wait until /clock received
  }
  ROS_INFO( "-#- FK/IK test started, random seed is %d", seed );
  srandom( seed );

  random_test_finger_fkik( nh, "FF", "ff", 1000 ,verbose);
  random_test_finger_fkik( nh, "MF", "mf", 1000 ,verbose);
  random_test_finger_fkik( nh, "RF", "rf", 1000 ,verbose);
  random_test_finger_fkik( nh, "LF", "lf", 1000 ,verbose);
  random_test_finger_fkik( nh, "TH", "th", 1000 ,verbose);
}
