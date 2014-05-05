
/* based on upmc_fkik_test.cpp
 *
 * test the forward/inverse kinematics for the Shadow hand with
 * J1/J2 finger-couplings, as developed by UPMC.

 * 08.08.2012 - new file
 *
 * (C) 2012 fnh
 * moved to gtest by Guillaume WALCK in 2014
 */


#include <map>
#include <string>
#include <math.h>

#include <pluginlib/class_loader.h>
#include <hand_kinematics/hand_kinematics_plugin.h>


// Gtest
#include <gtest/gtest.h>


#define DEG2RAD  (M_PI / 180.0)
#define RAD2DEG  (180.0 / M_PI)

// the maximum error (in radians) that we tolerate to count IK as a success
#define MAX_DELTA 0.01


double rand_range(double min_n, double max_n) {
  return (double)rand() / RAND_MAX * (max_n - min_n) + min_n;
}

void random_test_finger_fkik(pluginlib::ClassLoader<hand_kinematics::HandKinematicsPlugin> &hkp, std::string PREFIX, std::string prefix, int n_tests ,bool verbose=false) {
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
    if (isThumb) { // min-degrees 0 -30 -15 0 -60  max-degrees 90 30 15 75 60
      jj.push_back( rand_range(   0*DEG2RAD,  90*DEG2RAD ) ); // J1
      jj.push_back( rand_range( -30*DEG2RAD,  30*DEG2RAD ) ); // J2
      jj.push_back( rand_range( -15*DEG2RAD,  15*DEG2RAD ) ); // J3
      jj.push_back( rand_range(   0*DEG2RAD,  75*DEG2RAD ) ); // J4
      jj.push_back( rand_range( -60*DEG2RAD,  60*DEG2RAD ) ); // J5
    }
    else {
      jj.push_back( j1j2= rand_range( 0*DEG2RAD, 90*DEG2RAD ) ); // J1
      jj.push_back( j1j2 );    // we need J2==J1 to enforce the joint-coupling
      jj.push_back( rand_range(   0*DEG2RAD, 90*DEG2RAD ) ); // J3
      jj.push_back( rand_range( -25*DEG2RAD, 25*DEG2RAD )); // J4 abduction
      if (isLittleFinger) {
        jj.push_back( rand_range( 0*DEG2RAD, 40*DEG2RAD ) ); // LFJ5
      }
    }

    // call fk to get the fingertip <prefix+tip> position
    //
    fkdata.request.header.frame_id = "palm";
    fkdata.request.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.frame_id = "";
    fkdata.request.robot_state.joint_state.name.resize( jointNames.size() );
    fkdata.request.robot_state.joint_state.position.resize( jointNames.size() );

		std::vector<std::string> link_names;
		link_names.push_back(tipName);
		if(verbose)
      ROS_INFO( "FK req fk_link_names[0] is %s", tipName.c_str() );

		std::vector<double> joint_angles(jointNames.size());
		
    for( unsigned int j=0; j < jointNames.size(); j++ ) {
      fkdata.request.robot_state.joint_state.name[j] = jointNames[j];
      joint_angles[jointNames.size()-1-j] = jj[j]; // invert the order as it is inverted in the solver (base to tip)
      if(verbose)
        ROS_INFO( "FK req %d joint %d %s   radians %6.2f", i, j, jointNames[j].c_str(), jj[j] );
    }
    std::vector<geometry_msgs::Pose> poses;
		bool status = hkp.getPositionFK(link_names ,
                       joint_angles, 
                       poses));

    
    if(verbose)
      ROS_INFO( "FK returned status %d", int(status));

    
    geometry_msgs::Pose pose = poses[0]; // position.xyz orientation.xyzw

    if (status) {
      if(verbose)
        ROS_INFO( "FK pose is %8.4f %8.4f %8.4f", 
                  pose.position.x,
                  pose.position.y,
                  pose.position.z );
    }
    else {
      continue; // no use trying IK without FK success
    }
/*
    // now try IK to reconstruct FK angles
    // 
    moveit_msgs::GetPositionIK::Request  ikreq;
    moveit_msgs::GetPositionIK::Response ikres;

    ikreq.ik_request.ik_link_name = tipName;
    ikreq.ik_request.pose_stamped.header.frame_id = "palm";
    ikreq.ik_request.pose_stamped.pose.position.x = pose.position.x;
    ikreq.ik_request.pose_stamped.pose.position.y = pose.position.y;
    ikreq.ik_request.pose_stamped.pose.position.z = pose.position.z;

    //pos is not relevant with the used ik_solver but set it anyway to identity
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
    fk.call( fkdata ); // (fkeq, fkres );

    status = fkdata.response.error_code.val;
    if(verbose)
      ROS_INFO( "FK returned status %d", (int) status );

    std::vector<geometry_msgs::PoseStamped> ppp = fkdata.response.pose_stamped;
    geometry_msgs::Pose pppose = ppp[0].pose; // position.xyz orientation.xyzw
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
*/
  ROS_WARN( "-#- tested %d random positions, %d IK solved, %d matched", n_tests, n_iksolved, n_matched );
  EXPECT_TRUE(n_iksolved == n_tests);
  EXPECT_TRUE(double(n_matched)/n_tests >= 0.99);
}

// Declare a test
TEST(FKIK, first_finger)
{
	pluginlib::ClassLoader<hand_kinematics::HandKinematicsPlugin> handkinplugin("kinematics_base", "hand_kinematics::HandKinematicsPlugin");

  handkinplugin.initialize("robot_description");

	random_test_finger_fkik( handkinplugin, "FF", "ff", 1000 ,verbose);
	//random_test_finger_fkik( nh, "MF", "mf", 1000 ,verbose);
  //random_test_finger_fkik( nh, "RF", "rf", 1000 ,verbose);
  //random_test_finger_fkik( nh, "LF", "lf", 1000 ,verbose);
  //random_test_finger_fkik( nh, "TH", "th", 1000 ,verbose);
	
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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){

  
}
	
	testing::InitGoogleTest(&argc, argv);
	
	ros::init (argc, argv, "upmc_fkik_test" );
  ros::NodeHandle nh;
  bool verbose;
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
    seed = ros::Time::now().sec; // wait until /clock received
  }
  ROS_INFO( "-#- FK/IK test started, random seed is %d", seed );
  srandom( seed ); 
  
  return RUN_ALL_TESTS();
}
