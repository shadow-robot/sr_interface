Copyright UPMC 2012
Updated by Guillaume WALCK CITEC University Bielefeld 2014

DESCRIPTION
-----------
Hand_kinematics provides inverse kinematics for the Shadow Hand fingers and thumb, coping with coupled joints in the fingers and redundant joints in the thumb.
The IK services requires a 3D pose to be requested and not a 6D pose since 6D requests are most of the time unfeasible (only 3 to 5 DOF)
However, the request pose message must be 6D, only 3D translation part will be considered.

This code is based on some modified functions taken out of kdl and augmented with coupling possibilities. These functions are in package kdl_coupling, 
under the same KDL:: namespace and can work together with standard KDL functionalities. Indeed all the functions created have different names.


PRE-REQUEST
-----------
This package provides several interfaces, one of them is a plugin for constraint aware kinematics that depends on moveit_core package whereas the service interface only requires moveit_msgs. 
The plugin is now activated by default, comment out the dependencies in manifest and CMakelists.txt if you don't want it.

INSTALL
-------
make sure you have kdl_coupling in the workspace
rosmake hand_kinematics --rosdep-install

USAGE
-----
Launch files are provided to start the 5 kinematics services (one for each finger), either in standalone version (requires sr_hand to find URDF file)
or in standard version supposing you have already LOADED the robot_description on the parameter server. This robot description MUST contain tip frames (only in newer versions of URDF files)


TEST
----
To test the FK/IK several possibilities are offered : 
1) Automatic test
  * start a roscore
	* rostest hand_kinematics hand_kinematics_services_test.test
	* rostest hand_kinematics hand_kinematics_plugin_test.test
2) Manual provided tests 
       * start a simulation of the hand :
	roslaunch sr_hand gazebo_arm_and_hand_motor.launch
	roslaunch sr_hand sr_arm_motor.launch
       * start the kinematic node
	roslaunch hand_kinematics hand_kinematics.launch
       * start the test
	roslaunch hand_kinematics test_hand_kinematics.launch
	OR
	roslaunch hand_kinematics test_hand_kinematics_th.launch

	This will start a finger tip pos publisher to get a pos vector in space for each finger
	It will also start 3 test nodes or 1 test node, each one moving one finger. A circular movement is performed on each finger and a square movement for the thumb
	DO NOT START both at the same time on a real hand as the thumb and first finger will come into collision.

	To verify the result, plot the finger tip pos like this (xx = [ff:mf:rf:lf:th])
	
	rxplot -M 3d /xxtip/position/x /xxtip/position/y /xxtip/position/z

	look at the provided snapshots in test folder to verify what you should see.
	
3) Use provided command line requests (test/command_line_tests.txt) and paste them into the shell (works for first finger)
4) Use standard IK tutorials from ROS and adapt the service topics you request from


