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

TEST
----
To test the FK/IK run:

  ```   
  rostest hand_kinematics hand_kinematics_services_test.test
  rostest hand_kinematics hand_kinematics_plugin_test.test
  ```
