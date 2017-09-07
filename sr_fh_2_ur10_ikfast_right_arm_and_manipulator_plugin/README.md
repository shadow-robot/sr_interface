Following guide [here](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html) (sort of).

Made a temporary directory and entered it.

Generated hand urdf with:
`rosrun xacro xacro --inorder -o model.urdf $(rospack find fh_interface)/fh_2_ur10_moveit_config/config/fh_v2_ur10_with_hand.urdf.xacro`

Generated collada (.dae) file using:
`rosrun collada_urdf urdf_to_collada model.urdf model.dae`

Rounded collada file floats with:
`rosrun moveit_kinematics round_collada_numbers.py model.dae model.rounded.dae 5`

Removed python-mpmath
`sudo apt remove python mpmath`

Install python sympy v0.7.1:
`sudo pip install sympy==0.7.1`

Generated IKFast solver source code with:
`python $(openrave-config --python-dir)/openravepy/_openravepy_/ikfast.py --robot=model.rounded.dae --iktype=transform6d --baselink=1 --eelink=12 --savefile=ikfast_61_right_arm_and_manipulator.cpp`

Entered the fh_interface source folder and created a new package in fh_interface:
`catkin_create_pkg fh_2_ur10_ikfast_right_arm_and_manipulator_plugin`

Built the workspace.

Copied create_ikfast_moveit_plugin.py from moveit source to new package and modified it to cope with our weird robot name.

Returned to temporary directory and ran create_ikfast_moveit_plugin.py:

`rosrun fh_2_ur10_ikfast_right_arm_and_manipulator_plugin create_ikfast_moveit_plugin.py ursrfh right_arm_and_manipulator fh_2_ur10_ikfast_right_arm_and_manipulator_plugin $(pwd)/ikfast_61_right_arm_and_manipulator.cpp`

Moved update_ikfast_plugin.py to scripts and modified it to work in this context.

Yet to test this new package.