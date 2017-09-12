# Steps to have the ikfast pluging working
These steps are a summary of what we did to generate the ikfast pluging following the guide in [here](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html).

## If you need to compute the ikfast solver source code
- Made a temporary directory and entered it.

- Generate the robot urdf with:
```bash
rosrun xacro xacro --inorder -o model.urdf $(rospack find sr_multi_description)/urdf/right_srhand_ur10.urdf.xacro
```

- Generated collada (.dae) file using:
```bash
rosrun collada_urdf urdf_to_collada model.urdf model.dae
```

- Rounded collada file floats with:
```bash
rosrun ur10srh_right_arm_moveit_ikfast_plugin round_collada_numbers.py model.dae model.rounded.dae 5
```

- Removed python-mpmath
```bash
sudo apt remove python mpmath
```

- Install python sympy v0.7.1:
```bash
sudo pip install sympy==0.7.1
```

- Launch the robot to automatically generate the srdf and yaml files.
```bash
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
```

- Generated IKFast solver source code with:
```bash
openrave.py --database inversekinematics --robot=ur10.dae --maxcasedepth=1 --iktests=1000
```
or:
```
python $(openrave-config --python-dir)/openravepy/_openravepy_/ikfast.py --robot=model.rounded.dae --iktype=transform6d --baselink=1 --eelink=12 --savefile=ikfast_u10srh_right_arm.cpp
```
## Create the ikfast plugin
- Entered the sr_multi_moveit folder and created a new package:
```bash
catkin_create_pkg ur10srh_right_arm_moveit_ikfast_plugin
```

- Built the workspace.

- Copied create_ikfast_moveit_plugin.py from moveit source to new package and modified it to cope with our robot name.

- Returned to temporary directory and ran create_ikfast_moveit_plugin.py:

```bash
rosrun ur10srh_right_arm_moveit_ikfast_plugin create_ikfast_moveit_plugin.py ur10srh right_arm ur10srh_right_arm_moveit_ikfast_plugin $(rospack find ur10srh_right_arm_moveit_ikfast_plugin)/ur10_depth1.cpp
```
