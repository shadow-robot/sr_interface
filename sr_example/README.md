# Examples

The sr_example package contains examples on how to use our hands from c++ or python code.

We're providing a [high level wrapper](../sr_robot_commander/README) around our robots to make the interfacing easier for non-ROS people. 

You can find some examples showing you the different functionalities of the library in the [scripts/sr_example folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example).
Use cases are given for the [hand](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_examples) and for 
[the hand and arm](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_and_arm_examples).
To Run these examples, the robot(s) should be launched first, followed by MoveIt. Instructions for this process can be found [here](../sr_robot_launch/README.md) 

For the full functionalities of the hand you can interface directly through the ROS interface. You'll find the python examples in the
 [scripts/sr_example/advanced folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/advanced), 
 and c++ examples in the [src folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/src).

Descriptions of the function of each example can be found commented at the top of each script.
