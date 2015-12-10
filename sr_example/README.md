# Examples

The sr_example package contains examples on how to use our hands using c++ or python code.

We're providing a [high level wrapper](../sr_robot_commander/README.html) around our robots to make the interfacing easier for non-ROS people. 

You can find some examples showing you the different functionalities of the library in the [scripts/sr_example folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example).
Use cases are given for the [hand](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_examples) and for 
[the hand and arm](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_and_arm_examples).
To run these examples, the robot(s) should be launched first (instructions for this can be found [here](../sr_robot_launch/README.html)) 

In order to access the full functionality of the hand, you can interface with it directly through the ROS interface. You'll find python examples in the
 [scripts/sr_example/advanced folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/advanced), 
 and c++ examples in the [src folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/src).

Descriptions of the function of each example can be found in the comment at the top of each script.
