# Examples

The sr_example package contains examples on how to use our hands from c++ or python code.

We're providing a [high level wrapper](sr_robot_commander) around our robots to make the interfacing easier for non-ROS people. 

You can find some examples showing you the different functionalities of the library in the [scripts/sr_example folder](scripts/sr_example). Use cases are given for the [hand](scripts/sr_example/hand_examples), [arm](scripts/sr_example/arm_examples) and for [hand and arm](scripts/sr_example/hand_and_arm_examples).
To Run these examples, the robot(s) should be launched first, followed by MoveIt. Instructions for this process can be found [here](http://shadow-robot.readthedocs.org/en/latest/sr_robot_launch/README/) 

For the full functionalities of the hand you can interface directly through the ROS interface. You'll find the python examples in the [scripts/sr_example/advanced folder](scripts/sr_example/advanced), and c++ examples in the [src folder](src).

Descriptions of the function of each example can be found commented at the top of each script.