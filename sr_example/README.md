# Examples

We're providing a [high level wrapper](../sr_robot_commander/README.html) around our robots to make the interfacing easier for non-ROS people. The [sr_example](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example) package contains examples on how to use it.

Use cases are given for:

-  [Hand] (https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_examples)
-  [Arm] (https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/arm_examples)
-  [Hand and arm] (https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/hand_and_arm_examples)

To run these examples, the robot(s) should be launched first (instructions for this can be found [here](../sr_robot_launch/README.html)) 

In order to access the full functionality of the hand, you can interface with it directly through the ROS interface. You'll find python examples in the
 [advanced folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/scripts/sr_example/advanced), 
 and c++ examples in the [src folder](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_example/src).

Description of each example can be found in the comment at the top of each script.
