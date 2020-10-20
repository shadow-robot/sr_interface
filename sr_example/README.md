# Examples

We are providing a [high level wrapper](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_robot_commander) around our robots to make the interfacing easier for non-ROS people. The [sr_example](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example) package contains examples on how to use it.

Use cases are given for:

-  [Hand](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example/scripts/sr_example/hand_examples)
-  [Arm](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example/scripts/sr_example/arm_examples)
-  [Hand and arm](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example/scripts/sr_example/hand_and_arm_examples)

To run these examples, the robot should be launched first (instructions for this can be found [here](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_robot_launch)) 

In order to access the full functionality of the hand, you can interface with it directly through the ROS interface. You can find python examples in the
 [advanced](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example/scripts/sr_example/advanced) folder
 and c++ examples in the [src](https://github.com/shadow-robot/sr_interface/tree/melodic-devel/sr_example/src) folder.

Description of each example can be found in the comment at the top of each script.
