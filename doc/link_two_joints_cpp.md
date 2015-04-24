# Link Two Joints [C++]

## Reading and sending data from C++
This is a simple program that links one finger joint to another: it subscribes to the topic publishing a parent joints position data, and publishes the data of the selected parent joint as a target for it's child joint. This will simply make the child joint move together with the parent joint.

*NB: To send a new target to a joint, you simply need to publish a `std_msgs/Float64` message to the appropriate controllers command topic.*

The full list of joints to send targets to the hand is: wrj1, wrj2, ffj4, ffj3, ffj0, mfj4, mfj3, mfj0, rfj4, rfj3, rfj0, lfj5, lfj4, lfj3, lfj0, thj5, thj4, thj3, thj2, thj1.

```c++
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>

std::string parent_name = "ffj3";
std::string child_name  = "mfj3";
ros::Subscriber sub;
ros::Publisher pub;

void callback(const pr2_controllers_msgs::JointControllerStateConstPtr& msg)
{
  //publish the message
  std_msgs::Float64 command;
  command.data = msg->set_point;
  pub.publish(command);
}

int main(int argc, char** argv)
{
  //init the ros node
  ros::init(argc, argv, "link_joints_example");
  ros::NodeHandle node;

  pub = node.advertise<std_msgs::Float64>("sh_" + child_name + "_position_control/command", 2);
  sub = node.subscribe("sh_" + parent_name + controller_type + "/state", 2,  callback);

  while( ros::ok() )
    ros::spin();

  return 0;
}
```

Let's look at this code.
 * We need to import the messages to send / receive messages from the ROS interface. That's Float64 from `std_msgs` for the command messages sent to the child joint and jointControllerState from `pr2_controller_msgs` for the joint status messages from which we extract the parent joint position.
```c++
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>
```

 * In the main, after initialising the node, we create a publisher to the child joint controller command topic and subscribe to the parent joint controller state topic which contains information about the current state of the joint, including the current target. Each time we receive a message on this topic, we'll call the callback function.
```c++
int main(int argc, char** argv)
{
  [...]
    //the publisher
    pub = node.advertise<std_msgs::Float64>("sh_" + child_name + "_posion_control/command", 2);
    // the subscriber
    sub = node.subscribe("sh_" + parent_name + controller_type + "/state", 2,  callback);
  [...]
}
```

 * In the callback function we then take the set_point target value from the parent joint status message and publish it to the parent command topic:
```c++
void callback(const pr2_controllers_msgs::JointControllerStateConstPtr& msg)
{
  //publish the message
  std_msgs::Float64 command;
  command.data = msg->set_point;
  pub.publish(command);
}
```

## Compiling the Code

Now that we've written the code, we need to compile it. More information regarding how to properly compile using the catkin build system can be found [here](http://wiki.ros.org/catkin). In the CMakeLists.txt, you'll need to add the following lines:

```cmake
add_executable(link_joints src/link_joints.cpp)
add_dependencies(link_joints ${catkin_EXPORTED_TARGETS})
target_link_libraries(link_joints ${Boost_LIBRARIES} ${catkin_LIBRARIES})
```

You're now ready to compile using **catkin_make** in the root of your workspace.
```bash
roscd
cd ..
catkin_make
```

## Running the example

To test this example, you need to first start the simulator:
```bash
roslaunch sr_hand gazebo_arm_and_hand.launch
```

Then we run this code (which can be found in the [sr_example](/sr_example) package).
```bash
rosrun sr_example link_joints
```

Now if you send a target to **FFJ3**,**MFJ3** will follow:
```bash
rostopic pub /sh_ffj3_position_controller/command std_msgs/Float64 1.5
```

Please note that you can find more examples in the [sr_example](/sr_example) package.
