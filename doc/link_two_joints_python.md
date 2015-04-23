# Link Two Joints [python]

## Reading and sending data from Python
This is a simple program that links one finger joint to another: it subscribes to the topic publishing a parent joints position data, and publishes the data of the selected parent joint as a target for it's child joint. This will simply make the child joint move together with the parent joint.

*NB: To send a new target to a joint, you simply need to publish a `std_msgs/Float64` message to the appropriate controllers command topic.*

The full list of joints to send targets to the hand is: wrj1, wrj2, ffj4, ffj3, ffj0, mfj4, mfj3, mfj0, rfj4, rfj3, rfj0, lfj5, lfj4, lfj3, lfj0, thj5, thj4, thj3, thj2, thj1.

```python
import rospy
from pr2_controller_msgs.msg import JointControllerState
from std_msgs.msg import Float64

parent_name = "ffj3"
child_name = "mfj3"

def callback(data):
    """
    The callback function: called each time a message is received on the
    topic parent joint controller state topic

    @param data: the message
    """
    # publish the message to the child joint controller command topic.
    # here we insert the joint name into the topic name
    pub = rospy.Publisher('/sh_'+child_name+'_position_controller/command', Float64)
    pub.publish(data.set_point)


def listener():
    """
    The main function
    """
    # init the ros node
    rospy.init_node('joints_link_test', anonymous=True)

    # init the subscriber: subscribe to the
    # child joint controller topic, using the callback function
    # callback()
    rospy.Subscriber('/sh_'+parent_name+'_position_controller/state', JointControllerState, callback)
    # subscribe until interrupted
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Let's look at this code.
 * We need to import the messages to send / receive messages from the ROS interface. That's Float64 from `std_msgs|std_msgs` for the command messages sent to the child joint and jointcontrollerstate from `pr2_controller_msgs|pr2_controller_msgs` for the joint status messages from which we extract the parent joint position.
```python
from pr2_controller_msgs.msg import JointControllerState
from std_msgs.msg import Float64
```

 * We subscribe to the parent joint controller state topic which contains information about the current state of the joint, including the current target. Each time we receive a message on this topic, we'll call the callback function.
```python
def listener():
  [...]
    rospy.Subscriber('/sh_'+parent_name+'_position_controller/state', JointControllerState, callback)
```

 * In the callback function we create a publisher to the child joint controller command topic then take the set_point target value from the parent joint status message and publish it to the parent command topic:
```python
def callback(data):
  [...]
    pub = rospy.Publisher('/sh_'+child_name+'_position_controller/command', Float64)
    pub.publish(data.set_point)
```

To test this example, you need to first start the simulator:
```bash
roslaunch sr_hand gazebo_hand.launch
```
Then we run this code (which can be found in the [sr_example](/sr_example) package).
```bash
rosrun sr_example link_joints.py
```

Now if you send a target to **FFJ3**, **MFJ3** will follow:
```bash
rostopic pub /sh_ffj3_position_controller/command std_msgs/Float64 1.5
```

Please note that you can find more examples in the [sr_example](/sr_example) package.
