# Sending Commands [python]

The following python code will send one target (of 1.5 radians) to FFJ3. First we initialise the node (to register it with the rest of the ROS ecosystem). We then create a publisher to be able to stream some data to the correct topic. Finally we publish the target we want to send.

```python
import rospy
from std_msgs.msg import Float64

rospy.init_node('shadowhand_command_publisher_python')
pub = rospy.Publisher('sh_ffj3_position_controller/command', Float64, latch=True)
pub.publish(1.5)

rospy.spin()
```
