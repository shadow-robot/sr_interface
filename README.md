sr_grasp
========

Tools for running grasps (moveit_msgs/Grasp) on the Shadow Hand. Handles pre and main grasps, interpolating the motion
over a configuarable amount, using the position controllers.
Can optionally stop the grasp on contact with the finger tactile sensors.

Also contains:

* grasp_planner - A simple grasp planner for the shadow hand.
* isr_grasp.utils - A python library.

## grasp

### Quick Start

Start either your real Shadow Hand (with position controllers) or a simulated one:
```sh
roslaunch sr_hand gazebo_hand.launch
```
Then start the grasp node:
```sh
rosrun sr_grasp grasp
```
Now start the quick grasp tool, which provides a quick command line way to run a full hand grasp:
```sh
rosrun sr_grasp quick_grasp 
[INFO] [WallTime: 1405623038.743846] [0.000000] Looking for hand...
[INFO] [WallTime: 1405623040.068238] [624.276000] Found
Enter z for zero hand, g for grasp, p for pre-grasp, q quit:
```

### Actionlib

The main interface to the node is via actionlib, it exposes the sr_grasp_msgs/Grasp action on the grasp/ topic. See the
quick_grasp script for a python example of using this interface.


## grasp_planner

Node impliementing the sr_grasp_msgs/PlanGrasp action to generate grasps. Currently returns a single, hardcoded grasp.

## sr_grasp.utils

Python lib. Exports a mk_grasp function for quick generation of moveit_msgs/Grasp messages.
```py
from sr_grasp.utils import mk_grasp

grasp = mk_grasp({
    'LFJ3': 1.4, 'RFJ3': 1.4, 'MFJ3': 1.4, 'FFJ3': 1.4,
    'LFJ0': 2.0, 'RFJ0': 2.0, 'MFJ0': 2.0, 'FFJ0': 2.0,
    'THJ1': 0.4, 'THJ2': 0.36, 'THJ3': 0.2, 'THJ4': 1.23, 'THJ5': 0.06, 
})

```
