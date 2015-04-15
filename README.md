Launch files for UR10 arm and SR hand.

## Starting the robots

### Simulation
To start the simulation of the hand and arm, you can run:

```
roslaunch sr_robot_launch right_srhand_ur10arm.launch
```

### Real Robots
To start the real robots, do:

```
roslaunch sr_robot_launch right_srhand_ur10arm.launch sim:=false
```
