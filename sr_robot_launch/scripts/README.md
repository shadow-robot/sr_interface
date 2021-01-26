# Utility scripts for sr_robot_launch

This folder contains utility scripts to launch robots.

## Sr_ur_arm_unlock

Node that allows to release the UR robot arm in case of emergency or protective stop. It also offers functionality of braking the arms when needed.

#### Input

Topic name: "/sr_arm/release_or_brake"
Msg type: std_msgs/Bool

#### Output
- If the arm is running under trajectory controller, if signal is received the arm will be turned off (brakes engaged).

- If the arm has stopped as a consequence of emergency stop or protective stop or user braking decision, it will restart the arm and the trajectory controller.

- If the arm is enabled but the trajectory controller is stopped, it will resend the robot program to restart the trajectory controller.