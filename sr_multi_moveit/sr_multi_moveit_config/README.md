# sr_multi_moveit_config

Configuration of multiple robots running as a combined robot.

## Multi-Robot Move Groups and States

At run time, [generate_robot_srdf.py](scripts/generate_robot_srdf.py) parses individual robot (UR arms, Shadow Hands) U/SRDFs and creates move groups that include links from multiple robot definitions. It also loads and parses a configuration file ([multi_robot_move_group_states.yaml](config/multi_robot_move_group_states.yaml) by default) that defines named joint states for these combined robot move groups. These combined move group states can inherit joint angles from states defined in individual robot U/SRDFs.

The resulting combined robot description SRDF is loaded onto the ROS parameter server for use by various nodes, at `/robot_description`, and saved to [config/generated_robot.srdf](config/generated_robot.srdf) for introspection.
