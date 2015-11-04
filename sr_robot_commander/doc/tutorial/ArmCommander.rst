Arm Commander
-------------

Description
~~~~~~~~~~~

This commander provide commands specific to arm which allows to execute
all actions of the `robot commander <RobotCommander.md>`__. Also it
allows to move to certain position in Cartesian space, to joints states
values and to move using certain trajectory.

move\_to\_position\_target
~~~~~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

This method allows to move the end effector of the arm to certain point
in the space represented by (x, y, z) coordinates. The orientation of
the end effector can be any.

Parameters:

-  *xyz* new position of end-effector
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait* indicates if method should wait for movement end or not
   (default value is True)

Example
^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    arm_commander = SrArmCommander(name="right_arm", set_ground=True)

    new_position = [0.25527, 0.36682, 0.5426]
    arm_commander.move_to_position_target(new_position)

move\_to\_pose\_target
~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

This method allows to move the end effector of the arm to certain pose
(position and orientation) in the space represented by (x, y, z, rot\_x,
rot\_y, rot\_z).

Parameters:

-  *pose* new pose of end-effector: a Pose message, a PoseStamped
   message or a list of 6 floats: [x, y, z, rot\_x, rot\_y, rot\_z] or a
   list of 7 floats [x, y, z, qx, qy, qz, qw]
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait* indicates if method should wait for movement end or not
   (default value is True)

Example
^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    arm_commander = SrArmCommander(name="right_arm", set_ground=True)

    new_pose = [0.5, 0.3, 1.2, 0, 1.57, 0]
    arm_commander.move_to_pose_target(new_pose)

run\_joint\_trajectory
~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

In case the robot needs to have complex movement through several joint
states you can use this method.

Parameters:

-  *joint\_trajectory* object of JointTrajectory class. Represents
   trajectory of the joints which would be executed.

*IMPORTANT:* Bear in mind that the names of the joints are different for
the right arm/hand and for the left one.

Example
^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    arm_commander = SrArmCommander(name="right_arm", set_ground=True)

    joints_states_1 = {'ra_shoulder_pan_joint': 0.43221632746577665, 
                       'ra_elbow_joint': 2.118891128999479,
                       'ra_wrist_1_joint': -1.711370650686752, 
                       'ra_wrist_2_joint': 1.4834244535003318,
                       'ra_shoulder_lift_joint': -2.5813317754982474, 
                       'ra_wrist_3_joint': 1.6175960918705412}

    joints_states_2 = {'ra_shoulder_pan_joint': 0.4225743596855942, 
                       'ra_elbow_joint': 1.9732180863151747,
                       'ra_wrist_1_joint': -0.8874321427449576, 
                       'ra_wrist_2_joint': -0.9214312892819567,
                       'ra_shoulder_lift_joint': -1.9299519748391978, 
                       'ra_wrist_3_joint': 0.7143446787498702}

    joints_states_3 = {'ra_shoulder_pan_joint': 1.6113530596480121, 
                       'ra_elbow_joint': 1.1552231775506083,
                       'ra_wrist_1_joint': -0.2393325455779891, 
                       'ra_wrist_2_joint': 0.4969532212998553,
                       'ra_shoulder_lift_joint': -1.5826889903403423, 
                       'ra_wrist_3_joint': 2.1117520537195738}

    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = list(joints_states_1.keys())
    joint_trajectory.points = []
    time_from_start = rospy.Duration(5)

    for joints_states in [joints_states_1, joints_states_2, joints_states_3]:
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = time_from_start
        time_from_start = time_from_start + rospy.Duration(5)

        trajectory_point.positions = []
        trajectory_point.velocities = []
        trajectory_point.accelerations = []
        trajectory_point.effort = []
        for key in joint_trajectory.joint_names:
            trajectory_point.positions.append(joints_states[key])
            trajectory_point.velocities.append(0.0)
            trajectory_point.accelerations.append(0.0)
            trajectory_point.effort.append(0.0)

        joint_trajectory.points.append(trajectory_point)

    arm_commander.run_joint_trajectory(joint_trajectory)

get\_pose\_reference\_frame
~~~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

Returns the reference frame for planning in cartesian space.


plan\_cartesian\_path\_to\_pose
~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

Generates a linear plan in cartesian space from current end effector pose to new pose specified.

Parameters:

-  *target_pose* - geometry_msgs/Pose in the frame given by get_pose_reference_frame.
-  *min\_fraction* - Percentage of trajectory which must be calculated correctly to execute plan. Defaults to 1 (i.e. whole trajectory)
-  *eef_step* and *jump_threshold* - planning args to move_group_commander
