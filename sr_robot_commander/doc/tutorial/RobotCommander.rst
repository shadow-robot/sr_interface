Robot Commander
---------------

Overview
~~~~~~~~

The main purpose of the robot commander is to provide a base clase to the
hand and arm commanders, providing
methods which can be used on both. The RobotCommander should not be used directly unless necessary. 
Use the `HandCommander <HandCommander.html>`__ or `ArmCommander <ArmCommander.html>`__ instead. 

Examples of usage can be found `here <../../../sr_example/README.html>`__.

Following you can find decriptions of the most relevant functions available for both.

Basic terminology
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
A robot is described using an `srdf <http://wiki.ros.org/srdf>`__ file which contains the semantic description that is not available in the `urdf <http://wiki.ros.org/urdf>`__. It describes a robot as a collection of **groups** that are representations of different set of joints which are useful for planning. Each group can have specified its **end-effector** and **group states** which are specific set of joint values predifined for that group with a given name, for example *close_hand* or *folded_arm*.

As the robot commander is a high lever wrapper of the `moveit_commander <http://wiki.ros.org/moveit_commander>`__, its constructor takes the name of one of the robot groups for which the planning will be performed.

Here is an example of an UR10 arm and a shadow hand with their different groups and end-effectors.

(Add picture)

Getting basic information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
We can get the name of the robot, group or planning reference frame:

.. code:: python

   print "Robot name: ", commander.get_robot_name()
   print "Group name: ", commander.get_group_name()
   print "Planning frame: ", commander.get_planning_frame()

Get the list of names of the predifined group states from the srdf and warehouse for the current group:

.. code:: python

   # Refresh them first if they have recently changed
   commander.refresh_named_targets()
   
   print "Named targets: ", commander.get_named_targets()
   
Get the joints position and velocity:

.. code:: python

   joints_position = commander.get_joints_position()
   joints_velocity = commander.get_joints_velocity()

   print("Arm joints position\n" + str(joints_position) + "\n")
   print("Arm joints velocity\n" + str(joints_velocity) + "\n")
   
Get the current joint state of the group being used:

.. code:: python
   
   current_state = commander.get_current_state()
   
   # To get the current state but enforcing that each joint is within its limits
   current_state = commander.get_current_state_bounded()
   

Get the current position of the end-effector:

.. code:: python

   # Specify the desired reference frame if different from planning frame
   eef_position = commander.get_current_pose("palm")

Get the end-effector position from a specified joint-state:

.. code:: python

   joints_states = {'ra_shoulder_pan_joint': 0.5157461682721474,
                    'ra_elbow_joint': 0.6876824920327893,
                    'ra_wrist_1_joint': -0.7695210732233582,
                    'ra_wrist_2_joint': 0.2298871642157314,
                    'ra_shoulder_lift_joint': -0.9569080092786892,
                    'ra_wrist_3_joint': -0.25991215955733704}
   eef_position = get_end_effector_pose_from_state(joints_states)

Get the end-effector position from a group state previously defined:

.. code:: python

   eef_position = get_end_effector_pose_from_named_state("hand_open")

Setting functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
You can change the reference frame to get pose information:

.. code:: python

   set_pose_reference_frame("palm"):

You can also activate or deactivate the teach mode for the robot:

.. code:: python

   # Activation: stops the the trajectory controllers for the robot, and sets it to teach mode.
   commander.set_teach_mode(True)
   
   # Deactivation: stops the teach mode and starts trajectory controllers for the robot.  
   # Currently this method blocks for a few seconds when called on a hand, while the hand parameters are reloaded.
   commander.set_teach_mode(False)

Plan/move to a joint-space goal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the methods **plan\_to\_joint\_value\_target** or **move\_to\_joint\_value\_target**, a set of the joint values can be given for the specified group to create a plan and send it for execution.

Parameters:

-  *joint\_states* is a dictionary with joint name and value. It can
   contain joints values of which need to be changed.
-  *wait* indicates if method should wait for movement end or not
   (default value is True)
-  *angle\_degrees* should be set to true if the input angles are in
   degrees (default value is False)

*IMPORTANT:* Bear in mind that the names of the joints are different for
the right and left arm/hand.

Example
^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    arm_commander = SrArmCommander(name="right_arm", set_ground=True)
    joints_states = {'ra_shoulder_pan_joint': 0.5157461682721474,
                     'ra_elbow_joint': 0.6876824920327893,
                     'ra_wrist_1_joint': -0.7695210732233582,
                     'ra_wrist_2_joint': 0.2298871642157314,
                     'ra_shoulder_lift_joint': -0.9569080092786892,
                     'ra_wrist_3_joint': -0.25991215955733704}
    
    # Only plan
    arm_commander.plan_to_joint_value_target(joints_states)
    
    # Plan and execute
    arm_commander.move_to_joint_value_target(joints_states)

This example demonstrates how joint states for an arm can be sent to
SrArmCommander, as neither the 'wait' nor 'angle\_degrees' arguments are
specified, they take the default values of 'True' and 'False,
respectively.

Example 2
^^^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_commander = SrHandCommander(name="right_hand")
    joints_states = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 0.0,
                     'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 0.0,
                     'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 0.0,
                     'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                     'rh_THJ1': 40, 'rh_THJ2': 35, 'rh_THJ3': 0.0, 'rh_THJ4': 65, 'rh_THJ5': 15,
                     'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
    hand_commander.move_to_joint_value_target(joints_states, wait=False, angle_degrees=True))

In this example, joint states for a hand are sent to SrHandCommander,
the method is prompted by the 'wait=False' argument to not wait for the
movement to finish executing before moving on to the next command and
the 'angle\_degrees=True' argument tells the method that the input
angles are in degrees, so require a conversion to radians.

Plan to a trajectory of specified waypoints
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the method **plan\_to\_waypoints\_target**, it is posible to specify a set of waypoints for the end-effector and create a plan to follow it.

Parameters:

-  *waypoints* is an array of poses of the end-effector.
-  *eef\_step* indicates that the configurations are goint to be computed for every eef_step meters
-  *jump\_threshold* specify the maximum distance in configuration space between consecutive points in the resulting path

Example
^^^^^^^
(Add example)

Move to a trajectory of specified joint states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the method **run\_joint\_trajectory**, it is posible to specify a trajectory composed of a set of joint states with specified timeouts and follow it.

Example
^^^^^^^
(Add example)

Move to the start of a given trajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the method **move\_to\_trajectory\_start**, it is posible create and execute a plan from the current state to the first state of a pre-existing trajectory

Example
^^^^^^^
(Add example)

Plan/move to a predefined named pose
~~~~~~~~~~~~~~~~~~~~~~~

Using the methods **plan_to_named_target** or **move\_to\_named\_target** will allow to plan or move the group to a predefined pose. This pose can be defined in the srdf or saved as a group state in the moveit warehouse.

Parameters:

-  *name* is the unique identifier of the target pose
-  *wait* indicates if method should wait for movement end or not
   (default value is True)

Example
^^^^^^^

**pack** is a predifined pose defined in the SRDF file for the *right_hand* group:

.. code:: html

  <group_state group="right_hand" name="pack">
    <joint name="rh_THJ1" value="0.52"/>
    <joint name="rh_THJ2" value="0.61"/>
    <joint name="rh_THJ3" value="0.00"/>
    <joint name="rh_THJ4" value="1.20"/>
    <joint name="rh_THJ5" value="0.17"/>
    <joint name="rh_FFJ1" value="1.5707"/>
    <joint name="rh_FFJ2" value="1.5707"/>
    <joint name="rh_FFJ3" value="1.5707"/>
    <joint name="rh_FFJ4" value="0"/>
    <joint name="rh_MFJ1" value="1.5707"/>
    <joint name="rh_MFJ2" value="1.5707"/>
    <joint name="rh_MFJ3" value="1.5707"/>
    <joint name="rh_MFJ4" value="0"/>
    <joint name="rh_RFJ1" value="1.5707"/>
    <joint name="rh_RFJ2" value="1.5707"/>
    <joint name="rh_RFJ3" value="1.5707"/>
    <joint name="rh_RFJ4" value="0"/>
    <joint name="rh_LFJ1" value="1.5707"/>
    <joint name="rh_LFJ2" value="1.5707"/>
    <joint name="rh_LFJ3" value="1.5707"/>
    <joint name="rh_LFJ4" value="0"/>
    <joint name="rh_LFJ5" value="0"/>
    <joint name="rh_WRJ1" value="0"/>
    <joint name="rh_WRJ2" value="0"/>
  </group_state>

Here is how to move to it:

.. code:: python

    rospy.init_node("robot_commander_examples", anonymous=True)
    hand_commander = SrHandCommander(name="right_hand")
    
    # Only plan
    hand_commander.plan_to_named_target("pack")
    
    # Plan and execute
    hand_commander.move_to_named_target("pack")

Move through a trajectory of predefined group states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the method **run\_named\_trajectory**, it is posible to specify a trajectory composed of a set of names of previously defined group states (either from SRDF or from warehouse), plan and move to follow it.

Parameters:

-  *trajectory* specify a dictionary of waypoints with the following elements:
   -  name: the name of the way point
   -  interpolate_time: time to move from last waypoint
   -  pause_time: time to wait at this waypoint

Example
^^^^^^^

.. code:: python

    trajectory = [
      {
          'name': 'open',
          'interpolate_time': 3.0
      },
      {
          'name': 'pack',
          'interpolate_time': 3.0,
          'pause_time': 2
      },
      {
          'name': 'open',
          'interpolate_time': 3.0
      },
      {
          'name': 'pack',
          'interpolate_time': 3.0
      }
    ]

    hand_commander.run_named_trajectory(trajectory)
    
    # If you want to send the trajectory to the controller without using the planner, you can use:
    hand_commander.run_named_trajectory_unsafe(trajectory)

move_to_joint_value_target_unsafe
run_joint_trajectory_unsafe

Check if a plan is valid and execute it
~~~~~~~~~~~~~~~~~~~

Use the method **check_plan_is_valid** and **execute** to check if the current plan contains a valid trajectory and execute it. Only has meaning if called after a planning function has been attempted.

Example
^^^^^^^

.. code:: python

    rospy.init_node("robot_commander_examples", anonymous=True)

    arm_commander = SrArmCommander(name="right_arm")

    arm_commander.plan_to_named_target("target_name")
    
    if arm_commander.plan_is_valid():
        arm_commander.execute()
