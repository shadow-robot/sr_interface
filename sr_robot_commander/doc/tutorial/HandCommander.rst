Hand Commander
--------------

Description
~~~~~~~~~~~

This commander provides commands specific to hand which allows to
execute all actions of the `robot commander <RobotCommander.md>`__. Also
it allows to get state of tactile sensors, set max force and get joints
effort.

Setup
~~~~~~~~

Import the hand or arm commander (or both) depending of your application, along with basic rospy libraries:

.. code:: python

    import rospy
    from sr_robot_commander.sr_arm_commander import SrArmCommander
    from sr_robot_commander.sr_hand_commander import SrHandCommander

The constructors for ``SrArmCommander`` and ``SrHandCommander`` take a
name parameter that should match the group name of the robot to be used.

Example
^^^^^^^

This example uses `HandFinder <../../../sr_utilities/README.md>`__ for
finding launched hand. For a right arm:

.. code:: python

    arm_commander = SrArmCommander(name="right_arm", set_ground=True)

For a left arm:

.. code:: python

    arm_commander = SrArmCommander(name="left_arm", set_ground=True)

You can use HandFinder utility to find the hand launched on the system.

.. code:: python

    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()

    hand_serial = hand_parameters.mapping.keys()[0]

    hand_id = hand_parameters.mapping[hand_serial]

    prefix = hand_parameters.joint_prefix[hand_serial]

    if hand_id == 'rh':
        hand_commander = SrHandCommander(name="right_hand", prefix="rh")
    else:
        hand_commander = SrHandCommander(name="left_hand", prefix="lh")

Alternatively you can hardcode the hand you are launching.

For a right hand:

.. code:: python

    hand_commander = SrHandCommander(name="right_hand", prefix="rh")

For a left hand:

.. code:: python

    hand_commander = SrHandCommander(name="left_hand", prefix="lh")

get\_joints\_effort
~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

This method does not take any parameters and returns dictionary with
efforts of the robot joints

Example
^^^^^^^

This example uses `HandFinder <../../../sr_utilities/README.md>`__ for
finding launched hand.

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()

    hand_serial = hand_parameters.mapping.keys()[0]

    hand_id = hand_parameters.mapping[hand_serial]

    if hand_id == 'rh':
        hand_commander = SrHandCommander(name="right_hand", prefix="rh")
    else:
        hand_commander = SrHandCommander(name="left_hand", prefix="lh")

    hand_joints_effort = hand_commander.get_joints_effort()

    print("Hand joints effort \n " + str(hand_joints_effort) + "\n")

Alternatively if you do not want to use the HandFinder utility, you can
hardcode the hand prefix into the code similar to the example below
(discouraged).

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_commander = SrHandCommander("right_hand", "rh")

    hand_joints_effort = hand_commander.get_joints_effort()

    print("Hand joints effort \n " + str(hand_joints_effort) + "\n")

set\_max\_force
~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

This method sets maximum force for hand

Parameters:

-  *joint\_name* name of the joint.
-  *value* maximum force value

Example
^^^^^^^

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()

    hand_serial = hand_parameters.mapping.keys()[0]

    hand_id = hand_parameters.mapping[hand_serial]

    prefix = hand_parameters.joint_prefix[hand_serial]

    if hand_id == 'rh':
        hand_commander = SrHandCommander(name="right_hand", prefix="rh")
    else:
        hand_commander = SrHandCommander(name="left_hand", prefix="lh")
    ## The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units) 
    hand_commander.set_max_force(prefix + "FFJ3", 600)

Similarly if you do not want to use HandFinder, you can hardcode the
parameters into the code (discouraged).

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_commander = SrHandCommander("right_hand", "rh")

    ## The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units) 
    hand_commander.set_max_force("rh_FFJ3", 600)

get\_tactile\_type and get\_tactile\_state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Description
^^^^^^^^^^^

*get\_tactile\_type* returns a string indicating the type of tactile
sensors present (e.g. PST, biotac, UBI0). *get\_tactile\_state* returns
an object containing tactile data. The structure of the data is
different for every tactile\_type .

Example
^^^^^^^

.. code:: python


    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()

    hand_serial = hand_parameters.mapping.keys()[0]

    hand_id = hand_parameters.mapping[hand_serial]

    prefix = hand_parameters.joint_prefix[hand_serial]

    if hand_id == 'rh':
        hand_commander = SrHandCommander(name="right_hand", prefix="rh")
    else:
        hand_commander = SrHandCommander(name="left_hand", prefix="lh")

    tactile_type = hand_commander.get_tactile_type()
    tactile_state = hand_commander.get_tactile_state()

    print("Hand tactile type\n" + tactile_type + "\n")
    print("Hand tactile state\n" + str(tactile_state) + "\n")

Or if you prefer not to use the HandFinder utility, you can hard code
the hand parameter into your code.

.. code:: python


    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_commander = SrHandCommander("right_hand", "rh")

    tactile_type = hand_commander.get_tactile_type()
    tactile_state = hand_commander.get_tactile_state()

    print("Hand tactile type\n" + tactile_type + "\n")
    print("Hand tactile state\n" + str(tactile_state) + "\n")

**Warning** All of above codes will crash if hand is not launched yet.
If you are using HandFinder, you can avoid this by checking the length
of the mapping. Otherwise you can check the parameter server directly to
see if the hand is launched.
