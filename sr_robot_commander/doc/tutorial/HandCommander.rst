Arm Commander
-------------

Overview
~~~~~~~~~~~

The HandCommander inherits all methods from the `robot commander <RobotCommander.html>`__ and provides commands specific to the hand. It allows to get state of tactile sensors, set maximum force and get joints effort.

Setup
~~~~~~~~

Import the hand commander along with basic rospy libraries and the hand finder:

.. code:: python

    import rospy
    from sr_robot_commander.sr_hand_commander import SrHandCommander
    from sr_utilities.hand_finder import HandFinder

The constructor for ``SrHandCommander`` take a name parameter that should match the group name of the robot to be used. Also it takes the hand prefix, parameters and serial number that can be retrieved using the `HandFinder <../../../sr_utilities/README.html>`__.

Example
^^^^^^^

.. code:: python

    # Using the HandFinder
    hand_finder = HandFinder()
    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    # If name is not provided, it will set "right_hand" or "left_hand" by default
    hand_commander = SrHandCommander(name = "rh_first_finger",
                                     hand_parameters=hand_parameters,
                                     hand_serial=hand_serial)
    
    # Alternatively you launch the hand directly
    hand_commander = SrHandCommander(name = "right_hand", prefix = True)

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
