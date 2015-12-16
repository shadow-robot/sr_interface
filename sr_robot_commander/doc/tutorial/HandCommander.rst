Hand Commander
-------------

Overview
~~~~~~~~~~~

The SrHandCommander inherits all methods from the `robot commander <RobotCommander.html>`__ and provides commands specific to the hand. It allows the state of the tactile sensors and joints effort to be read, and the maximum force to be set.

Setup
~~~~~~~~

Import the hand commander along with basic rospy libraries and the hand finder:

.. code:: python

    import rospy
    from sr_robot_commander.sr_hand_commander import SrHandCommander
    from sr_utilities.hand_finder import HandFinder

The constructor for the ``SrHandCommander`` takes a name parameter that should match the group name of the robot to be used. Also it takes the hand prefix, parameters and serial number that can be retrieved using the `HandFinder <https://github.com/shadow-robot/sr_core/blob/indigo-devel/sr_utilities/scripts/sr_utilities/hand_finder.py>`__.

Example
^^^^^^^

.. code:: python

    # Using the HandFinder
    hand_finder = HandFinder()
    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    # If name is not provided, it will set "right_hand" or "left_hand" by default, depending on the hand.
    hand_commander = SrHandCommander(name = "rh_first_finger",
                                     hand_parameters=hand_parameters,
                                     hand_serial=hand_serial)
    
    # Alternatively you can launch the hand directly
    hand_commander = SrHandCommander(name = "right_hand", prefix = "rh")
    
Getting information
~~~~~~~~

Use the ``get_joints_effort`` method to get a dictionary with efforts of the group joints.

.. code:: python

    hand_joints_effort = hand_commander.get_joints_effort()
    print("Hand joints effort \n " + str(hand_joints_effort) + "\n")


Use the ``get_tactile_type`` to get a string indicating the type of tactile
sensors present (e.g. PST, biotac, UBI0) or ``get_tactile_state`` to get
an object containing tactile data. The structure of the data is
different for every ``tactile_type`` .

.. code:: python

    tactile_type = hand_commander.get_tactile_type()
    tactile_state = hand_commander.get_tactile_state()

    print("Hand tactile type\n" + tactile_type + "\n")
    print("Hand tactile state\n" + str(tactile_state) + "\n")

Set the maximum force
~~~~~~~~~~~~~~~

Use the method ``set_max_force`` to set the maximum force for a hand joint.

Parameters:

-  *joint\_name* name of the joint.
-  *value* maximum force value

Example
^^^^^^^

.. code:: python

    ## The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units) 
    hand_commander.set_max_force("rh_FFJ3", 600)
