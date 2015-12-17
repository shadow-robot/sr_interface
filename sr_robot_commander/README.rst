

High Level Interface
====================

This package contains libraries that provide a high level interface to easily control the different robots
supported by Shadow Robot. They encapsulate the functionality provided by different ROS packages, specially the 
`moveit_commander <http://wiki.ros.org/moveit_commander>`__, enabling their access throughout a more simplified interface.


The following classes can be found in this package

-  `SrRobotCommander <doc/tutorial/RobotCommander.html>`__ - base class
-  `SrArmCommander <doc/tutorial/ArmCommander.html>`__ - arm management
   class
-  `SrHandCommander <doc/tutorial/HandCommander.html>`__ - hand management
   class

In order to access the full functionality of the hand, you can also interface with it directly through the ROS interface.