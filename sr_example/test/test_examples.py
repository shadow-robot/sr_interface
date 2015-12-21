#!/usr/bin/env python

import rospy
from unittest import TestCase
from sensor_msgs.msg import JointState
import roslaunch
import sys


PKG = "sr_example"


class TestExample(TestCase):
    """
    Test class that spawns an example and checks its output for errors.
    """

    def setUp(self):
        self.__executable = str(sys.argv[1])

        self.__received_js_msg = False

        rospy.init_node("test_examples", anonymous=True)

        self.__joint_state_subscriber = rospy.Subscriber("joint_states", JointState,
                                                         self.__joint_state_callback)

    def __joint_state_callback(self, msg):
        self.__received_js_msg = True
        self.__joint_state_subscriber.unregister()

    def test_example(self):
        while not self.__received_js_msg:
            rospy.sleep(1.)

        node = roslaunch.roslaunch_core.Node("sr_example", self.__executable)
        launch = roslaunch.ROSLaunch()
        launch.start()

        process = launch.launch(node)

        while process.is_alive():
            rospy.sleep(0.1)

        self.assertIsNone(process.exit_code)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_examples", TestExample)