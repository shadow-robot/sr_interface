#!/usr/bin/env python
# Software License Agreement (BSD License)
#

import rospy
import tf
from sr_utilities.local_urdf_parser_py import URDF


def publish_world_to_base_transform():
    while not rospy.has_param('/robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")
    urdf_str = rospy.get_param('/robot_description')
    robot_urdf = URDF.from_xml_string(urdf_str)
    robot = URDF.from_xml_string(urdf_str)
    robot_root = robot.get_root()

    if robot_root is not None and robot_root != "world":
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            br = tf.TransformBroadcaster()
            br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),  robot_root, "world")
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('virtual_joint_broadcaster', anonymous=True)
    # Publish world to base transform
    try:
        publish_world_to_base_transform()
    except rospy.ROSInterruptException:
        pass
