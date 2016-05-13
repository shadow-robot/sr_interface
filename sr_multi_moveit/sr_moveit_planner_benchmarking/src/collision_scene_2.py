#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import tf
import time


class CreateScene2(object):
    def __init__(self):
        self._scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        # pause to wait for rviz to load
        rospy.sleep(4)

        # add floor object
        floor_pose = [0, 0, -0.12, 0, 0, 0, 1]
        floor_dimensions = [4, 4, 0.02]
        self.add_box_object("floor", floor_dimensions, floor_pose)

        # add collision objects

        self._kinect_pose_1 = [0.0, 0.815, 0.665, 0, -0.707, 0.707, 0]

    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]
        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))

    def spin(self, cloud_topic):
        tf_broadcaster = tf.TransformBroadcaster()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            tf_broadcaster.sendTransform((self._kinect_pose_1[0], self._kinect_pose_1[1], self._kinect_pose_1[2]),
                                         (self._kinect_pose_1[3], self._kinect_pose_1[4], self._kinect_pose_1[5],
                                          self._kinect_pose_1[6]), rospy.Time.now(), cloud_topic, "world")
            rate.sleep()


def main():
    rospy.init_node("collision_scene_2")
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        time.sleep(0.5)

    load_collision_objects = CreateScene2()
    kinect_cloud_topic = "kinect2_rgb_optical_frame"
    load_collision_objects.spin(kinect_cloud_topic)

if __name__ == "__main__":
    main()
