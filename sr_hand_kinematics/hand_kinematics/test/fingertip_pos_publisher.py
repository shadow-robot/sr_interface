#!/usr/bin/env python
import roslib
import rospy
import tf
from geometry_msgs.msg import Vector3

roslib.load_manifest('hand_kinematics')

if __name__ == '__main__':
    rospy.init_node('tf_tippos_publisher')
    listener = tf.TransformListener()
    tip_pos_pub = []
    tip_pos_pub.append(rospy.Publisher('fftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('mftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('rftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('lftip/position/', Vector3, queue_size=10))
    tip_pos_pub.append(rospy.Publisher('thtip/position/', Vector3, queue_size=10))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(
                '/rh_palm', '/rh_fftip', rospy.Time.now(), rospy.Duration(1.0))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_fftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[0].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_mftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[1].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_rftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[2].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_lftip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[3].publish(Vector3(trans[0], trans[1], trans[2]))
            try:
                (trans, rot) = listener.lookupTransform(
                    '/rh_palm', '/rh_thtip', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            tip_pos_pub[4].publish(Vector3(trans[0], trans[1], trans[2]))
        except:
            continue

        rate.sleep()
