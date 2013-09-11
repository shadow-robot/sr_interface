#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('sr_hand')
import rospy

import time, mutex, subprocess

from sr_robot_msgs.msg import sendupdate, joint, joints_data
from sensor_msgs.msg import *
from std_msgs.msg import Float64


class FancyDemo(object):
    # starting position for the hand
    start_pos_hand = [ joint(joint_name = "THJ1", joint_target = 21),
                       joint(joint_name = "THJ2", joint_target = 30),
                       joint(joint_name = "THJ3", joint_target = 0),
                       joint(joint_name = "THJ4", joint_target = 30),
                       joint(joint_name = "THJ5", joint_target = 9),
                       joint(joint_name = "FFJ0", joint_target = 180),
                       joint(joint_name = "FFJ3", joint_target = 90),
                       joint(joint_name = "FFJ4", joint_target = 0),
                       joint(joint_name = "MFJ0", joint_target = 180),
                       joint(joint_name = "MFJ3", joint_target = 90),
                       joint(joint_name = "MFJ4", joint_target = 0),
                       joint(joint_name = "RFJ0", joint_target = 180),
                       joint(joint_name = "RFJ3", joint_target = 90),
                       joint(joint_name = "RFJ4", joint_target = 0),
                       joint(joint_name = "LFJ0", joint_target = 180),
                       joint(joint_name = "LFJ3", joint_target = 90),
                       joint(joint_name = "LFJ4", joint_target = 0),
                       joint(joint_name = "LFJ5", joint_target = 0),
                       joint(joint_name = "WRJ1", joint_target = 0),
                       joint(joint_name = "WRJ2", joint_target = 0) ]
    #starting position for the arm
    start_pos_arm = [ joint(joint_name = "ElbowJRotate", joint_target = 0),
                      joint(joint_name = "ElbowJSwing", joint_target = 59),
                      joint(joint_name = "ShoulderJRotate", joint_target = 0),
                      joint(joint_name = "ShoulderJSwing", joint_target = 33)
                      ]
    #thumb up position for the hand
    thumb_up_pos_hand = [ joint(joint_name = "THJ1", joint_target = 0),
                          joint(joint_name = "THJ2", joint_target = 0),
                          joint(joint_name = "THJ3", joint_target = 0),
                          joint(joint_name = "THJ4", joint_target = 0),
                          joint(joint_name = "THJ5", joint_target = 0),
                          joint(joint_name = "FFJ0", joint_target = 180),
                          joint(joint_name = "FFJ3", joint_target = 90),
                          joint(joint_name = "FFJ4", joint_target = 0),
                          joint(joint_name = "MFJ0", joint_target = 180),
                          joint(joint_name = "MFJ3", joint_target = 90),
                          joint(joint_name = "MFJ4", joint_target = 0),
                          joint(joint_name = "RFJ0", joint_target = 180),
                          joint(joint_name = "RFJ3", joint_target = 90),
                          joint(joint_name = "RFJ4", joint_target = 0),
                          joint(joint_name = "LFJ0", joint_target = 180),
                          joint(joint_name = "LFJ3", joint_target = 90),
                          joint(joint_name = "LFJ4", joint_target = 0),
                          joint(joint_name = "LFJ5", joint_target = 0),
                          joint(joint_name = "WRJ1", joint_target = 0),
                          joint(joint_name = "WRJ2", joint_target = 10) ]

    #The hand publisher:
    # publish the message to the /srh/sendupdate topic.
    hand_publisher = rospy.Publisher('/srh/sendupdate', sendupdate)

    #The arm publisher:
    # publish the message to the /sr_arm/sendupdate topic.
    arm_publisher = rospy.Publisher('/sr_arm/sendupdate', sendupdate)

    #A boolean used in this demo: set to true while an action is running
    # just so we don't do 2 actions at once
    action_running = mutex.mutex()

    def __init__(self):
        #send the start position to the hand
        self.hand_publisher.publish(sendupdate(len(self.start_pos_hand), self.start_pos_hand))
        #send the start position to the arm
        self.arm_publisher.publish(sendupdate(len(self.start_pos_arm), self.start_pos_arm))

        #wait for the node to be initialized and then go to the starting position
        time.sleep(1)
        rospy.loginfo("OK, ready for the demo")

        # We have one subscriber per tactile sensor
        # This way we can easily have a different action mapped to
        # each tactile sensor
        # NB: The tactile sensor on the ring finger is not mapped in this example
        self.sub_ff = rospy.Subscriber("/sr_tactile/touch/ff", Float64, self.callback_ff, queue_size=1)
        self.sub_mf = rospy.Subscriber("/sr_tactile/touch/mf", Float64, self.callback_mf, queue_size=1)
        self.sub_lf = rospy.Subscriber("/sr_tactile/touch/lf", Float64, self.callback_lf, queue_size=1)
        self.sub_th = rospy.Subscriber("/sr_tactile/touch/th", Float64, self.callback_th, queue_size=1)


    def callback_ff(self,data):
        """
        The callback function for the first finger:
        called each time the tactile sensor publishes a message.

        If no action is currently running, we set the ShoulderJRotate
        to a negative value proportional to the pressure received.

        @param data: the pressure value
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #filters the noise: check when the finger is being pressed
        if data.data < .1:
            self.action_running.unlock()
            return

        #ok the finger sensor was pressed
        p = subprocess.Popen('beep')

        #rotate the trunk to (data_received * min_pos)
        data.data /= 2.
        if data.data > 1.:
            data.data = 1.
        target_sh_rot    = 0.  + data.data * (-45.0)
        target_sh_swing  = 20. + data.data * (-10.0)
        target_elb_swing = 90. + data.data * (-10.0)
        target_elb_rot   = 0.  + data.data * (-45.0)

        rospy.loginfo("FF touched, going to new target ")

        #wait 1s for the user to release the sensor
        time.sleep(.2)
        message = [joint(joint_name = "ShoulderJRotate", joint_target = target_sh_rot),
                   joint(joint_name = "ShoulderJSwing", joint_target = target_sh_swing),
                   joint(joint_name = "ElbowJRotate", joint_target = target_elb_rot),
                   joint(joint_name = "ElbowJSwing", joint_target = target_elb_swing)
                   ]

        self.arm_publisher.publish(sendupdate(len(message), message))

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def callback_mf(self, data):
        """
        The callback function for the middle finger:
        called each time the tactile sensor publishes a message.

        If no action is currently running, we set the ShoulderJRotate
        to a positive value proportional to the pressure received.

        @param data: the pressure value
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #filters the noise: check when the finger is being pressed
        if data.data < .1:
            self.action_running.unlock()
            return

        #ok finger was pressed
        p = subprocess.Popen('beep')
 
        #rotate the trunk to (data_received * min_pos)
        data.data /= 2.
        if data.data > 1.:
            data.data = 1.
        target_sh_rot    = 0.  + data.data * (45.0)
        target_sh_swing  = 20. + data.data * (20.0)
        target_elb_swing = 90. + data.data * (20.0)
        target_elb_rot   = 0.  + data.data * (45.0)

        rospy.loginfo("MF touched, going to new target ")

        #wait 1s for the user to release the sensor
        time.sleep(.2)
        message = [joint(joint_name = "ShoulderJRotate", joint_target = target_sh_rot),
                   joint(joint_name = "ShoulderJSwing", joint_target = target_sh_swing),
                   joint(joint_name = "ElbowJRotate", joint_target = target_elb_rot),
                   joint(joint_name = "ElbowJSwing", joint_target = target_elb_swing)
                   ]

        self.arm_publisher.publish(sendupdate(len(message), message))

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def callback_th(self, data):
        """
        The callback function for the thumb:
        called each time the tactile sensor publishes a message.
        
        If no action is currently running, we send the thumb_up
        targets to the hand.
        
        @param data: the pressure value
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #filters the noise: check when the finger is being pressed
        if data.data < .1:
            self.action_running.unlock()
            return

        #ok the finger was pressed
        p = subprocess.Popen('beep')

        rospy.loginfo("TH touched, going to thumb up position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the thumb_up_position to the hand
        self.hand_publisher.publish(sendupdate(len(self.thumb_up_pos_hand), self.thumb_up_pos_hand))

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def callback_lf(self, data):
        """
        The callback function for the little finger:
        called each time the tactile sensor publishes a message.
        
        If no action is currently running, we reset the arm and
        hand to their starting position
        
        @param data: the pressure value
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #filters the noise: check when the finger is being pressed
        if data.data < .1:
            self.action_running.unlock()
            return
        #ok finger pressed
        p = subprocess.Popen('beep')

        rospy.loginfo("LF touched, going to start position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the start position to the hand
        self.hand_publisher.publish(sendupdate(len(self.start_pos_hand), self.start_pos_hand))
        #send the start position to the arm
        self.arm_publisher.publish(sendupdate(len(self.start_pos_arm), self.start_pos_arm))

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

def main():
    """
    The main function
    """
    # init the ros node
    rospy.init_node('fancy_touch_demo', anonymous=True)
    
    fancy_demo = FancyDemo()

    # subscribe until interrupted
    rospy.spin()


if __name__ == '__main__':
    main()

