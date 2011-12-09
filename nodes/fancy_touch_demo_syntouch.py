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

import time, mutex, subprocess, math

from sr_robot_msgs.msg import sendupdate, joint, Biotac, BiotacAll
from sensor_msgs.msg import *
from std_msgs.msg import Float64

#the threshold for pac0 above which the tactile is considered "pressed"
PAC0_THRESHOLD = 2000

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

    #A vector containing the different callbacks, in the same order
    # as the tactiles.
    fingers_pressed_functions = [self.ff_pressed, self.mf_pressed, self.rf_pressed,
                                 self.lf_pressed, self.th_pressed]

    #The hand publishers:
    # we use a dictionnary of publishers, because on the etherCAT hand
    # you have one publisher per controller.
    hand_publishers = self.create_hand_publishers()

    #The arm publisher:
    # publish the message to the /sr_arm/sendupdate topic.
    arm_publisher = rospy.Publisher('/sr_arm/sendupdate', sendupdate)

    #A boolean used in this demo: set to true while an action is running
    # just so we don't do 2 actions at once
    action_running = mutex.mutex()

    def __init__(self):
        #send the start position to the hand
        self.hand_publish(self.start_pos_hand)
        #send the start position to the arm
        self.arm_publisher.publish( sendupdate(len(self.start_pos_arm), self.start_pos_arm) )

        #wait for the node to be initialized and then go to the starting position
        time.sleep(1)
        rospy.loginfo("OK, ready for the demo")

        # We subscribe to the data being published by the biotac sensors.
        self.sub_ff = rospy.Subscriber("/tactiles", BiotacAll, self.callback_biotacs, queue_size=1)

    def create_hand_publishers(self):
        """
        Creates a dictionnary of publishers to send the targets to the controllers
        on /sh_??j?_mixed_position_velocity_controller/command
        """
        hand_pub = {}

        for joint in ["FFJ0", "FFJ3", "FFJ4",
                      "MFJ0", "MFJ3", "MFJ4",
                      "RFJ0", "RFJ3", "RFJ4",
                      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                      "WRJ1", "WRJ2" ]:
            hand_pub[joint] = rospy.Publisher('/sh_'+joint.lower()+'_mixed_position_velocity_controller/command', Float64)

        return hand_pub

    def hand_publish(self, pose):
        """
        Publishes the given pose to the correct controllers for the hand.
        The targets are converted in radians.
        """
        for joint in pose:
            self.hand_publishers[joint.joint_name].publish( math.radians(joint.joint_target) )

    def callback_biotacs(self, msg):
        """
        The callback function for the biotacs. Checks if one of the finger
        was pressed (filter the noise). If it is the case, call the
        corresponding function.

        @msg is the message containing the biotac data
        """
        #loop through the five tactiles
        for index,tactile in enumerate(msg.tactiles):
            #here we're just checking pac0 (the pressure)
            # to see if a finger has been pressed, but you have
            # access to the whole data from the sensor
            # (look at sr_robot_msgs/msg/Biotac.msg)
            if tactile.pac0 >= PAC0_THRESHOLD:
                # the tactile has been pressed, call the
                # corresponding function
                self.fingers_pressed_functions[index](tactile.pac0)

    def ff_pressed(self,data):
        """
        The first finger was pressed.

        If no action is currently running, we set the ShoulderJRotate
        to a negative value proportional to the pressure received.

        @param data: the pressure value (pac0)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger sensor was pressed
        p = subprocess.Popen('beep')

        #rotate the trunk to (data_received * min_pos)
        # convert data to be in [0., 1.]
        data /= 4000.
        if data > 1.:
            data = 1.
        target_sh_rot    = 0.  + data * (-45.0)
        target_sh_swing  = 20. + data * (-10.0)
        target_elb_swing = 90. + data * (-10.0)
        target_elb_rot   = 0.  + data * (-45.0)

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

    def mf_pressed(self, data):
        """
        The middle finger was pressed.

        If no action is currently running, we set the ShoulderJRotate
        to a positive value proportional to the pressure received.

        @param data: the pressure value (pac0)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed
        p = subprocess.Popen('beep')

        #rotate the trunk to (data_received * min_pos)
        # convert data to be in [0., 1.]
        data /= 4000.
        if data > 1.:
            data = 1.
        target_sh_rot    = 0.  + data * (45.0)
        target_sh_swing  = 20. + data * (20.0)
        target_elb_swing = 90. + data * (20.0)
        target_elb_rot   = 0.  + data * (45.0)

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


    def rf_pressed(self, data):
        """
        The ring finger was pressed.

        If no action is currently running, we make a beep
        but don't do anything else.

        @param data: the pressure value (pac0)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed
        p = subprocess.Popen('beep')

        rospy.loginfo("RF touched, not doing anything.")

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def lf_pressed(self, data):
        """
        The little finger was pressed.

        If no action is currently running, we reset the arm and
        hand to their starting position

        @param data: the pressure value (pac0)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger pressed
        p = subprocess.Popen('beep')

        rospy.loginfo("LF touched, going to start position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the start position to the hand
        self.hand_publish( self.start_pos_hand )
        #send the start position to the arm
        self.arm_publisher.publish(sendupdate(len(self.start_pos_arm), self.start_pos_arm))

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def th_pressed(self, data):
        """
        The thumb was pressed.

        If no action is currently running, we send the thumb_up
        targets to the hand.

        @param data: the pressure value (pac0)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger was pressed
        p = subprocess.Popen('beep')

        rospy.loginfo("TH touched, going to thumb up position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the thumb_up_position to the hand
        self.hand_publish( self.thumb_up_pos_hand )

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

