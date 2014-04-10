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

# This demo is a simple script that reads data from the tactile sensors in the finger tips and does some actions in response.
# This version can work with PST tactiles and (with a modification in the subscriber topic name) with Biotac sensors
# When a finger tip is pressed and the value of the pressure exceeds a predefined threshold an action is triggered
# There is a different action defined for each finger. Actions involve one of the following things:
# - send a predefined pose command to the hand
# - send a predefined pose command to the arm (if there is not an arm the arm commands will be ignored)
# - the PC beeps (you should install the beep (sudo apt-get install beep)


import roslib; roslib.load_manifest('sr_example')
import rospy

import time, mutex, subprocess, math

from sr_robot_msgs.msg import sendupdate, joint, Biotac, BiotacAll, ShadowPST
from sensor_msgs.msg import *
from std_msgs.msg import Float64

#the threshold for pdc above which the Biotac tactile is considered "pressed"
PDC_THRESHOLD = 2000
#the threshold for the PSTs above which the tactile is considered "pressed"
PST_THRESHOLD = 400

class FancyDemo(object):
    # type of controller that is running
    controller_type = "_position_controller"

    # starting position for the hand (DON't use until reviewed. Should be executed in two movement sequences)
    start_pos_hand = { "THJ1":21,
                       "THJ2":25,
                       "THJ3":0,
                       "THJ4":50,
                       "THJ5":-6,
                       "FFJ0":180,
                       "FFJ3":90,
                       "FFJ4":0,
                       "MFJ0":180,
                       "MFJ3":90,
                       "MFJ4":0,
                       "RFJ0":180,
                       "RFJ3":90,
                       "RFJ4":0,
                       "LFJ0":180,
                       "LFJ3":90,
                       "LFJ4":0,
                       "LFJ5":0,
                       "WRJ1":0,
                       "WRJ2":0 }
    # starting position for the hand
    extended_pos_hand = {  "THJ1":0,
                           "THJ2":0,
                           "THJ3":0,
                           "THJ4":0,
                           "THJ5":-6,
                           "FFJ0":0,
                           "FFJ3":0,
                           "FFJ4":0,
                           "MFJ0":0,
                           "MFJ3":0,
                           "MFJ4":0,
                           "RFJ0":0,
                           "RFJ3":0,
                           "RFJ4":0,
                           "LFJ0":0,
                           "LFJ3":0,
                           "LFJ4":0,
                           "LFJ5":0,
                           "WRJ1":0,
                           "WRJ2":0 }

    #thumb up position for the hand
    thumb_up_pos_hand = {  "THJ1":0,
                           "THJ2":0,
                           "THJ3":0,
                           "THJ4":0,
                           "THJ5":0,
                           "FFJ0":180,
                           "FFJ3":90,
                           "FFJ4":0,
                           "MFJ0":180,
                           "MFJ3":90,
                           "MFJ4":0,
                           "RFJ0":180,
                           "RFJ3":90,
                           "RFJ4":0,
                           "LFJ0":180,
                           "LFJ3":90,
                           "LFJ4":0,
                           "LFJ5":0,
                           "WRJ1":0,
                           "WRJ2":10 }

    #starting position for the arm
    start_pos_arm = [ joint(joint_name = "ElbowJRotate", joint_target = 0),
                      joint(joint_name = "ElbowJSwing", joint_target = 59),
                      joint(joint_name = "ShoulderJRotate", joint_target = 0),
                      joint(joint_name = "ShoulderJSwing", joint_target = 33)
                      ]


    #The arm publisher:
    # publish the message to the /sr_arm/sendupdate topic.
    arm_publisher = rospy.Publisher('/sr_arm/sendupdate', sendupdate)

    #A boolean used in this demo: set to true while an action is running
    # just so we don't do 2 actions at once
    action_running = mutex.mutex()

    def __init__(self):
        #A vector containing the different callbacks, in the same order
        # as the tactiles.
        self.fingers_pressed_functions = [self.ff_pressed, self.mf_pressed, self.rf_pressed,
                                          self.lf_pressed, self.th_pressed]

        #The hand publishers:
        # we use a dictionnary of publishers, because on the etherCAT hand
        # you have one publisher per controller.
        self.hand_publishers = self.create_hand_publishers()

        #send the start position to the hand
        self.hand_publish(self.start_pos_hand)
        #send the start position to the arm
        self.arm_publisher.publish( sendupdate(len(self.start_pos_arm), self.start_pos_arm) )

        #wait for the node to be initialized and then go to the starting position
        time.sleep(1)
        rospy.loginfo("OK, ready for the demo")

        # We subscribe to the data being published by the biotac sensors.
        self.sub_biotacs = rospy.Subscriber("tactiles", BiotacAll, self.callback_biotacs, queue_size=1)
        self.sub_psts    = rospy.Subscriber("tactile", ShadowPST, self.callback_psts, queue_size=1)

    def create_hand_publishers(self):
        """
        Creates a dictionnary of publishers to send the targets to the controllers
        on /sh_??j?_position_controller/command
        """
        hand_pub = {}

        for joint in ["FFJ0", "FFJ3", "FFJ4",
                      "MFJ0", "MFJ3", "MFJ4",
                      "RFJ0", "RFJ3", "RFJ4",
                      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                      "WRJ1", "WRJ2" ]:
            hand_pub[joint] = rospy.Publisher('sh_'+joint.lower() + self.controller_type + '/command', Float64, latch=True)

        return hand_pub

    def hand_publish(self, pose):
        """
        Publishes the given pose to the correct controllers for the hand.
        The targets are converted in radians.
        """
        for joint, pos in pose.iteritems():
            self.hand_publishers[joint].publish( math.radians(pos) )

    def callback_biotacs(self, msg):
        """
        The callback function for the biotacs. Checks if one of the fingers
        was pressed (filter the noise). If it is the case, call the
        corresponding function.

        @msg is the message containing the biotac data
        """
        #loop through the five tactiles
        for index,tactile in enumerate(msg.tactiles):
            #here we're just checking pdc (the pressure)
            # to see if a finger has been pressed, but you have
            # access to the whole data from the sensor
            # (look at sr_robot_msgs/msg/Biotac.msg)
            if tactile.pdc >= PDC_THRESHOLD:
                # the tactile has been pressed, call the
                # corresponding function
                self.fingers_pressed_functions[index](tactile.pdc)

    def callback_psts(self, msg):
        """
        The callback function for the PSTs. Checks if one of the fingers
        was pressed (filter the noise). If it is the case, call the
        corresponding function.

        @msg is the message containing the biotac data
        """
        #loop through the five tactiles
        for index,tactile in enumerate(msg.pressure):
            #here we're just checking the pressure
            # to see if a finger has been pressed
            # 18456 is the value the PST takes when the sensor is not plugged in
            if tactile >= PST_THRESHOLD and tactile != 18456:
                # the tactile has been pressed, call the
                # corresponding function
                self.fingers_pressed_functions[index](tactile)

    def ff_pressed(self,data):
        """
        The first finger was pressed.

        If no action is currently running, we set the ShoulderJRotate
        to a negative value proportional to the pressure received.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger sensor was pressed
        
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

        #send the start position to the hand
        self.hand_publish( self.extended_pos_hand )

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def mf_pressed(self, data):
        """
        The middle finger was pressed.

        If no action is currently running, we move the arm angles
        to a positive value proportional to the pressure received.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed

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

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed
        rospy.loginfo("RF touched, not doing anything.")

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def lf_pressed(self, data):
        """
        The little finger was pressed.

        If no action is currently running, we reset the arm and
        hand to their starting position

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger pressed
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

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger was pressed
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
