#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
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


import rospy
import random
import time
from sr_hand.shadowhand_commander import Commander

rospy.init_node('receiver_example')
c = Commander()

##########
# RANGES #
##########

# Minimum alllowed range for the joints in this particular script
min_range = {"THJ2": -40, "THJ3": -12, "THJ4": 0, "THJ5": -55,
             "FFJ0": 20, "FFJ3": 0, "FFJ4": -20,
             "MFJ0": 20, "MFJ3": 0, "MFJ4": -10,
             "RFJ0": 20, "RFJ3": 0, "RFJ4": -10,
             "LFJ0": 20, "LFJ3": 0, "LFJ4": -20, "LFJ5": 0,
             "WRJ1": -20, "WRJ2": -10,
             "interpolation_time": 0.0}

# Maximum alllowed range for the joints in this particular script
max_range = {"THJ2": 20, "THJ3": 12, "THJ4": 70, "THJ5": 0,
             "FFJ0": 110, "FFJ3": 90, "FFJ4": 0,
             "MFJ0": 110, "MFJ3": 90, "MFJ4": 0,
             "RFJ0": 110, "RFJ3": 90, "RFJ4": 0,
             "LFJ0": 110, "LFJ3": 90, "LFJ4": 0, "LFJ5": 1,
             "WRJ1": 10, "WRJ2": 5,
             "interpolation_time": 4.0}



####################
# POSE DEFINITIONS #
####################

# starting position for the hand
start_pos = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
             "FFJ0": 0, "FFJ3": 0, "FFJ4": 0,
             "MFJ0": 0, "MFJ3": 0, "MFJ4": 0,
             "RFJ0": 0, "RFJ3": 0, "RFJ4": 0,
             "LFJ0": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
             "WRJ1": 0, "WRJ2": 0}
# Start position for the Hand
pregrasp_pos = {"THJ2": 12, "THJ3": 15, "THJ4": 69, "THJ5": -23,
                "FFJ0": 40, "FFJ3": 21, "FFJ4": -15,
                "MFJ0": 40, "MFJ3": 21, "MFJ4": 0,
                "RFJ0": 40, "RFJ3": 21, "RFJ4": -7,
                "LFJ0": 40, "LFJ3": 21, "LFJ4": -10, "LFJ5": 0,
                "WRJ1": 0, "WRJ2": 0}
# Close position for the Hand
grasp_pos = {"THJ2": 30, "THJ3": 15, "THJ4": 69, "THJ5": -3,
             "FFJ0": 77, "FFJ3": 67, "FFJ4": -19,
             "MFJ0": 82, "MFJ3": 62, "MFJ4": 0,
             "RFJ0": 89, "RFJ3": 64, "RFJ4": -18,
             "LFJ0": 97, "LFJ3": 64, "LFJ4": -19, "LFJ5": 0,
             "WRJ1": 0, "WRJ2": 0,
             "interpolation_time": 10.0}
# Random position for the Hand (initialied at 0)
rand_pos = {"THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
            "FFJ0": 0, "FFJ3": 0, "FFJ4": 0,
            "MFJ0": 0, "MFJ3": 0, "MFJ4": 0,
            "RFJ0": 0, "RFJ3": 0, "RFJ4": 0,
            "LFJ0": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
            "WRJ1": 0, "WRJ2": 0,
            "interpolation_time": 0.0}
# flex first finger
flex_ff = {"FFJ0": 180, "FFJ3": 90, "FFJ4": 0}
# extend first finger
ext_ff = {"FFJ0": 0, "FFJ3": 0, "FFJ4": 0}
# flex middle finger
flex_mf = {"MFJ0": 180, "MFJ3": 90, "MFJ4": 0}
# extend middle finger
ext_mf = {"MFJ0": 0, "MFJ3": 0, "MFJ4": 0}
# flex ring finger
flex_rf = {"RFJ0": 180, "RFJ3": 90, "RFJ4": 0}
# extend ring finger
ext_rf = {"RFJ0": 0, "RFJ3": 0, "RFJ4": 0}
# flex little finger
flex_lf = {"LFJ0": 180, "LFJ3": 90, "LFJ4": 0}
# extend middle finger
ext_lf = {"LFJ0": 0, "LFJ3": 0, "LFJ4": 0}
# flex thumb step 1
flex_th_1 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 70, "THJ5": 0}
# flex thumb step 2
flex_th_2 = {"THJ1": 20, "THJ2": 40, "THJ3": 10, "THJ4": 70, "THJ5": 58}
# extend thumb step 1
ext_th_1 = {"THJ1": 10, "THJ2": 20, "THJ3": 5, "THJ4": 35, "THJ5": 25}
# extend thumb step 2
ext_th_2 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0}
# zero thumb
zero_th = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"THJ4": 70}
# O.K. with first finger
ff_ok = {"THJ1": 15, "THJ2": 20, "THJ3": 0, "THJ4": 56, "THJ5": 11,
         "FFJ0": 75, "FFJ3": 65, "FFJ4": -0.2,
         "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
         "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"THJ1": 5, "THJ2": 12, "THJ3": -10, "THJ4": 60, "THJ5": 2,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
            "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. with middle finger
mf_ok = {"THJ1": 15, "THJ2": 18, "THJ3": 7, "THJ4": 66, "THJ5": 23,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 88, "MFJ3": 63, "MFJ4": 11,
         "RFJ0": 50, "RFJ3": 18, "RFJ4": -10,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"THJ1": 5, "THJ2": -5, "THJ3": 15, "THJ4": 70, "THJ5": 19,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
            "RFJ0": 50, "RFJ3": 18, "RFJ4": -19,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7}
# O.K. with ring finger
rf_ok = {"THJ1": 15, "THJ2": 5, "THJ3": 15, "THJ4": 70, "THJ5": 42,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
         "RFJ0": 103, "RFJ3": 52, "RFJ4": -19,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"THJ1": 5, "THJ2": 4.5, "THJ3": 8, "THJ4": 60, "THJ5": 21,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
            "RFJ0": 30, "RFJ3": 6, "RFJ4": 0.5,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 7}
# O.K. with little finger
lf_ok = {"THJ1": 30, "THJ2": 8, "THJ3": 10, "THJ4": 69, "THJ5": 26,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 15, "MFJ3": 4, "MFJ4": -1,
         "RFJ0": 15, "RFJ3": 6, "RFJ4": 0.5,
         "LFJ0": 96, "LFJ3": 19, "LFJ4": -7, "LFJ5": 45}
# zero wrist
zero_wr = {"WRJ1": 0, "WRJ2": 0}
# north wrist
n_wr = {"WRJ1": 15, "WRJ2": 0}
# south wrist
s_wr = {"WRJ1": -20, "WRJ2": 0}
# east wrist
e_wr = {"WRJ1": 0, "WRJ2": 8}
# west wrist
w_wr = {"WRJ1": 0, "WRJ2": -14}
# northeast wrist
ne_wr = {"WRJ1": 15, "WRJ2": 8}
# northwest wrist
nw_wr = {"WRJ1": 15, "WRJ2": -14}
# southweast wrist
sw_wr = {"WRJ1": -20, "WRJ2": -14}
# southeast wrist
se_wr = {"WRJ1": -20, "WRJ2": 8}
# lateral lf ext side
l_ext_lf = {"LFJ4": -15}
# lateral rf ext side
l_ext_rf = {"RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"FFJ4": 15}
# lateral all int side
l_int_all = {"FFJ4": -15, "MFJ4": -15, "RFJ4": 15, "LFJ4": 15}
# lateral all ext side
l_ext_all = {"FFJ4": 15, "MFJ4": 15, "RFJ4": -15, "LFJ4": -15}
# lateral ff int side
l_int_ff = {"FFJ4": -15}
# lateral mf int side
l_int_mf = {"MFJ4": -15}
# lateral rf int side
l_int_rf = {"RFJ4": 15}
# lateral lf int side
l_int_lf = {"LFJ4": 15}
# all zero
l_zero_all = {"FFJ4": 0, "MFJ4": 0, "RFJ4": 0, "LFJ4": 0}
# spock
l_spock = {"FFJ4": -20, "MFJ4": -20, "RFJ4": -20, "LFJ4": -20}
# grasp for shaking hands step 1
shake_grasp_1 = {"THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
                 "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
                 "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
                 "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
                 "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10}
# grasp for shaking hands step 2
shake_grasp_2 = {"THJ1": 21, "THJ2": 21, "THJ3": 10, "THJ4": 42, "THJ5": 21,
                 "FFJ0": 75, "FFJ3": 29, "FFJ4": 0,
                 "MFJ0": 75, "MFJ3": 41, "MFJ4": 0,
                 "RFJ0": 75, "RFJ3": 41, "RFJ4": 0,
                 "LFJ0": 100, "LFJ3": 41, "LFJ4": 0, "LFJ5": 0}
# store step 1 PST
store_1_PST = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 60, "THJ5": 0,
               "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
               "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
               "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
               "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
               "WRJ1": 0, "WRJ2": 0}
# store step 2 PST
store_2_PST = {"THJ1": 50, "THJ2": 12, "THJ3": 0, "THJ4": 60, "THJ5": 27,
               "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
               "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
               "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
               "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
               "WRJ1": 0, "WRJ2": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 30, "THJ5": 0,
                  "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
                  "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
                  "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
                  "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
                  "WRJ1": 0, "WRJ2": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"THJ1": 20, "THJ2": 36, "THJ3": 0, "THJ4": 30, "THJ5": -3,
                  "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
                  "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
                  "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
                  "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
                  "WRJ1": 0, "WRJ2": 0}
# store step 3
store_3 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 65, "THJ5": 0}


########################
# FUNCTION DEFINITIONS #
########################

def secuence_ff():
    # Start secuence 1
    rospy.sleep(1)
    c.move_hand(store_3)
    rospy.sleep(1)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    c.move_hand(flex_ff)
    rospy.sleep(1.0)
    c.move_hand(ext_ff)
    rospy.sleep(1.0)
    c.move_hand(flex_mf)
    rospy.sleep(1.0)
    c.move_hand(ext_mf)
    rospy.sleep(1.0)
    c.move_hand(flex_rf)
    rospy.sleep(1.0)
    c.move_hand(ext_rf)
    rospy.sleep(1.0)
    c.move_hand(flex_lf)
    rospy.sleep(1.0)
    c.move_hand(ext_lf)
    rospy.sleep(1.0)
    c.move_hand(flex_th_1)
    rospy.sleep(0.7)
    tmp = flex_th_2.copy()
    tmp.update({'interpolation_time': 2.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] ):
            c.move_hand(hand_pos)
            print 'Thumb contact'
            break

    rospy.sleep(1)
    c.move_hand(ext_th_2)
    rospy.sleep(0.5)
    c.move_hand(l_ext_lf)
    rospy.sleep(0.5)
    c.move_hand(l_ext_rf)
    rospy.sleep(0.5)
    c.move_hand(l_ext_mf)
    rospy.sleep(0.5)
    c.move_hand(l_ext_ff)
    rospy.sleep(0.5)
    c.move_hand(l_int_all)
    rospy.sleep(0.5)
    c.move_hand(l_ext_all)
    rospy.sleep(0.5)
    c.move_hand(l_int_ff)
    rospy.sleep(0.5)
    c.move_hand(l_int_mf)
    rospy.sleep(0.5)
    c.move_hand(l_int_rf)
    rospy.sleep(0.5)
    c.move_hand(l_int_lf)
    rospy.sleep(0.5)
    c.move_hand(l_zero_all)
    rospy.sleep(0.5)
    c.move_hand(l_spock)
    rospy.sleep(0.5)
    c.move_hand(l_zero_all)
    rospy.sleep(0.5)
    c.move_hand(pre_ff_ok)
    rospy.sleep(1)
    tmp = ff_ok.copy()
    tmp.update({'interpolation_time': 2.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['FF'] > force_zero['FF'] ):
            c.move_hand(hand_pos)
            print 'First finger contact'
            break

    rospy.sleep(1)
    c.move_hand(ff2mf_ok)
    rospy.sleep(0.8)
    tmp = mf_ok.copy()
    tmp.update({'interpolation_time': 2.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['MF'] > force_zero['MF'] ):
            c.move_hand(hand_pos)
            print 'Middle finger contact'
            break

    rospy.sleep(1)
    c.move_hand(mf2rf_ok)
    rospy.sleep(0.8)
    tmp = rf_ok.copy()
    tmp.update({'interpolation_time': 2.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['RF'] > force_zero['RF'] ):
            c.move_hand(hand_pos)
            print 'Ring finger contact'
            break

    rospy.sleep(1)
    c.move_hand(rf2lf_ok)
    rospy.sleep(0.8)
    tmp = lf_ok.copy()
    tmp.update({'interpolation_time': 2.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['LF'] > force_zero['LF'] ):
            c.move_hand(hand_pos)
            print 'Little finger contact'
            break

    rospy.sleep(1)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    c.move_hand(flex_ff)
    rospy.sleep(0.4)
    c.move_hand(flex_mf)
    rospy.sleep(0.4)
    c.move_hand(flex_rf)
    rospy.sleep(0.4)
    c.move_hand(flex_lf)
    rospy.sleep(0.4)
    c.move_hand(ext_ff)
    rospy.sleep(0.4)
    c.move_hand(ext_mf)
    rospy.sleep(0.4)
    c.move_hand(ext_rf)
    rospy.sleep(0.4)
    c.move_hand(ext_lf)
    rospy.sleep(0.4)
    c.move_hand(flex_ff)
    rospy.sleep(0.4)
    c.move_hand(flex_mf)
    rospy.sleep(0.4)
    c.move_hand(flex_rf)
    rospy.sleep(0.4)
    c.move_hand(flex_lf)
    rospy.sleep(0.4)
    c.move_hand(ext_ff)
    rospy.sleep(0.4)
    c.move_hand(ext_mf)
    rospy.sleep(0.4)
    c.move_hand(ext_rf)
    rospy.sleep(0.4)
    c.move_hand(ext_lf)
    rospy.sleep(0.4)
    c.move_hand(flex_ff)
    rospy.sleep(0.4)
    c.move_hand(flex_mf)
    rospy.sleep(0.4)
    c.move_hand(flex_rf)
    rospy.sleep(0.4)
    c.move_hand(flex_lf)
    rospy.sleep(0.4)
    c.move_hand(ext_ff)
    rospy.sleep(0.4)
    c.move_hand(ext_mf)
    rospy.sleep(0.4)
    c.move_hand(ext_rf)
    rospy.sleep(0.4)
    c.move_hand(ext_lf)
    rospy.sleep(1.5)
    c.move_hand(pre_ff_ok)
    rospy.sleep(1)
    tmp = ff_ok.copy()
    tmp.update({'interpolation_time': 3.0})
    c.move_hand(tmp)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = c.get_hand_position()
        # If the tacticle sensor is triggered stop movement
        if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['FF'] > force_zero['FF'] ):
            c.move_hand(hand_pos)
            print 'First finger contact'
            break

    rospy.sleep(1.5)
    c.move_hand(ne_wr)
    rospy.sleep(1.4)
    c.move_hand(nw_wr)
    rospy.sleep(1.4)
    c.move_hand(sw_wr)
    rospy.sleep(1.4)
    c.move_hand(se_wr)
    rospy.sleep(1.4)
    c.move_hand(ne_wr)
    rospy.sleep(0.7)
    c.move_hand(nw_wr)
    rospy.sleep(0.7)
    c.move_hand(sw_wr)
    rospy.sleep(0.7)
    c.move_hand(se_wr)
    rospy.sleep(0.7)
    c.move_hand(zero_wr)
    rospy.sleep(0.4)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    return


def secuence_mf():
    # Start the secuence 2
    rospy.sleep(2.0)
    # Initialize wake time
    wake_time = time.time()

    while True:
        # Check if any of the tactile senors have been triggered
        # If so, send the Hand to its start position
        read_tactile_values()
        if ( tactile_values['FF'] > force_zero['FF'] or
                     tactile_values['MF'] > force_zero['MF'] or
                     tactile_values['RF'] > force_zero['RF'] or
                     tactile_values['LF'] > force_zero['LF'] or
                     tactile_values['TH'] > force_zero['TH'] ):

            c.move_hand(start_pos)
            print 'HAND TOUCHED!'
            rospy.sleep(2.0)

        if ( tactile_values['TH'] > force_zero['TH'] ):
            break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in rand_pos:
                    rand_pos[i] = random.randrange(min_range[i], max_range[i])

                rand_pos['FFJ4'] = random.randrange(min_range['FFJ4'], rand_pos['MFJ4'])
                rand_pos['LFJ4'] = random.randrange(min_range['LFJ4'], rand_pos['RFJ4'])
                rand_pos['interpolation_time'] = max_range['interpolation_time'] * random.random()

                c.move_hand(rand_pos)
                wake_time = time.time() + rand_pos['interpolation_time'] * 0.9
    return


def secuence_rf():
    # Start the secuence 3
    rospy.sleep(0.5)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    c.move_hand(shake_grasp_1)
    rospy.sleep(2.5)
    c.move_hand(shake_grasp_2)
    rospy.sleep(1)
    c.move_hand(e_wr)
    rospy.sleep(0.4)
    c.move_hand(w_wr)
    rospy.sleep(0.4)
    c.move_hand(zero_wr)
    rospy.sleep(0.8)
    c.move_hand(shake_grasp_1)
    rospy.sleep(1.5)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    return


def secuence_lf():
    # Start the secuence 4
    # Trigger flag array
    trigger = [0, 0, 0, 0, 0]

    # Move Hand to zero position
    c.move_hand(start_pos)
    rospy.sleep(2.0)

    # Move Hand to starting position
    c.move_hand(pregrasp_pos)
    rospy.sleep(2.0)

    # Move Hand to close position
    c.move_hand(grasp_pos)
    offset1 = 3

    # Initialize end time
    end_time = time.time() + 11

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()

        # Record current joint positions
        hand_pos = c.get_hand_position()

        # If any tacticle sensor has been triggered, send
        # the corresponding digit to its current position
        if ( tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0 ):
            c.move_hand({"FFJ0": hand_pos['FFJ0'] + offset1, "FFJ3": hand_pos['FFJ3'] + offset1})
            print 'First finger contact'
            trigger[0] = 1

        if ( tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0 ):
            c.move_hand({"MFJ0": hand_pos['MFJ0'] + offset1, "MFJ3": hand_pos['MFJ3'] + offset1})
            print 'Middle finger contact'
            trigger[1] = 1

        if ( tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0 ):
            c.move_hand({"RFJ0": hand_pos['RFJ0'] + offset1, "RFJ3": hand_pos['RFJ3'] + offset1})
            print 'Ring finger contact'
            trigger[2] = 1

        if ( tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0 ):
            c.move_hand({"LFJ0": hand_pos['LFJ0'] + offset1, "LFJ3": hand_pos['LFJ3'] + offset1})
            print 'Little finger contact'
            trigger[3] = 1

        if ( tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0 ):
            c.move_hand({"THJ2": hand_pos['THJ2'] + offset1, "THJ5": hand_pos['THJ5'] + offset1})
            print 'Thumb contact'
            trigger[4] = 1

        if ( trigger[0] == 1 and
                     trigger[1] == 1 and
                     trigger[2] == 1 and
                     trigger[3] == 1 and
                     trigger[4] == 1 ):
            break

        if time.time() > end_time:
            break

    # Send all joints to current position to compensate
    # for minor offsets created in the previous loop
    hand_pos = c.get_hand_position()
    c.move_hand(hand_pos)
    rospy.sleep(2.0)

    # Generate new values to squeeze object slightly
    offset2 = 3
    squeeze = hand_pos.copy()
    squeeze.update({"THJ5": hand_pos['THJ5'] + offset2, "THJ2": hand_pos['THJ2'] + offset2,
                    "FFJ3": hand_pos['FFJ3'] + offset2, "FFJ0": hand_pos['FFJ0'] + offset2,
                    "MFJ3": hand_pos['MFJ3'] + offset2, "MFJ0": hand_pos['MFJ0'] + offset2,
                    "RFJ3": hand_pos['RFJ3'] + offset2, "RFJ0": hand_pos['RFJ0'] + offset2,
                    "LFJ3": hand_pos['LFJ3'] + offset2, "LFJ0": hand_pos['LFJ0'] + offset2})

    # Squeeze object gently
    c.move_hand(squeeze)
    rospy.sleep(0.5)
    c.move_hand(hand_pos)
    rospy.sleep(0.5)
    c.move_hand(squeeze)
    rospy.sleep(0.5)
    c.move_hand(hand_pos)
    rospy.sleep(2.0)
    c.move_hand(pregrasp_pos)
    rospy.sleep(2.0)
    c.move_hand(start_pos)
    rospy.sleep(2.0)

    return


def secuence_th():
    # Start the secuence 5
    rospy.sleep(0.5)
    c.move_hand(start_pos)
    rospy.sleep(1.5)
    return


def zero_tactile_sensors():
    # Move Hand to zero position
    rospy.sleep(0.5)
    c.move_hand(start_pos)

    print '\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n'
    # raw_input ('Press ENTER to continue')
    rospy.sleep(1.0)

    for _ in xrange(1, 1000):
        # Read current state of tactile sensors to zero them
        read_tactile_values()

        if tactile_values['FF'] > force_zero['FF']:
            force_zero['FF'] = tactile_values['FF']
        if tactile_values['MF'] > force_zero['MF']:
            force_zero['MF'] = tactile_values['MF']
        if tactile_values['RF'] > force_zero['RF']:
            force_zero['RF'] = tactile_values['RF']
        if tactile_values['LF'] > force_zero['LF']:
            force_zero['LF'] = tactile_values['LF']
        if tactile_values['TH'] > force_zero['TH']:
            force_zero['TH'] = tactile_values['TH']

    force_zero['FF'] = force_zero['FF'] + 3
    force_zero['MF'] = force_zero['MF'] + 3
    force_zero['RF'] = force_zero['RF'] + 3
    force_zero['LF'] = force_zero['LF'] + 3
    force_zero['TH'] = force_zero['TH'] + 3

    print 'Force Zero', force_zero

    rospy.loginfo("\n\nOK, ready for the demo")

    print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
    print "   FF: Standard Demo"
    print "   MF: Shy Hand Demo"
    print "   RF: Handshake Demo"
    print "   LF: Grasp Demo"
    print "   TH: Open Hand"

    return


def read_tactile_values():
    # Read tactile type
    tactile_type = c.get_tactile_type()
    # Read current state of tactile sensors
    tactile_state = c.get_tactile_state()

    if tactile_type == "biotac":
        tactile_values['FF'] = tactile_state.tactiles[0].pdc
        tactile_values['MF'] = tactile_state.tactiles[1].pdc
        tactile_values['RF'] = tactile_state.tactiles[2].pdc
        tactile_values['LF'] = tactile_state.tactiles[3].pdc
        tactile_values['TH'] = tactile_state.tactiles[4].pdc

    elif tactile_type == "PST":
        tactile_values['FF'] = tactile_state.pressure[0]
        tactile_values['MF'] = tactile_state.pressure[1]
        tactile_values['RF'] = tactile_state.pressure[2]
        tactile_values['LF'] = tactile_state.pressure[3]
        tactile_values['TH'] = tactile_state.pressure[4]

    elif tactile_type is None:
        print "You don't have tactile sensors. Talk to your Shadow representative to purchase some"

    return

########
# MAIN #
########

# Zero values in dictionary for tactile sensors (initialized at 0)
force_zero = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Initialize values for current tactile values
tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Zero tactile sensors
zero_tactile_sensors()

while not rospy.is_shutdown():
    # Check the state of the tactile senors
    read_tactile_values()

    # If the tactile is touched, trigger the corresponding function
    if (tactile_values['FF'] > force_zero['FF']):
        print 'First finger contact'
        secuence_ff()
        print 'FF demo completed'
        zero_tactile_sensors()
    if (tactile_values['MF'] > force_zero['MF']):
        print 'Middle finger contact'
        secuence_mf()
        print 'MF demo completed'
        zero_tactile_sensors()
    if (tactile_values['RF'] > force_zero['RF']):
        print 'Ring finger contact'
        secuence_rf()
        print 'RF demo completed'
        zero_tactile_sensors()
    if (tactile_values['LF'] > force_zero['LF']):
        print 'Little finger contact'
        secuence_lf()
        print 'LF demo completed'
        zero_tactile_sensors()
    if (tactile_values['TH'] > force_zero['TH']):
        print 'Thumb finger contact'
        secuence_th()
        print 'TH demo completed'
        zero_tactile_sensors()
