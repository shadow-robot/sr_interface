#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2024 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.

import sys
import os
import csv
import rosbag
import rospy

PWM_TOPIC_SUFFIXES = ['ffj0', 'ffj3', 'ffj4', 'lfj0', 'lfj3', 'lfj4', 'lfj5', 'mfj0', 'mfj3',
                      'mfj4', 'rfj0', 'rfj3', 'rfj4', 'thj1', 'thj2', 'thj3', 'thj4', 'thj5',
                      'wrj1', 'wrj2']

filename = sys.argv[1]
print("Reading the rosbag file")
bag = rosbag.Bag(filename)

print("Extracting trial times")
trial_times = []
for topic, msg, t in bag.read_messages(topics=['/trial_state']):
    if topic != '/trial_state':
        continue
    if msg.frame_id.endswith('start'):
        trial_times.append([t, None])
    elif msg.frame_id.endswith('end') and len(trial_times) > 0:
        trial_times[-1][1] = t

print("Creating csv file")
# with open(filename[:-4] + '_joint_states.csv', mode='w') as data_file:
#     data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#     data_writer.writerow(["trial_n",
#                           "time",
#                           "joint_state_pos",
#                           "joint_state_vel",
#                           "joint_state_effort"])

#     trial_n = 0
#     latest_msg = None
#     record_start_state = True
#     for topic, msg, t in bag.read_messages(topics=['/joint_states']):
#         if trial_n >= len(trial_times):
#             break

#         # =========== CSV of all joint states ===========
#         if t >= trial_times[trial_n][0] and t <= trial_times[trial_n][1]:
#             data_writer.writerow([trial_n, t, msg.position, msg.velocity, msg.effort])
#         elif t > trial_times[trial_n][1]:
#             trial_n += 1

#         # =========== CSV of only end joint states ===========
#         # if t >= trial_times[trial_n][0] and t <= trial_times[trial_n][1]:
#         #     if record_start_state:
#         #         data_writer.writerow(["Open", t, msg.position, msg.velocity, msg.effort])
#         #         record_start_state = False
#         #     latest_msg = msg
#         # elif t > trial_times[trial_n][1]:
#         #     data_writer.writerow([trial_n, t, latest_msg.position, latest_msg.velocity, 
#         #                           latest_msg.effort])
#         #     trial_n += 1

with open(filename[:-4] + '_pwms.csv', mode='w') as data_file:
    data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    data_writer.writerow(["trial_n", "time"] + [name + '_pwm' for name in PWM_TOPIC_SUFFIXES])

    trial_n = 0
    for topic, msg, t in bag.read_messages(topics=['/sh_rh_' + PWM_TOPIC_SUFFIXES[0] + '_position_controller/state']):
        if trial_n >= len(trial_times):
            break

        if t >= trial_times[trial_n][0] and t <= trial_times[trial_n][1]:
            pwm_list = [msg.command]
            print("=====================================")
            for name in PWM_TOPIC_SUFFIXES[1:]:
                for sub_topic, sub_msg, sub_t in bag.read_messages(topics=['/sh_rh_' + name + '_position_controller/state']):
                    if sub_t < t:
                        continue
                    pwm_list.append(sub_msg.command)
                    break
            print(pwm_list)
            data_writer.writerow([trial_n, t] + pwm_list)
        elif t > trial_times[trial_n][1]:
            trial_n += 1

print("Done")
bag.close()
