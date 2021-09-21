#!/usr/bin/env python3
# Copyright 20 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# Reading the tactiles from the hand.

from __future__ import absolute_import
import rospy
from threading import Thread
import termios
import sys
import tty
import yaml
from sr_robot_commander.sr_hand_commander import SrHandCommander


class GraspExecution(object):
    def __init__(self):
        self.keyboard_pressed = False
        self.hand_commander = SrHandCommander(name='right_hand')
        self.grasp_yaml = {}

    def _open_yaml(self):
        grasp_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                                'sr_interface/sr_example/config/demo_grasps.yaml'
        with open(grasp_config_filename) as f:
            self.grasp_yaml = yaml.load(f, Loader=yaml.FullLoader)

    def _get_input(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        self._open_yaml()

        while True:
            input_val = self._get_input()
            if input_val == "1":
                self.execute_grasp("open_hand")
            elif input_val == "2":
                self.execute_grasp("close_hand")
            elif input_val == "3":
                self.execute_grasp("point")
            elif input_val == "4":
                self.execute_grasp("2_finger_pinch")
            elif input_val == "5":
                self.execute_grasp("3_finger_pinch")
            elif input_val == "6":
                self.execute_grasp("parallel_extension")
            elif input_val == "7":
                self.execute_grasp("grasp_sphere")


            if '0x1b' == hex(ord(input_val)):
                sys.exit(0)

    def execute_grasp(self, grasp):
        rospy.loginfo("Grasp {} started.".format(grasp))
        open_dict = dict(zip(self.grasp_yaml['joint_names'], self.grasp_yaml['grasps']['open_hand']))
        grasp_dict = dict(zip(self.grasp_yaml['joint_names'], self.grasp_yaml['grasps'][grasp]))
        self.hand_commander.move_to_joint_value_target_unsafe(open_dict, 5.0, True)
        self.hand_commander.move_to_joint_value_target_unsafe(grasp_dict, 5.0, True)
        rospy.sleep(2.0)
        rospy.loginfo("Grasp {} completed.".format(grasp))


if __name__ == "__main__":

    rospy.init_node("right_hand_demo", anonymous=True)

    # Keyboard thread for input
    kpd = GraspExecution()
    keyboard_thread = Thread(target=kpd.run)
    keyboard_thread.start()

    rospy.loginfo("\nPRESS 1-9 ON THE KEYBOARD TO SHOW A GRASP:\
                \n   1: Open Hand\
                \n   2: Close Hand\
                \n   3: Point\
                \n   4: 2 Finger Pinch\
                \n   5: 3 Finger Pinch\
                \n   6: Parallel Extension\
                \n   7: Grasp Sphere")
