import rospy
import pprint
from copy import deepcopy

from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListState
from moveit_msgs.msg import RobotState

class SrRobotStateExporter(object):
    def __init__(self, start_dictionary={}):
        # define service proxies
        self._get_state = rospy.ServiceProxy("/get_robot_state", GetState)
        self._has_state = rospy.ServiceProxy("/has_robot_state", HasState)
        self._list_states = rospy.ServiceProxy("/list_robot_states", ListState)
        self._save_state = rospy.ServiceProxy("/save_robot_state", SaveState)
        self._dictionary = deepcopy(start_dictionary)

    def extract_list(self, list_of_states):
        for name in list_of_states:
            self.extract_one_state(name)

    def extract_one_state(self, name):
        if self._has_state(name, '').exists:
            state = self._get_state(name,'').state.joint_state
            names = state.name
            position = state.position
            self._dictionary[name] = {name : position[n] for n, name in enumerate(names) }
        else:
            rospy.logerr("State %s not present in the warehouse." % name)

    def extract_from_trajectory(self, dictionary_trajectory):
        for entry in dictionary_trajectory:
            if 'name' in entry:
                self.extract_one_state(entry['name'])

    def extract_all(self):
        for state in self._list_states("","").states:
            self.extract_one_state(state)

    def output_module(self, file_name):
        pp = pprint.PrettyPrinter()
        with open(file_name, "w") as output:
            output.write('warehouse_states = %s\n' % pp.pformat(self._dictionary))

    def convert_trajectory(self, named_trajectory):
        new_trajectory = []
        for entry in named_trajectory:
            new_entry = deepcopy(entry)
            if 'name' in new_entry:
                if new_entry['name'] in self._dictionary:
                    new_entry['joint_angles'] = self._dictionary[new_entry['name']]
                    new_entry.pop('name')
                else:
                    rospy.logwarn("Entry named %s not present in dictionary. Not replacing." % name)
            new_trajectory.append(new_entry)
        return new_trajectory

    def repopulate_warehouse(self):
        for name in self._dictionary:
            if self._has_state(name, '').exists:
                rospy.logwarn("State named %s already in warehouse, not re-adding." % name)
            else:
                state = RobotState()
                state.joint_state.name = self._dictionary[name].keys()
                state.joint_state.position = self._dictionary[name].values()
                self._save_state(name,'',state)
