#!/usr/bin/env python

import rospy
from sr_benchmarking.sr_benchmarking import AnnotationParserBase
from sr_benchmarking.sr_benchmarking import BenchmarkingBase
from collision_scene_1 import CreateScene1
from collision_scene_2 import CreateScene2
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState, MoveGroupActionResult
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy


class BenchmarkingScene(object):
    """
    Static python scenes for the benchmarking.
    """
    def __init__(self, scene_name):
        scenes_dict = {"simplified_1": CreateScene1,
                       "simplified_2": CreateScene2}
        scenes_dict[scene_name]()


class PlannerAnnotationParser(AnnotationParserBase):
    """
    Parses the annotations files that contains the benchmarking
    information.
    """
    def __init__(self, path_to_annotation, path_to_data):
        super(PlannerAnnotationParser, self).__init__(path_to_annotation, path_to_data)
        self.parse()

        self._init_planning()
        self._load_scene()
        self.benchmark()

    def to_xml(self, results):
        """
        Saves the results as a Jenkins parsable xml file for display.
        """
        print self.planner_data

    def _load_scene(self):
        """
        Loads the proper scene for the planner.
        It can be either a python static scene or bag containing an occupancy map.
        """
        scene = self._annotations["scene"]
        if scene["type"] == "python":
            BenchmarkingScene(scene["name"])

    def _init_planning(self):
        """
        Initialises the needed connections for the planning.
        """

        self.group_id = self._annotations["group_id"]
        self.planner_id = self._annotations["planner_id"]
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(self.group_id)
        self._marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10, latch=True)
        self._planning_time_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult,
                                                   self._check_computation_time)
        rospy.sleep(1)

        self.group.set_planner_id(self.planner_id)
        self.group.set_num_planning_attempts(self._annotations["planning_attempts"])

        self.group.set_goal_tolerance(self._annotations["goal_tolerance"])
        self.group.set_planning_time(self._annotations["planning_time"])
        self.group.allow_replanning(self._annotations["allow_replanning"])

        self._comp_time = []

        self.planner_data = []

    def benchmark(self):
        marker_position_1 = self._annotations["start_xyz"]
        marker_position_2 = self._annotations["goal_xyz"]
        self._add_markers(marker_position_1, "Start test \n sequence", marker_position_2, "Goal")

        # Start planning in a given joint position
        joints = self._annotations["start_joints"]
        current = RobotState()
        current.joint_state.name = self.robot.get_current_state().joint_state.name
        current_joints = list(
            self.robot.get_current_state().joint_state.position)
        current_joints[0:6] = joints
        current.joint_state.position = current_joints

        self.group.set_start_state(current)
        joints = self._annotations["goal_joints"]
        self._plan_joints(joints, "Test 1")

        return self.planner_data

    def _add_markers(self, point, text1, point_2, text2):
        # add marker for start and goal pose of EE
        marker_array = MarkerArray()
        marker_1 = Marker()
        marker_1.header.frame_id = "world"
        marker_1.header.stamp = rospy.Time.now()
        marker_1.type = Marker.SPHERE
        marker_1.scale.x = 0.04
        marker_1.scale.y = 0.04
        marker_1.scale.z = 0.04
        marker_1.lifetime = rospy.Duration()

        marker_2 = deepcopy(marker_1)

        marker_1.color.g = 0.5
        marker_1.color.a = 1.0
        marker_1.id = 0
        marker_1.pose.position.x = point[0]
        marker_1.pose.position.y = point[1]
        marker_1.pose.position.z = point[2]
        marker_2.color.r = 0.5
        marker_2.color.a = 1.0
        marker_2.id = 1
        marker_2.pose.position.x = point_2[0]
        marker_2.pose.position.y = point_2[1]
        marker_2.pose.position.z = point_2[2]

        marker_3 = Marker()
        marker_3.header.frame_id = "world"
        marker_3.header.stamp = rospy.Time.now()
        marker_3.type = Marker.TEXT_VIEW_FACING
        marker_3.scale.z = 0.10
        marker_3.lifetime = rospy.Duration()

        marker_4 = deepcopy(marker_3)

        marker_3.text = text1
        marker_3.id = 2
        marker_3.color.g = 0.5
        marker_3.color.a = 1.0
        marker_3.pose.position.x = point[0]
        marker_3.pose.position.y = point[1]
        marker_3.pose.position.z = point[2] + 0.15
        marker_4.text = text2
        marker_4.id = 3
        marker_4.color.r = 0.5
        marker_4.color.a = 1.0
        marker_4.pose.position.x = point_2[0]
        marker_4.pose.position.y = point_2[1]
        marker_4.pose.position.z = point_2[2] + 0.15

        marker_array.markers = [marker_1, marker_2, marker_3, marker_4]

        self._marker_pub.publish(marker_array)
        rospy.sleep(1)

    def _plan_joints(self, joints, test_name):
        # plan to joint target and determine success
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0:6] = joints[0:6]
        self.group.set_joint_value_target(group_variable_values)

        plan = self.group.plan()
        plan_time = "N/A"
        total_joint_rotation = "N/A"
        comp_time = "N/A"

        plan_success = self._check_plan_success(plan)
        if plan_success:
            plan_time = self._check_plan_time(plan)
            total_joint_rotation = self._check_plan_total_rotation(plan)
            while not self._comp_time:
                rospy.sleep(0.5)
            comp_time = self._comp_time.pop(0)
        self.planner_data.append([self.planner_id, test_name, str(plan_success), plan_time, total_joint_rotation,
                                  comp_time])

    @staticmethod
    def _check_plan_success(plan):
        if len(plan.joint_trajectory.points) > 0:
            return True
        else:
            return False

    @staticmethod
    def _check_plan_time(plan):
        # find duration of successful plan
        number_of_points = len(plan.joint_trajectory.points)
        time = plan.joint_trajectory.points[number_of_points - 1].time_from_start.to_sec()
        return time

    @staticmethod
    def _check_plan_total_rotation(plan):
        # find total joint rotation in successful trajectory
        angles = [0, 0, 0, 0, 0, 0]
        number_of_points = len(plan.joint_trajectory.points)
        for i in range(number_of_points - 1):
            angles_temp = [abs(x - y) for x, y in zip(plan.joint_trajectory.points[i + 1].positions,
                                                      plan.joint_trajectory.points[i].positions)]
            angles = [x + y for x, y in zip(angles, angles_temp)]

        total_angle_change = sum(angles)
        return total_angle_change

    def _check_computation_time(self, msg):
        # get computation time for successful plan to be found
        if msg.status.status == 3:
            self._comp_time.append(msg.result.planning_time)

    def _check_path_length(self, plan):
        # find distance travelled by end effector
        # not yet implemented
        return


class PlannerBenchmarking(BenchmarkingBase):
    """
    Runs the benchmarking for the planners.
    """
    def __init__(self, path_to_data):
        super(PlannerBenchmarking, self).__init__(path_to_data)

        # iterate through all annotation files to run the benchmarks
        for annotation_file in self.load_files(path_to_data):
            parser = PlannerAnnotationParser(annotation_file, path_to_data)
            parser.to_xml(None)


if __name__ == '__main__':
    rospy.init_node("planner_benchmark")

    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        rospy.sleep(0.5)

    PlannerBenchmarking("/projects/data/planners_benchmarks")
