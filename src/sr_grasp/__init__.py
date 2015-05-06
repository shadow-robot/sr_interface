
from copy import deepcopy
import os, yaml, string
from rospy import logerr, loginfo, get_param
import rospkg, genpy
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from sr_robot_msgs.msg import GraspArray


def reindent(s, numSpaces):
    """
    http://code.activestate.com/recipes/66055-changing-the-indentation-of-a-multi-line-string/
    """
    s = string.split(s, '\n')
    s = [(numSpaces * ' ') + line for line in s]
    s = string.join(s, '\n')
    return s


class Grasp(moveit_msgs.msg.Grasp):
    """
    Represents a single grasp, basically a wrapper around moveit_msgs/Grasp
    with added functions and Shadow Hand specific knowledge.
    """
    def __init__(self):
        moveit_msgs.msg.Grasp.__init__(self)
        self.grasp_quality = 0.001
        self.joint_names = [
            'FFJ1', 'FFJ2', 'FFJ3', 'FFJ4',
            'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5',
            'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4',
            'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4',
            'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5',
            'WRJ1', 'WRJ2']

        # Default the pre grasp to all 0.0
        zero_joints = dict.fromkeys(self.joint_names, 0.0)
        self.set_pre_grasp_point(zero_joints)

    @classmethod
    def from_msg(cls, msg):
        """Construct a shadow grasp object from moveit grasp object."""
        grasp = Grasp()
        grasp.id = msg.id
        grasp.pre_grasp_posture  = deepcopy(msg.pre_grasp_posture)
        grasp.grasp_posture      = deepcopy(msg.grasp_posture)
        grasp.grasp_pose         = deepcopy(msg.grasp_pose)
        grasp.grasp_quality      = msg.grasp_quality
        grasp.pre_grasp_approach = deepcopy(msg.pre_grasp_approach)
        grasp.post_grasp_retreat = deepcopy(msg.post_grasp_retreat)
        grasp.post_place_retreat = deepcopy(msg.post_place_retreat)
        grasp.max_contact_force  = msg.max_contact_force
        grasp.allowed_touch_objects = deepcopy(msg.allowed_touch_objects)
        return grasp

    def to_msg(self):
        """Return plain moveit_msgs/Grasp version of self."""
        raise Exception("TODO - to_msg")

    @classmethod
    def from_yaml(self, y):
        """
        Construct a shadow grasp object from YAML object. For example YAML
        grabbed from rostopic to a file.
        """
        grasp = Grasp()
        genpy.message.fill_message_args(grasp, y)
        return grasp

    def set_pre_grasp_point(self, *args, **kwargs):
        """
        Set the positions for a point (default 0) in the pre-grasp to a dict of
        joint positions.
        """
        self._set_posture_point(self.pre_grasp_posture, *args, **kwargs)

    def set_grasp_point(self, *args, **kwargs):
        """
        Set the positions for a point (default 0) in the grasp to a dict of
        joint positions.
        """
        self._set_posture_point(self.grasp_posture, *args, **kwargs)

    def _set_posture_point(self, posture, positions, point=0):
        """Set the posture positions using a dict of joint positions."""
        # XXX: Why have we been doing this?
        #posture.header.stamp = now
        posture.joint_names = positions.keys()

        # Extend the array to be big enough.
        if len(posture.points) < point+1:
            for _ in range(point+1):
                posture.points.append(JointTrajectoryPoint())

        # Update the point in place
        jtp = JointTrajectoryPoint()
        for _, pos in positions.iteritems():
            jtp.positions.append(pos)
        posture.points[point] = jtp




class GraspStash(object):
    """
    Interface to the list of grasps stored in the system. Clients should all
    use this library so that it can deal with the detail of the undelying
    storage.
    """
    def __init__(self):
        # Store of all loaded grasps, indexed on grasp.id.
        self._store = {}
        rp = rospkg.RosPack()
        self.grasps_file = get_param('~grasps_file',
                                     default = os.path.join(
                                         rp.get_path('sr_grasp'), 'resource', 'grasps.yaml') )

    def get_all(self):
        """Return list of all grasps."""
        return self._store.values();

    def get_grasp_array(self):
        arr = GraspArray()
        arr.grasps = self.get_all()
        return arr

    def get_grasp(self, grasp_index):
        """Return a single grasp from the stash from it's id field."""
        return self._store.get(grasp_index)
        
    def get_grasp_at(self, idx):
        """Return the Grasp at the given index."""
        return self.get_all()[idx]

    def size(self):
        """Return the number of grasps."""
        return len(self._store)

    def put_grasp(self, grasp):
        """Stash the given grasp, using it's id field, which must be set."""
        if grasp.id is None or grasp.id == "":
            raise Exception("Grasp has no id")
        # Up convert a plain grasp msg to our wrapper
        #if isinstance(grasp, moveit_msgs.msg.Grasp):
        #    grasp = Grasp.from_msg(grasp)
        self._store[grasp.id] = grasp

    def load_all(self):
        """Load all configured sources of grasps into the stash."""
        self.load_yaml_file(self.grasps_file)

    def as_yaml(self):
        return genpy.message.strify_message(self.get_grasp_array().grasps)

    def load_yaml_file(self, fname):
        """Load a set of grasps from a YAML file."""
        try:
            data = yaml.load(file(fname))
            self.load_yaml(data)
        except Exception as e:
            logerr("Failed to load YAML grasp file: %s error:%s"%(fname, e))
            return False
        else:
            loginfo("Loaded grasps from file: %s"%(fname))
            return True

    def load_yaml(self, data):
        """Load a set of grasps from a YAML object. Throws exceptions on errors."""
        for g in data:
            grasp = Grasp.from_yaml(g)
            self.put_grasp(grasp)

    def save_yaml_file(self, fname=""):
        if fname == "":
            fname = self.grasps_file
        with open(fname, "w") as txtfile:
            txtfile.write(self.as_yaml())

