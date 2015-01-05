
from copy import deepcopy
import os, yaml
from rospy import logerr, loginfo
import rospkg, genpy
import moveit_msgs.msg
from sr_grasp.utils import mk_grasp

class Grasp(moveit_msgs.msg.Grasp):
    """
    Represents a single grasp, basically a warpper around moveit_msgs/Grasp.
    """
    def __init__(self):
        super(Grasp, self).__init__()

    @classmethod
    def from_msg(cls, msg):
        """
        Construct a shadow grasp object from moveit grasp object.
        """
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
        """
        Return plain moveit_msgs/Grasp version of self.
        """
        raise Exception("TODO - to_msg")

    @classmethod
    def from_yaml(self, y):
        grasp = Grasp()
        genpy.message.fill_message_args(grasp, y)
        return grasp

# Dummy inline store for now, will become - params, file, db etc...
_store = {}

class GraspStash(object):
    """
    Interface to the list of grasps stored in the system. Clients should all
    use this library so that it can deal with the detail of the undelying
    storage.
    """
    def __init__(self):
        pass

    def get_all(self):
        """
        Return list of all grasps.
        """
        return _store.values();

    def get_grasp(self, id):
        """
        Return a single grasp from the stash from it's id field.
        """
        Grasp()
        return Grasp;

    def get_grasp_at(self, idx):
        """Return the Grasp at the given index."""
        return self.get_all()[idx]

    def size(self):
        """Return the number of grasps."""
        return len(_store)

    def put_grasp(self, grasp):
        """
        Stash the given grasp, using it's id field, which must be set.
        """
        if grasp.id is None or grasp.id == "":
            raise Exception("Grasp has no id")
        # Up convert a plain grasp msg to our wrapper
        #if isinstance(grasp, moveit_msgs.msg.Grasp):
        #    grasp = Grasp.from_msg(grasp)
        _store[grasp.id] = grasp

    def load_all(self):
        """
        Load all configured sources of grasps into the stash.
        """
        rp = rospkg.RosPack()
        grasp_file = os.path.join(rp.get_path('sr_grasp'), 'resource', 'grasps.yaml')
        self.load_yaml_file(grasp_file)

    def load_yaml_file(self, fname):
        """
        Load a set of grasps from a YAML file.
        """
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
        """
        Load a set of grasps from a YAML object. Throws exceptions on errors.
        """
        for g in data:
            grasp = Grasp.from_yaml(g)
            self.put_grasp(grasp)

