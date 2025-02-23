from typing import List
import re
from scipy.spatial.transform import Rotation
import dateutil.parser
import inspect
from pycram.datastructures.dataclasses import Pose


def atom(string: str):
    try:
        if re.match(".+:'.+'", string):
            # Has namespace prefix --> don't wrap in quotes
            return string
        return f"'{string}'"
    except:
        print(string)
        raise RuntimeError()

class PC:
    PINK = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    GREY = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Pose:
    def __init__(self, reference_frame: str, pos: List[float], ori: Rotation):
        self.reference_frame = reference_frame
        self.pos = pos
        self.ori = ori

    @staticmethod
    def from_prolog(prolog_pose: List):
        return Pose(reference_frame=prolog_pose[0], pos=prolog_pose[1], ori=Rotation.from_quat(prolog_pose[2]))

    def to_knowrob_string(self) -> str:
        """
        Convert to a KnowRob pose "[reference_cs, [x,y,z],[qx,qy,qz,qw]]"
        """
        quat = self.ori.as_quat()   # qxyzw
        return f"[{atom(self.reference_frame)}, [{self.pos[0]},{self.pos[1]},{self.pos[2]}], [{quat[0]}," \
               f"{quat[1]},{quat[2]},{quat[3]}]]"


class Datapoint:
    def __init__(self, timestamp: float, frame: str, reference_frame: str, pos: List[float], ori: Rotation,
                 wrench: List[float] = None):
        """
        :param timestamp:
        :param reference_frame: e.g. 'world'
        :param pos: [x,y,z] in m
        :param ori: [qx,qy,qz,qw] in a right-handed coordinate system
        :param wrench: [fx,fy,fz,mx,my,mz] in N / Nm
        """
        self.timestamp = timestamp
        self.frame = frame
        self.reference_frame = reference_frame
        self.pos = pos
        self.ori = ori
        self.wrench = wrench

    @staticmethod
    def from_prolog(prolog_dp: dict, frame=""):
        ori = Rotation.from_quat(prolog_dp["term"][2][2])
        return Datapoint(timestamp=prolog_dp["term"][1], frame=frame, reference_frame=prolog_dp["term"][2][0],
                              pos=prolog_dp["term"][2][1], ori=ori)

    def to_knowrob_string(self) -> str:
        """
        Convert to a KnowRob pose "[reference_cs, [x,y,z],[qx,qy,qz,qw]]"
        """
        quat = self.ori.as_quat()   # qxyzw
        return f"[{atom(self.reference_frame)}, [{self.pos[0]},{self.pos[1]},{self.pos[2]}], [{quat[0]}," \
               f"{quat[1]},{quat[2]},{quat[3]}]]"

    @staticmethod
    def from_tf(tf_msg: dict):
        timestamp = dateutil.parser.parse(tf_msg["header"]["stamp"]["$date"]).timestamp()
        frame = tf_msg["child_frame_id"]
        reference_frame = tf_msg["header"]["frame_id"]
        trans = tf_msg["transform"]["translation"]
        pos = [trans["x"], trans["y"], trans["z"]]
        rot = tf_msg["transform"]["rotation"]
        ori = Rotation.from_quat([rot["x"], rot["y"], rot["z"], rot["w"]])
        return Datapoint(timestamp, frame, reference_frame, pos, ori)

    @staticmethod
    def from_unreal(timestamp: float, frame: str, reference_frame: str, pos_cm: List[float], ori_lhs: List[float]):
        """
        See https://github.com/robcog-iai/UUtils/blob/master/Source/UConversions/Public/Conversions.h#L59-L74
        :param timestamp: In seconds
        :param frame:
        :param reference_frame:
        :param pos_cm: [x,y,z] in cm
        :param ori_lhs: [qx,qy,qz,qw] in a left-handed coordinate system
        :return:
        """
        # Convert cm to mm
        pos_m = [p / 100.0 for p in pos_cm]
        pos_rhs = [pos_m[1], pos_m[0], pos_m[2]]

        # Convert handedness of coordinate systems
        x, y, z, w = ori_lhs
        ori_rhs = Rotation.from_quat([-x, y, -z, w])
        return Datapoint(timestamp, frame, reference_frame, pos_rhs, ori_rhs)


# utility function for mapping of action designators
# returns a list of possible deisgnator names
def get_classes_from_file(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)
    # Filter out only the classes defined in the module
    classes = [member[0] for member in members if
               member[1].__module__ == module.__name__ and "performable" not in member[0].lower()]
    return classes


def autogenerate_class_name_to_class(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)
    # Create a dictionary mapping class names to class objects
    class_name_to_class = {member[0]: member[1] for member in members if member[1].__module__ == module.__name__}
    return class_name_to_class


import inspect


def get_classes_and_parameters_from_file(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)

    # Filter out only the classes defined in the module that do not have "performable" in their name
    classes = [member for member in members if
               member[1].__module__ == module.__name__ and "performable" not in member[0].lower()]

    class_info = []
    for class_name, class_obj in classes:
        # Get the __init__ method parameters
        init_method = class_obj.__init__
        params = inspect.signature(init_method).parameters

        param_info = []
        for param_name, param_obj in params.items():
            if param_name != 'self':
                # Get the docstring for each parameter
                param_doc = param_obj.annotation.__doc__ if param_obj.annotation else None
                param_info.append((param_name, param_doc))

        class_info.append((class_name, param_info))

    return class_info

def pose_to_string(pose):
    """Convert the given Pose to an array of strings for KnowRob"""
    return [pose.frame, pose.position_as_list(), pose.orientation_as_list()]