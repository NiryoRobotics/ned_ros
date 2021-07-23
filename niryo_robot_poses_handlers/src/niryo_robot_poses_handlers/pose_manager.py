#!/usr/bin/env python

from niryo_robot_poses_handlers.file_manager import FileManager


class PoseObj:

    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.joints = None
        self.position = None
        self.rpy = None
        self.orientation = None

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["description"] = self.description
        dict_["joints"] = self.joints
        dict_["position"] = self.position
        dict_["rpy"] = self.rpy
        dict_["orientation"] = self.orientation
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        pos = cls(dict_["name"], dict_["description"])
        pos.joints = dict_["joints"]
        pos.position = dict_["position"]
        pos.rpy = dict_["rpy"]
        pos.orientation = dict_["orientation"]
        return pos

    def get_value(self):
        return [self.joints, self.position, self.rpy, self.orientation]

    def __str__(self):
        return "{} {} {}".format(self.name, self.position, self.rpy)


class PoseManager(FileManager):
    """
    Manages the creation, storage and loading of positions.

    :raises NiryoRobotFileException:
    """
    object_type = PoseObj

    def __init__(self, position_dir):
        super(PoseManager, self).__init__(position_dir, "position")

    def create(self, pos_name, joints, position, rpy, orientation, description=""):
        position_obj = PoseObj(pos_name, description)
        position_obj.joints = joints
        position_obj.rpy = rpy
        position_obj.position = position
        position_obj.orientation = orientation
        self._write(pos_name, position_obj)
