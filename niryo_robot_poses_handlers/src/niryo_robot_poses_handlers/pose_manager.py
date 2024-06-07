#!/usr/bin/env python

from .file_manager import FileManager

from niryo_robot_utils.dataclasses.ABCSerializable import ABCSerializable
from niryo_robot_utils.dataclasses.Pose import Pose
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition


class PoseObj(ABCSerializable):

    def __init__(self, name: str, description: str, joints: JointsPosition, pose: Pose):
        self.name = name
        self.description = description
        self.joints = joints
        self.pose = pose

    def to_dict(self):
        return {
            'name': self.name,
            'description': self.description,
            'joints': self.joints.to_dict(),
            'pose': self.pose.to_dict()
        }

    @classmethod
    def from_dict(cls, d):
        if 'pose' not in d:
            d['joints'] = {f'joint_{n}': joint for n, joint in enumerate(d['joints'])}
            d['pose'] = {
                'x': d['position'][0],
                'y': d['position'][1],
                'z': d['position'][2],
                'roll': d['rpy'][0],
                'pitch': d['rpy'][1],
                'yaw': d['rpy'][2]
            }
        return cls(d['name'], d['description'], JointsPosition.from_dict(d['joints']), Pose.from_dict(d['pose']))


class PoseManager(FileManager):
    """
    Manages the creation, storage and loading of positions.

    :raises NiryoRobotFileException:
    """
    object_type = PoseObj

    def __init__(self, position_dir):
        super(PoseManager, self).__init__(position_dir, "position")

    def create(self, pose_obj):
        self._write(pose_obj.name, pose_obj)
