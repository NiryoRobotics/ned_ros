#!/usr/bin/env python

from niryo_robot_poses_handlers.file_manager import FileManager


class Trajectory:
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.list_poses = None

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["description"] = self.description
        dict_["list_poses"] = []
        for pose in self.list_poses:
            sub_dict = dict()
            sub_dict["position"] = pose[0]
            sub_dict["orientation"] = pose[1]
            dict_["list_poses"].append(sub_dict)
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        traj = cls(dict_["name"], dict_["description"])
        traj.list_poses = []
        for pose in dict_["list_poses"]:
            point = pose["position"]
            quaternion = pose["orientation"]
            traj.list_poses.append([point, quaternion])
        return traj

    def get_value(self):
        return self.list_poses


class TrajectoryManager(FileManager):
    """
    Manages the creation, storage and loading of trajectories.

    :raises NiryoRobotFileException:
    """
    object_type = Trajectory

    def __init__(self, trajectory_dir):
        super(TrajectoryManager, self).__init__(trajectory_dir, "trajectory")

    def create(self, trajectory_name, lists_poses, description=""):
        trajectory = Trajectory(trajectory_name, description)
        trajectory.list_poses = lists_poses
        self._write(trajectory_name, trajectory)
