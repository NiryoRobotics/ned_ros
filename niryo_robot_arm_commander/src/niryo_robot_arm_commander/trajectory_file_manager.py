#!/usr/bin/env python

from niryo_robot_poses_handlers.file_manager import PickleFileManager


class Trajectory:
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.list_poses = None

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["description"] = self.description
        dict_["list_poses"] = [trajectory_point.positions for trajectory_point in self.list_poses]
        # for trajectory_point in self.list_poses:
        #     sub_dict = dict()
        #     sub_dict["positions"] = trajectory_point.positions
        #     sub_dict["velocities"] = trajectory_point.velocities
        #     sub_dict["accelerations"] = trajectory_point.accelerations
        #     sub_dict["effort"] = trajectory_point.effort
        #     sub_dict["time_from_start_secs"] = trajectory_point.time_from_start.secs
        #     sub_dict["time_from_start_nsecs"] = trajectory_point.time_from_start.nsecs
        #     dict_["list_poses"].append(sub_dict)
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        try:
            traj = cls(dict_["name"], dict_["description"])
            traj.list_poses = [trajectory_point for trajectory_point in dict_["list_poses"]]
            # for trajectory_point in dict_["list_poses"]:
            # positions = trajectory_point["positions"]
            # velocities = trajectory_point["velocities"]
            # accelerations = trajectory_point["accelerations"]
            # effort = trajectory_point["effort"]
            # time_from_start_secs = trajectory_point["time_from_start_secs"]
            # time_from_start_nsecs = trajectory_point["time_from_start_nsecs"]
            # traj.list_poses.append([positions, velocities, accelerations, effort,
            #  [time_from_start_secs, time_from_start_nsecs]])
            return traj
        except Exception:
            return None

    def get_value(self):
        return self.list_poses


class TrajectoryFileManager(PickleFileManager):
    """
    Manages the creation, storage and loading of trajectories.

    :raises NiryoRobotFileException:
    """
    object_type = Trajectory

    def __init__(self, trajectory_dir):
        super(TrajectoryFileManager, self).__init__(
            trajectory_dir, "trajectory")

    def create(self, trajectory_name, lists_poses, description=""):
        trajectory = Trajectory(trajectory_name, description)
        trajectory.list_poses = lists_poses
        self._write(trajectory_name, trajectory)
