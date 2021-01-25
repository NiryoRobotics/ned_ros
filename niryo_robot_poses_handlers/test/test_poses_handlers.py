#!/usr/bin/env python

import rospy
import rostest

from niryo_robot_poses_handlers.test_pure_python_poses_handlers import *

from geometry_msgs.msg import Pose, Point, Quaternion
from niryo_robot_msgs.msg import RPY, RobotState

from niryo_robot_msgs.srv import GetNameDescriptionList

from niryo_robot_poses_handlers.srv import GetPose
from niryo_robot_poses_handlers.srv import ManagePose, ManagePoseRequest
from niryo_robot_poses_handlers.srv import GetTrajectory
from niryo_robot_poses_handlers.srv import ManageTrajectory, ManageTrajectoryRequest
from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses
from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

service_timeout = 3
position_dir = trajectory_dir = workspace_dir = None


def call_service(service_name, service_msg_type, *args):
    # Connect to service
    rospy.wait_for_service(service_name, service_timeout)
    # Call service
    service = rospy.ServiceProxy(service_name, service_msg_type)
    response = service(*args)
    return response


def clean_folders():
    folders_list_raw = [position_dir, trajectory_dir, workspace_dir]
    folders_list = [os.path.expanduser(folder) for folder in folders_list_raw]
    for folder in folders_list:
        if os.path.isdir(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)


class TestPoseHandlerAbstract(unittest.TestCase):
    file_dir = None

    def setUp(self):
        clean_folders()

    def tearDown(self):
        clean_folders()

    def create_folder(self):
        if not os.path.isdir(self.file_dir):
            os.makedirs(self.file_dir)

    def delete_folder(self):
        if os.path.isdir(self.file_dir):
            shutil.rmtree(self.file_dir)

    def assertStatus(self, ret):
        self.assertIsNotNone(ret, msg="Assert Status cannot operate on None ret")
        self.assertTrue(ret.status > 0, msg="status : {} - message : {}".format(ret.status, ret.message))

    def assertNotStatus(self, ret):
        self.assertIsNotNone(ret, msg="AssertNotStatus cannot operate on None ret")
        self.assertTrue(ret.status < 0, msg="status : {} - message : {}".format(ret.status, ret.message))


class TestServicePose(TestPoseHandlerAbstract):
    file_dir = position_dir

    # def setUp(self):
    #     super(TestServicePostion, self).setUp()
    #
    # def tearDown(self):
    #     super(TestServicePostion, self).tearDown()

    @staticmethod
    def get_pose_list():
        return call_service('/niryo_robot_poses_handlers/get_pose_list', GetNameDescriptionList).name_list

    @staticmethod
    def get_pose(name):
        return call_service('/niryo_robot_poses_handlers/get_pose', GetPose, name)

    @staticmethod
    def save_pose(name, x, y, z, roll, pitch, yaw):
        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.SAVE
        req.pose.name = name
        req.pose.position = Point(x, y, z)
        req.pose.rpy = RPY(roll, pitch, yaw)
        return call_service('/niryo_robot_poses_handlers/manage_pose', ManagePose, req)

    @staticmethod
    def delete_pose(name):
        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.DELETE
        req.pose.name = name
        return call_service('/niryo_robot_poses_handlers/manage_pose', ManagePose, req)

    def test_service_get_saved_pose_list(self):
        self.assertEqual(self.get_pose_list(), [])
        self.assertNotStatus(self.get_pose("SomePose"))  # Do not exist
        self.assertNotStatus(self.delete_pose("SomePose"))  # Do not exist

    def test_service_creation_delete_pos(self):
        self.assertEqual(self.get_pose_list(), [])
        list_names = []
        pose = [1, 2, 3, 1, 2, 3]
        for i in range(5):
            name = "Test{}".format(i)
            self.assertStatus(self.save_pose(name, *pose))
            ret_get_pos = self.get_pose(name)
            self.assertStatus(ret_get_pos)
            p = ret_get_pos.pose

            self.assertEqual([p.position.x, p.position.y, p.position.z, p.rpy.roll, p.rpy.pitch, p.rpy.yaw], pose,
                             "Read Failed")

            list_names.append(name)
            self.assertEqual(self.get_pose_list(), list_names, "Read All Failed")
        for name in list_names:
            self.assertStatus(self.delete_pose(name))
        self.assertEqual(self.get_pose_list(), [], "Read All Failed")


class TestServiceTrajetory(TestPoseHandlerAbstract):
    file_dir = trajectory_dir

    @staticmethod
    def get_trajectory_list():
        return call_service('/niryo_robot_poses_handlers/get_trajectory_list', GetNameDescriptionList).name_list

    @staticmethod
    def get_trajectory(name):
        ret = call_service('/niryo_robot_poses_handlers/get_trajectory', GetTrajectory, name)
        list_p = ret.list_poses
        list_p_raw = []
        for p in list_p:
            pose = [[p.position.x, p.position.y, p.position.z],
                    [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]]
            list_p_raw.append(pose)
        return ret, list_p_raw

    @staticmethod
    def save_trajectory(name, list_poses_raw):
        req = ManageTrajectoryRequest()
        req.name = name
        req.cmd = ManageTrajectoryRequest.SAVE
        list_poses = []
        for p in list_poses_raw:
            point = Point(*p[0])
            orientation = Quaternion(*p[1])
            list_poses.append(Pose(point, orientation))
        req.poses = list_poses
        return call_service('/niryo_robot_poses_handlers/manage_trajectory', ManageTrajectory, req)

    @staticmethod
    def delete_trajectory(name):
        req = ManageTrajectoryRequest()
        req.cmd = ManageTrajectoryRequest.DELETE
        req.name = name
        return call_service('/niryo_robot_poses_handlers/manage_trajectory', ManageTrajectory, req)

    def test_service_get_saved_traj_list(self):
        self.assertEqual(self.get_trajectory_list(), [])
        self.assertNotStatus(self.get_trajectory("Some Traj")[0])  # Do not exist
        self.assertNotStatus(self.delete_trajectory("Some Traj"))  # Do not exist

    def test_service_creation_traj_pos(self):
        self.assertEqual(self.get_trajectory_list(), [])
        list_names = []
        poses = [[[1, 2, 3], [1, 0, 0, 0]],
                 [[1, 2, 3], [1, 0, 0, 0]]]
        for i in range(5):
            name = "Test{}".format(i)
            self.assertStatus(self.save_trajectory(name, poses))
            ret_get_pos = self.get_trajectory(name)
            self.assertStatus(ret_get_pos[0])

            self.assertEqual(ret_get_pos[1], poses)

            list_names.append(name)
            self.assertEqual(self.get_trajectory_list(), list_names, "Read All Failed")
        for name in list_names:
            self.assertStatus(self.delete_trajectory(name))
        self.assertEqual(self.get_trajectory_list(), [], "Read All Failed")


class TestServiceWorkspace(TestPoseHandlerAbstract):
    file_dir = workspace_dir

    @staticmethod
    def get_workspace_list():
        return call_service('/niryo_robot_poses_handlers/get_workspace_list', GetNameDescriptionList).name_list

    @staticmethod
    def get_workspace_poses(name):
        ret = call_service('/niryo_robot_poses_handlers/get_workspace_poses', GetWorkspaceRobotPoses, name)
        poses = ret.poses
        list_p_raw = []
        for p in poses:
            pose = [[p.position.x, p.position.y, p.position.z],
                    [p.rpy.roll, p.rpy.pitch, p.rpy.yaw]]
            list_p_raw.append(pose)
        return ret, list_p_raw

    @staticmethod
    def save_workspace(name, list_poses_raw):
        list_poses = []
        for p in list_poses_raw:
            point = Point(*p[0])
            rpy = RPY(*p[1])
            list_poses.append(RobotState(point, rpy, Quaternion()))
        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.SAVE
        req.workspace.name = name
        req.workspace.poses = list_poses
        return call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)

    @staticmethod
    def save_workspace_from_points(name, list_points_raw):
        list_points = []
        for p in list_points_raw:
            list_points.append(Point(*p))
        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.SAVE_WITH_POINTS
        req.workspace.name = name
        req.workspace.points = list_points
        return call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)

    @staticmethod
    def delete_workspace(name):
        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.DELETE
        req.workspace.name = name
        return call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)

    def test_service_get_saved_workspace_list(self):
        self.assertEqual(self.get_workspace_list(), [])
        self.assertNotStatus(self.get_workspace_poses("Some Workspace")[0])  # Do not exist
        self.assertNotStatus(self.delete_workspace("Some Workspace"))  # Do not exist

    def test_service_creation_delete_workspace(self):
        self.assertEqual(self.get_workspace_list(), [])
        list_names = []
        # robot_poses can be found in test_pure_python_pose_handlers
        for i in range(3):
            name = "Test{}".format(2 * i)
            self.assertStatus(self.save_workspace(name, robot_poses))
            ret_get_pos = self.get_workspace_poses(name)
            self.assertStatus(ret_get_pos[0])

            self.assertEqual(ret_get_pos[1], robot_poses)
            name2 = "Test{}".format(2 * i + 1)
            self.assertStatus(self.save_workspace_from_points(name2, points))

            list_names.append(name)
            list_names.append(name2)

            self.assertEqual(self.get_workspace_list(), list_names)
        for name in list_names:
            self.assertStatus(self.delete_workspace(name))
        self.assertEqual(self.get_workspace_list(), [])


if __name__ == '__main__':
    while not rospy.has_param('/niryo_robot_poses_handlers/initialized'):
        rospy.sleep(0.10)

    position_dir = os.path.expanduser(rospy.get_param('/niryo_robot_poses_handlers/poses_dir'))
    trajectory_dir = os.path.expanduser(rospy.get_param('/niryo_robot_poses_handlers/trajectories_dir'))
    workspace_dir = os.path.expanduser(rospy.get_param('/niryo_robot_poses_handlers/workspace_dir'))
    # Going to execute all unittest.TestCase subclasses in the file -> Import are also concerned
    rostest.rosrun("poses_handlers", "test_poses_handlers", __name__)
