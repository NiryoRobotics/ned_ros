#!/usr/bin/env python

import rospy
import logging

from niryo_robot_utils import sentry_init

import rospkg

import yaml
import time
import os
import cv2
import numpy as np

from niryo_robot_vision.enums import ObjectType, ColorHSV
from niryo_robot_vision.image_functions import debug_threshold_color, debug_markers
from niryo_robot_vision.math_functions import euclidean_3d_dist, calculate_barycenter
from niryo_robot_vision.visualization_functions import *
from niryo_robot_vision.CalibrationObject import CalibrationObject
from ObjectDetector import ObjectDetector
from fonctions_camera import generate_msg_from_image
from VideoStream import WebcamStream, GazeboStream
from tf.transformations import quaternion_from_matrix

# Messages
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Point
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.msg import ObjectPose
from niryo_robot_poses_handlers.srv import GetWorkspaceMatrixPoses, GetWorkspacePoints, GetWorkspaceRatio

# Services
from niryo_robot_vision.srv import DebugColorDetection
from niryo_robot_vision.srv import DebugMarkers
from niryo_robot_vision.srv import ObjDetection
from niryo_robot_vision.srv import TakePicture
from niryo_robot_vision.srv import Visualization


class VisionNode:
    """
    Object which will contains all ROS Publishers & Services relate to image processing
    """

    def __init__(self):
        rospy.logdebug("Vision Node - Entering in Init")
        # -- ROS
        self.__path_package = rospkg.RosPack().get_path('niryo_robot_vision')
        self.__simulation_mode = rospy.get_param("~simulation_mode")
        self.__debug_compression_quality = rospy.get_param("~debug_compression_quality")

        rospy.logdebug("VisionNode.init - debug_compression_quality: {}".format(self.__debug_compression_quality))

        # PUBLISHERS
        self.__publisher_compressed_stream = rospy.Publisher('~compressed_video_stream', CompressedImage, queue_size=1)
        self.__publisher_rviz = rospy.Publisher('~visualization_marker', MarkerArray, queue_size=10, latch=True)

        # OBJECT DETECTION
        rospy.Service('~obj_detection_rel', ObjDetection, self.__callback_get_obj_relative_pose)

        # CALIBRATION
        if not self.__simulation_mode:
            self.__calibration_object = self.__generate_calib_object_from_setup()
        else:
            self.__calibration_object = self.__generate_calib_object_from_gazebo_topic()

        self.__camera_intrinsics_publisher = rospy.Publisher('~camera_intrinsics', CameraInfo, latch=True, queue_size=1)
        self.publish_camera_intrinsics()
        rospy.logdebug("Vision Node - Camera Intrinsics published !")

        # Debug features
        rospy.Service('~take_picture', TakePicture, self.__callback_take_picture)
        rospy.Service('~debug_markers', DebugMarkers, self.__callback_debug_markers)
        rospy.Service('~debug_colors', DebugColorDetection, self.__callback_debug_color)
        rospy.Service('~visualization', Visualization, self.__callback_visualization)

        self.__visualization_workspace = None
        self.__time_add_object = rospy.Time.now()

        # -- VIDEO STREAM
        rospy.logdebug("Vision Node - Creating Video Stream object")
        cls_ = GazeboStream if self.__simulation_mode else WebcamStream
        self.__video_stream = cls_(self.__calibration_object,
                                   self.__publisher_compressed_stream,
                                   rospy.get_param("~flip_img"))
        rospy.logdebug("Vision Node - Video Stream Created")

        self.__video_stream.start()

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Vision Node - Initialized")

    def __generate_calib_object_from_setup(self):
        calibration_object_name = rospy.get_param("~obj_calib_name")
        rospy.logdebug("VisionNode.init - obj_calib_name: {}".format(calibration_object_name))

        path_yaml = os.path.join(self.__path_package, "config/{}.yaml".format(calibration_object_name))
        if not os.path.isfile(path_yaml):
            rospy.logwarn("Vision Node - Intrinsics file '{}' does not exist".format(calibration_object_name))
            return CalibrationObject.set_empty()
        with open(path_yaml, "r") as input_file:
            yaml_file = yaml.safe_load(input_file)
        return CalibrationObject.set_from_yaml(yaml_file)

    @staticmethod
    def __generate_calib_object_from_gazebo_topic():
        camera_info_message = rospy.wait_for_message("/gazebo_camera/camera_info", CameraInfo)
        mtx = np.reshape(camera_info_message.K, (3, 3))
        dist = np.expand_dims(camera_info_message.D, axis=0)
        return CalibrationObject.set_from_values(mtx, dist)

    def publish_camera_intrinsics(self):
        mtx, dist = self.__calibration_object.get_camera_info()

        msg_camera_info = CameraInfo()
        msg_camera_info.K = list(mtx.flatten())
        msg_camera_info.D = list(dist.flatten())
        return self.__camera_intrinsics_publisher.publish(msg_camera_info)

    # - CALLBACK
    def __callback_get_obj_relative_pose(self, req):
        # Reading last image
        img = self.__video_stream.read_undistorted()
        if img is None:
            rospy.logwarn("Vision Node - Try to get object relative pose while stream is not running !")
            return CommandStatus.VIDEO_STREAM_NOT_RUNNING, ObjectPose(), "", "", CompressedImage()

        # Extracting parameters from request
        obj_type = ObjectType[req.obj_type]
        obj_color = ColorHSV[req.obj_color]
        workspace_ratio = req.workspace_ratio
        self.__object_detector = ObjectDetector(
            obj_type=obj_type,
            obj_color=obj_color,
            workspace_ratio=workspace_ratio,
            ret_image_bool=req.ret_image,
        )

        # Launching pipeline
        status, msg_res_pos, obj_type, obj_color, im_draw = self.__object_detector.extract_object_with_hsv(img)
        if self.__object_detector.should_ret_image():
            _, msg_img = generate_msg_from_image(im_draw, compression_quality=self.__debug_compression_quality)
        else:
            msg_img = CompressedImage()
        return status, msg_res_pos, obj_type, obj_color, msg_img

    def __callback_take_picture(self, req):
        path = os.path.expanduser(req.path)
        if not os.path.isdir(path):
            os.makedirs(path)
        time_string = time.strftime("%Y%m%d-%H%M%S")

        img_full_path = "{}.jpg".format(os.path.join(path, time_string))
        im = self.__video_stream.read_raw_img()
        res_bool = cv2.imwrite(img_full_path, im)

        if res_bool:
            rospy.logdebug("Vision Node - Picture taken & saved")
        else:
            rospy.logwarn("Vision Node - Cannot save picture")

        return res_bool

    def __callback_debug_markers(self, _):
        img = self.__video_stream.read_undistorted()
        if img is None:
            message = 'Try to get debug markers while stream is not running !'
            rospy.logwarn_throttle(2.0, f'Vision Node - {message}')
            return CommandStatus.VIDEO_STREAM_NOT_RUNNING, message, False, CompressedImage()

        markers_detected, img_res = debug_markers(img)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return CommandStatus.SUCCESS, 'Successfully got debug markers', markers_detected, msg_img

    def __callback_debug_color(self, req):
        img = self.__video_stream.read_undistorted()
        if img is None:
            message = 'Try to get debug colors while stream is not running !'
            rospy.logwarn_throttle(2.0, f'Vision Node - {message}')
            return CommandStatus.VIDEO_STREAM_NOT_RUNNING, message, CompressedImage()
        color = ColorHSV[req.color]
        img_res = debug_threshold_color(img, color)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return CommandStatus.SUCCESS, 'Successfully got debug colors', msg_img

    def __callback_visualization(self, req):
        if req.clearing:
            marker = create_clear_all_marker(rospy.Time.now())
            self.__publisher_rviz.publish([marker])
            return CommandStatus.SUCCESS, 'Successfully cleared markers'
        else:
            # Get workspace matrix
            rospy.wait_for_service('/niryo_robot_poses_handlers/get_workspace_matrix_poses')
            try:
                get_matrix = rospy.ServiceProxy('/niryo_robot_poses_handlers/get_workspace_matrix_poses',
                                                GetWorkspaceMatrixPoses)
                response = get_matrix(req.workspace)

                position_matrix = np.empty(shape=[0, 3])
                for elem in response.matrix_position:
                    position_matrix = np.append(position_matrix, [[elem.x, elem.y, elem.z]], axis=0)
                rotation_matrix = np.empty(shape=[0, 4])
                for elem in response.matrix_orientation:
                    rotation_matrix = np.append(rotation_matrix, [[elem.x, elem.y, elem.z, elem.w]], axis=0)
                workspace = {
                    "name": req.workspace, "matrix_position": position_matrix, "matrix_rotation": rotation_matrix
                }
            except rospy.ServiceException as e:
                workspace = None
                rospy.logerr(f'Vision Node - get workspace matrix failed: {e}')

            # Get workspace ratio
            rospy.wait_for_service('/niryo_robot_poses_handlers/get_workspace_ratio')
            try:
                get_ratio = rospy.ServiceProxy('/niryo_robot_poses_handlers/get_workspace_ratio', GetWorkspaceRatio)
                response = get_ratio(req.workspace)

                workspace["ratio"] = response.ratio
            except rospy.ServiceException as e:
                workspace = None
                rospy.logerr(f'Vision Node - get workspace ratio failed: {e}')

            # Publish marker's workspace
            rospy.wait_for_service('/niryo_robot_poses_handlers/get_workspace_points')
            try:
                get_points = rospy.ServiceProxy('/niryo_robot_poses_handlers/get_workspace_points', GetWorkspacePoints)
                response = get_points(req.workspace)

                index = 1
                time_workspace = rospy.Time.now()
                list_marker = []
                for point in response.points:
                    list_marker.append(
                        create_marker_rviz([point.x, point.y, point.z], [0, 0, 0, 1], index, time_workspace))
                    list_marker.append(
                        create_text_marker(4 + index,
                                           str(index),
                                           time_workspace, [point.x, point.y, point.z], [0, 0, 0, 1],
                                           0.035))
                    index += 1

                # Title workspace
                middle_text = Point((response.points[0].x + response.points[1].x) / 2,
                                    (response.points[0].y + response.points[1].y) / 2,
                                    (response.points[0].z + response.points[1].z) / 2)

                list_marker.append(
                    create_text_marker(9,
                                       str(workspace["name"]),
                                       time_workspace, [middle_text.x, middle_text.y, middle_text.z], [0, 0, 0, 1],
                                       0.035))

                self.__publisher_rviz.publish(list_marker)

                # Publish workspace plane
                bary_center = calculate_barycenter(response.points)
                sizeX = euclidean_3d_dist(response.points[0], response.points[1]) + 0.03
                sizeY = euclidean_3d_dist(response.points[1], response.points[2]) + 0.03

                quaternion = quaternion_from_matrix(workspace["matrix_rotation"])
                marker = create_workspace(10,
                                          time_workspace, [bary_center.x, bary_center.y, bary_center.z],
                                          quaternion,
                                          sizeX,
                                          sizeY,
                                          0.0005)
                self.__publisher_rviz.publish([marker])

            except rospy.ServiceException as e:
                rospy.logerr(f'Vision Node - Publishing marker\'s workspace failed: {e}')

            # Reading last image
            img = self.__video_stream.read_undistorted()
            if img is None:
                message = 'Try to get object relative pose while stream is not running !'
                rospy.logwarn(f'Vision Node - {message}')
                return CommandStatus.VIDEO_STREAM_NOT_RUNNING, message

            # Detect  object
            index = 11
            index_max = 50
            obj_type = ObjectType["ANY"]
            obj_color = ColorHSV["ANY"]
            workspace_ratio = workspace["ratio"]
            self.__object_detector = ObjectDetector(
                obj_type=obj_type,
                obj_color=obj_color,
                workspace_ratio=workspace_ratio,
                ret_image_bool=False,
            )
            # Launching pipeline
            status, cx_list, cy_list, list_size, angle_list, color_list, \
                object_type_list = self.__object_detector.extract_all_object_with_hsv(img, workspace)

            time_object = rospy.Time.now()
            if workspace and status == CommandStatus.SUCCESS:
                self.__time_add_object = time_object
                for cx, cy, size, angle, obj_color, obj_type in zip(cx_list, cy_list, list_size, angle_list, color_list,
                                                                    object_type_list):
                    position, orientation = get_pose(workspace, cx, cy, angle)
                    publish_object_rviz(position,
                                        orientation,
                                        size,
                                        obj_type,
                                        obj_color,
                                        index,
                                        self.__publisher_rviz,
                                        time_object,
                                        rospy.Duration.from_sec(3))

                    if index < index_max:
                        index += 1

                # Clearing
                list_clear_marker = []
                for i in range(index, index_max + 1):
                    list_clear_marker.append(create_clear_marker(i, time_object))
                self.__publisher_rviz.publish(list_clear_marker)

            else:
                if self.__time_add_object + rospy.Duration(3) < rospy.Time.now():
                    # Reset
                    list_clear_marker = []
                    for i in range(index, index_max + 1):
                        list_clear_marker.append(create_clear_marker(i, time_object))
                    self.__publisher_rviz.publish(list_clear_marker)

        return status, ''


if __name__ == '__main__':
    sentry_init()

    # we need to layun
    rospy.init_node('niryo_robot_vision', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        vision_node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
