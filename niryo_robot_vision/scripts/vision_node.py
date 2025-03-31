#!/usr/bin/env python

import logging
import os
import time

import cv2
import rospy

from niryo_robot_utils import sentry_init

# Services
from niryo_robot_vision.srv import DebugColorDetection, DebugMarkers, ObjDetection, TakePicture
from sensor_msgs.msg import CameraInfo, CompressedImage

# Messages
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.msg import ObjectPose

from niryo_robot_vision.CalibrationObject import CalibrationObject
from niryo_robot_vision.enums import ColorHSV
from niryo_robot_vision.fonctions_camera import generate_msg_from_image
from niryo_robot_vision.image_functions import debug_threshold_color, debug_markers
from niryo_robot_vision.ObjectDetector import ObjectDetector
from niryo_robot_vision.visualization_functions import ObjectType

from VideoStream import WebcamStream, GazeboStream


class VisionNode:
    """
    Object which will contains all ROS Publishers & Services relate to image processing
    """

    def __init__(self):
        rospy.logdebug("Vision Node - Entering in Init")
        # -- ROS
        self.__simulation_mode = rospy.get_param("~simulation_mode")
        self.__debug_compression_quality = rospy.get_param("~debug_compression_quality")

        rospy.logdebug("VisionNode.init - debug_compression_quality: {}".format(self.__debug_compression_quality))

        # PUBLISHERS
        self.__publisher_compressed_stream = rospy.Publisher('~compressed_video_stream', CompressedImage, queue_size=1)

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

    @staticmethod
    def __generate_calib_object_from_setup():
        calibration_object_name = rospy.get_param("~obj_calib_name")
        rospy.logdebug("VisionNode.init - obj_calib_name: {}".format(calibration_object_name))

        calibration_object_params = rospy.get_param(f'~{calibration_object_name}', None)
        if calibration_object_params is None:
            rospy.logwarn("Vision Node - No calibration object found in parameter server")
            return CalibrationObject.set_empty()

        return CalibrationObject.set_from_dict(calibration_object_params)

    @staticmethod
    def __generate_calib_object_from_gazebo_topic():
        camera_info_message = rospy.wait_for_message("/gazebo_camera/camera_info", CameraInfo)
        return CalibrationObject.set_from_raw(camera_info_message.K, camera_info_message.D)

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
