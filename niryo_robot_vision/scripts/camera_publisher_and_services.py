#!/usr/bin/env python

import rospy
import rospkg

import yaml
import time
import os
import cv2
import numpy as np

from niryo_robot_vision.enums import ObjectType, ColorHSV
from niryo_robot_vision.image_functions import debug_threshold_color, debug_markers
from niryo_robot_vision.CalibrationObject import CalibrationObject
from ObjectDetector import ObjectDetector
from fonctions_camera import generate_msg_from_image
from VideoStream import WebcamStream, GazeboStream

# Messages
from sensor_msgs.msg import CameraInfo, CompressedImage
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.msg import ObjectPose

# Services
from niryo_robot_vision.srv import DebugColorDetection
from niryo_robot_vision.srv import DebugMarkers
from niryo_robot_vision.srv import ObjDetection
from niryo_robot_vision.srv import TakePicture


class VisionNode:
    """
    Object which will contains all ROS Publishers & Services relate to image processing
    """

    def __init__(self):
        rospy.logdebug("Vision Node - Entering in Init")
        # -- ROS
        self.__path_package = rospkg.RosPack().get_path('niryo_robot_vision')
        self.__simulation_mode = rospy.get_param('~simulation_mode')

        # PUBLISHERS
        self.__publisher_compressed_stream = rospy.Publisher('~compressed_video_stream',
                                                             CompressedImage, queue_size=1)

        # OBJECT DETECTION
        rospy.Service('~obj_detection_rel', ObjDetection,
                      self.__callback_get_obj_relative_pose)

        # CALIBRATION
        if not self.__simulation_mode:
            self.__calibration_object = self.__generate_calib_object_from_setup()
        else:
            self.__calibration_object = self.__generate_calib_object_from_gazebo_topic()

        self.__camera_intrinsics_publisher = rospy.Publisher(
            '~camera_intrinsics', CameraInfo, latch=True, queue_size=1)
        self.publish_camera_intrinsics()
        rospy.logdebug("Vision Node - Camera Intrinsics published !")

        # Debug features
        rospy.Service('~take_picture', TakePicture,
                      self.__callback_take_picture)

        self.__debug_compression_quality = rospy.get_param("~debug_compression_quality")
        rospy.Service('~debug_markers', DebugMarkers,
                      self.__callback_debug_markers)
        rospy.Service('~debug_colors', DebugColorDetection,
                      self.__callback_debug_color)

        # -- VIDEO STREAM
        rospy.logdebug("Vision Node - Creating Video Stream object")
        cls_ = GazeboStream if self.__simulation_mode else WebcamStream
        self.__video_stream = cls_(self.__calibration_object, self.__publisher_compressed_stream)
        rospy.logdebug("Vision Node - Video Stream Created")

        self.__video_stream.start()

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Vision Node - Initialized")

    def __generate_calib_object_from_setup(self):
        calibration_object_name = rospy.get_param("~obj_calib_name")

        path_yaml = os.path.join(self.__path_package, "config/{}.yaml".format(calibration_object_name))
        if not os.path.isfile(path_yaml):
            rospy.logwarn("Vision Node - Intrinsics file '{}' does not exist".format(calibration_object_name))
            return CalibrationObject.set_empty()
        with open(path_yaml, "r") as input_file:
            yaml_file = yaml.load(input_file)
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
        ret_image = req.ret_image

        # Creating ObjectDetector, an object for object detection
        self.__object_detector = ObjectDetector(
            obj_type=obj_type, obj_color=obj_color,
            workspace_ratio=workspace_ratio,
            ret_image_bool=ret_image,
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
            rospy.logwarn_throttle(2.0, "Vision Node - Try to get debug markers while stream is not running !")
            return False, CompressedImage()
        markers_detected, img_res = debug_markers(img)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return markers_detected, msg_img

    def __callback_debug_color(self, req):
        img = self.__video_stream.read_undistorted()
        if img is None:
            rospy.logwarn_throttle(2.0, "Vision Node - Try to get debug colors while stream is not running !")
            return CompressedImage()
        color = ColorHSV[req.color]
        img_res = debug_threshold_color(img, color)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return msg_img


if __name__ == '__main__':
    rospy.init_node('niryo_robot_vision', anonymous=False, log_level=rospy.INFO)
    try:
        vision_node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
