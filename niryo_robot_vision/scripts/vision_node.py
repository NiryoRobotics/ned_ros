#!/usr/bin/env python
import cv2
import numpy as np
from niryo_robot_msgs.msg import CommandStatus, ObjectPose

import rospy
from niryo_robot_utils import sentry_init

# Services
from niryo_robot_msgs.srv import SetBool, SetBoolRequest

from niryo_robot_vision.implem.built_in_camera import CameraConfig
from niryo_robot_vision.srv import (DebugColorDetection,
                                    DebugColorDetectionRequest,
                                    DebugMarkers,
                                    DebugMarkersRequest,
                                    ObjDetection,
                                    ObjDetectionRequest,
                                    ObjDetectionResponse,
                                    SetImageParameter,
                                    SetImageParameterRequest)

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from niryo_robot_vision.msg import ImageParameters

from niryo_robot_vision.business.enums import ObjectShape, ObjectColor
from niryo_robot_vision.implem.abstract_stream import AbstractStream
from niryo_robot_vision.implem import built_in_camera, client_stream
from niryo_robot_vision.business import image_processing


class VisionNode:

    def __init__(self):

        self.__camera_port = rospy.get_param('~camera_port')
        self.__stream_compression_quality = rospy.get_param('~stream_compression_quality')

        camera_intrinsics = rospy.get_param('~cam_intrinsics')

        self.__webcam_stream = built_in_camera.Stream(
            camera_index=self.__camera_port,
            config=CameraConfig(mtx=np.array(camera_intrinsics['mtx']),
                                dist=np.array(camera_intrinsics['dist']),
                                flip=rospy.get_param('~flip_img'),
                                width_img=camera_intrinsics['width_img'],
                                height_img=camera_intrinsics['height_img'],
                                acquisition_rate=rospy.get_param('~frame_rate')),
            publish_frame_cb=self.__publish_compressed_stream,
        )
        if self.__webcam_stream.is_available:
            self.__webcam_stream.start()
        # self.__client_stream = client_stream.Stream()

        # Publishers
        self.__compressed_stream_publisher = rospy.Publisher('~compressed_video_stream', CompressedImage, queue_size=1)
        self.__stream_active_publisher = rospy.Publisher('~video_stream_is_active', Bool, queue_size=2)
        self.__stream_parameters_publisher = rospy.Publisher('~video_stream_parameters',
                                                             ImageParameters,
                                                             queue_size=2,
                                                             latch=True)

        # Object detection
        rospy.Service('~obj_detection_rel', ObjDetection, self.__callback_get_obj_relative_pose)

        # Debug features
        rospy.Service('~debug_markers', DebugMarkers, self.__callback_debug_markers)
        rospy.Service('~debug_colors', DebugColorDetection, self.__callback_debug_color)

        # Stream parameters
        rospy.Service('~start_stop_video_streaming', SetBool, self.__callback_start_stop)
        rospy.Service('~set_saturation', SetImageParameter, self.__callback_set_saturation)
        rospy.Service('~set_brightness', SetImageParameter, self.__callback_set_brightness)
        rospy.Service('~set_contrast', SetImageParameter, self.__callback_set_contrast)

        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~is_active_rate')), self.__publish_is_active)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo('Vision Node - Initialized')

    @property
    def __stream(self) -> AbstractStream:
        return self.__webcam_stream
        if self.__client_stream.is_active:
            return self.__client_stream
        else:
            return self.__webcam_stream

    # - CALLBACK
    def __callback_get_obj_relative_pose(self, req: ObjDetectionRequest):
        try:
            found_object = image_processing.detect_object(self.__stream.image,
                                                          ObjectColor[req.obj_color],
                                                          ObjectShape[req.obj_type],
                                                          req.workspace_ratio)
        except image_processing.MarkersNotFoundError:
            return CommandStatus.MARKERS_NOT_FOUND, None, None, None, None
        except image_processing.NoShapeFoundError:
            return CommandStatus.OBJECT_NOT_FOUND, None, None, None, None

        return ObjDetectionResponse(
            status=CommandStatus.SUCCESS,
            obj_pose=ObjectPose(x=found_object.x, y=found_object.y, yaw=found_object.yaw),
            obj_type=found_object.shape.name,
            obj_color=found_object.color.name,
        )

    def __callback_debug_markers(self, _: DebugMarkersRequest):
        ...
        # return CommandStatus.SUCCESS, 'Successfully got debug markers', markers_detected, msg_img

    def __callback_debug_color(self, req: DebugColorDetectionRequest):
        ...
        # return CommandStatus.SUCCESS, 'Successfully got debug colors', msg_img

    def __callback_start_stop(self, req: SetBoolRequest):
        '''
        Callback to start or stop the video stream.
        :param req: The request containing the desired state
        :type req: SetBoolRequest
        :return: The response indicating success or failure
        :rtype: SetBoolResponse
        '''
        if req.value is True:
            try:
                self.__stream.start()
            except RuntimeError as e:
                rospy.logerr(f'Failed to start stream: {e}')
                return False, f'Failed to start stream: {e}'
            self.__stream_active_publisher.publish(Bool(data=True))
            return True, 'Video stream started'
        else:
            self.__stream.stop()
            self.__stream_active_publisher.publish(Bool(data=False))
            return True, 'Video stream stopped'

    def __callback_set_saturation(self, req: SetImageParameterRequest):
        ...

    def __callback_set_brightness(self, req: SetImageParameterRequest):
        ...

    def __callback_set_contrast(self, req: SetImageParameterRequest):
        ...

    def __publish_is_active(self, _: rospy.timer.TimerEvent):
        '''
        Publish the is_active parameter.
        :param _: The timer event
        :type _: rospy.timer.TimerEvent
        '''
        self.__stream_active_publisher.publish(Bool(data=self.__stream.is_active))

    def __publish_compressed_stream(self, frame: np.ndarray):
        '''
        Publish the compressed image stream.
        :param frame: The frame to publish
        :type frame: numpy.ndarray
        '''
        _, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.__stream_compression_quality])

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpg'
        msg.data = encoded_img.tobytes()
        self.__compressed_stream_publisher.publish(msg)


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_vision', anonymous=False, log_level=rospy.INFO)

    try:
        vision_node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
