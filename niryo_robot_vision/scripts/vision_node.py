#!/usr/bin/env python
import cv2
import numpy as np

import rospy

# Services
from niryo_robot_msgs.srv import SetBool, SetBoolRequest, SetString, SetStringRequest
from niryo_robot_vision.srv import (ActivateDebugTopics,
                                    DebugColorDetection,
                                    DebugColorDetectionRequest,
                                    DebugMarkers,
                                    DebugMarkersRequest,
                                    ObjDetection,
                                    ObjDetectionRequest,
                                    ObjDetectionResponse,
                                    SetImageParameter,
                                    SetImageParameterRequest,
                                    ActivateDebugTopicsRequest)

# Messages
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool
from niryo_robot_msgs.msg import CommandStatus, ObjectPose
from niryo_robot_vision.msg import ActiveDebugTopics, ImageParameters

# Packages
from niryo_robot_utils import sentry_init
from niryo_robot_vision.business.enums import ObjectShape, ObjectColor
from niryo_robot_vision.business import image_processing, markers, colors
from niryo_robot_vision.implem.abstract_stream import AbstractStream
from niryo_robot_vision.implem import built_in_camera, client_stream


class VisionNode:

    def __init__(self):

        self.__camera_port = rospy.get_param('~camera_port')
        self.__stream_compression_quality = rospy.get_param('~stream_compression_quality')
        self.__debug_compression_quality = rospy.get_param('~debug_compression_quality')

        camera_intrinsics = rospy.get_param('~cam_intrinsics')
        frame_rate = rospy.get_param('~frame_rate')

        camera_config = built_in_camera.CameraConfig(mtx=np.array(camera_intrinsics['mtx']),
                                                     dist=np.array(camera_intrinsics['dist']),
                                                     flip=rospy.get_param('~flip_img'),
                                                     width_img=camera_intrinsics['width_img'],
                                                     height_img=camera_intrinsics['height_img'],
                                                     acquisition_rate=frame_rate)
        self.__webcam_stream = built_in_camera.Stream(
            camera_index=self.__camera_port,
            config=camera_config,
            publish_frame_cb=self.__publish_compressed_stream,
        )
        self.__client_stream = client_stream.Stream(
            rospy.get_param('~client_stream_topic'),
            publish_frame_cb=self.__publish_compressed_stream,
        )

        self.__source = 'webcam'
        if self.__webcam_stream.is_available:
            self.__webcam_stream.start()

        self.__camera_intrinsics_publisher = rospy.Publisher('~camera_intrinsics', CameraInfo, latch=True, queue_size=1)
        self.__camera_intrinsics_publisher.publish(camera_config.to_ros_camera_info())

        # == Ros interface == #

        # Main stream
        self.__compressed_stream_publisher = rospy.Publisher('~compressed_video_stream', CompressedImage, queue_size=1)
        self.__stream_active_publisher = rospy.Publisher('~video_stream_is_active', Bool, queue_size=2)

        # Stream editing
        rospy.Service('~start_stop_video_streaming', SetBool, self.__callback_start_stop)
        rospy.Service('~set_source', SetString, self.__callback_set_source)
        rospy.Service('~set_saturation', SetImageParameter, self.__callback_set_saturation)
        rospy.Service('~set_brightness', SetImageParameter, self.__callback_set_brightness)
        rospy.Service('~set_contrast', SetImageParameter, self.__callback_set_contrast)
        self.__stream_parameters_publisher = rospy.Publisher('~video_stream_parameters',
                                                             ImageParameters,
                                                             queue_size=2,
                                                             latch=True)

        # Debug features
        self.__red_channel_publisher = rospy.Publisher('~debug/red_channel', CompressedImage, queue_size=1)
        self.__green_channel_publisher = rospy.Publisher('~debug/green_channel', CompressedImage, queue_size=1)
        self.__blue_channel_publisher = rospy.Publisher('~debug/blue_channel', CompressedImage, queue_size=1)
        self.__any_channel_publisher = rospy.Publisher('~debug/any_channel', CompressedImage, queue_size=1)
        self.__markers_publisher = rospy.Publisher('~debug/markers', CompressedImage, queue_size=1)
        self.__active_debug_topics = rospy.Publisher('~debug/active_topics',
                                                     ActiveDebugTopics,
                                                     queue_size=1,
                                                     latch=True)
        # dict to keep track of what debug topics are activated, and what method to call to get the image
        self.__activated_debug_publishers = {
            self.__red_channel_publisher: {
                'is_active': False,
                'publish_method': lambda: self.__get_image_channel_as_msg(ObjectColor.RED),
            },
            self.__green_channel_publisher: {
                'is_active': False,
                'publish_method': lambda: self.__get_image_channel_as_msg(ObjectColor.GREEN),
            },
            self.__blue_channel_publisher: {
                'is_active': False,
                'publish_method': lambda: self.__get_image_channel_as_msg(ObjectColor.BLUE),
            },
            self.__any_channel_publisher: {
                'is_active': False,
                'publish_method': lambda: self.__get_image_channel_as_msg(ObjectColor.ANY),
            },
            self.__markers_publisher: {
                'is_active': False,
                'publish_method': lambda: self.__get_image_w_markers_as_msg()[1],
            },
        }
        rospy.Service('~debug_markers', DebugMarkers, self.__callback_debug_markers)
        rospy.Service('~debug_colors', DebugColorDetection, self.__callback_debug_color)
        rospy.Service('~debug/activate', ActivateDebugTopics, self.__callback_activate_debug)

        # Object detection
        rospy.Service('~obj_detection_rel', ObjDetection, self.__callback_get_obj_relative_pose)

        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~is_active_rate')), self.__publish_is_active)
        rospy.Timer(rospy.Duration(1.0 / frame_rate), self.__publish_debug_topics_callback)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo('Vision Node - Initialized')

    @property
    def __stream(self) -> AbstractStream:
        """
        Return the selected stream.
        :return:
        """
        return {
            'webcam': self.__webcam_stream,
            'client': self.__client_stream,
        }[self.__source]

    # - CALLBACK
    def __callback_get_obj_relative_pose(self, req: ObjDetectionRequest):
        """
        Callback to get the relative pose of an object detected in the image.
        :param req: The request containing the object color and type
        :return: The response containing the object relative x, y and yaw and its shape and color
        """
        try:
            found_object = image_processing.detect_object(self.__stream.get_image(),
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
        """
        Callback to get the image with the markers drawn on it.
        :param _: The request
        :return: The response containing the image
        """
        return CommandStatus.SUCCESS, 'Successfully got debug markers', *self.__get_image_w_markers_as_msg()

    def __callback_debug_color(self, req: DebugColorDetectionRequest):
        """
        Callback to get the image with the specified color channel.
        :param req: The request containing the color channel
        :return: The response containing the image
        """
        try:
            color = ObjectColor[req.color]
        except image_processing.NoShapeFoundError:
            return CommandStatus.FAILURE, f'Invalid color. Possible values are {[c.name for c in ObjectColor]}', None
        try:
            return CommandStatus.SUCCESS, 'Successfully got debug colors', self.__get_image_channel_as_msg(color)
        except Exception as e:
            return CommandStatus.FAILURE, f'Failed to get debug colors: {e}', None

    def __callback_activate_debug(self, req: ActivateDebugTopicsRequest):
        self.__activated_debug_publishers[self.__red_channel_publisher]['is_active'] = req.topics.red_channel
        self.__activated_debug_publishers[self.__green_channel_publisher]['is_active'] = req.topics.green_channel
        self.__activated_debug_publishers[self.__blue_channel_publisher]['is_active'] = req.topics.blue_channel
        self.__activated_debug_publishers[self.__any_channel_publisher]['is_active'] = req.topics.any_channel
        self.__activated_debug_publishers[self.__markers_publisher]['is_active'] = req.topics.markers

        self.__active_debug_topics.publish(req.topics)
        return CommandStatus.SUCCESS, 'Successfully updated the debug topics'

    def __callback_start_stop(self, req: SetBoolRequest):
        """
        Callback to start or stop the video stream.
        :param req: The request containing the desired state
        :return: The response indicating success or failure
        """
        if req.value is True:
            if not self.__stream.is_available:
                return CommandStatus.FAILURE, 'Stream is not available.'

            try:
                self.__stream.start()
            except RuntimeError:
                return True, 'Stream already started'
            self.__stream_active_publisher.publish(Bool(data=True))
            return True, 'Video stream started'
        else:
            try:
                self.__stream.stop()
            except RuntimeError:
                return True, 'Stream already stopped'
            self.__stream_active_publisher.publish(Bool(data=False))
            return True, 'Video stream stopped'

    def __callback_set_source(self, req: SetStringRequest):
        if req.value not in ['webcam', 'client']:
            return CommandStatus.FAILURE, 'Invalid source. Possible values are [webcam, client]'

        try:
            self.__stream.stop()
        except RuntimeError:
            pass

        self.__source = req.value
        self.__stream.start()

        return CommandStatus.SUCCESS, f'Source set successfully set to {req.value}'

    def __callback_set_saturation(self, req: SetImageParameterRequest):
        """
        Callback to set the saturation of the image stream.
        :param req: The request containing the saturation factor
        :return: The response indicating success or failure
        """
        self.__stream.saturation = req.factor
        self.__stream_parameters_publisher.publish(
            ImageParameters(brightness_factor=self.__stream.brightness,
                            contrast_factor=self.__stream.contrast,
                            saturation_factor=self.__stream.saturation))
        return CommandStatus.SUCCESS, 'Saturation set successfully'

    def __callback_set_brightness(self, req: SetImageParameterRequest):
        """
        Callback to set the brightness of the image stream.
        :param req: The request containing the brightness factor
        :return: The response indicating success or failure
        """
        self.__stream.brightness = req.factor
        return CommandStatus.SUCCESS, 'Brightness set successfully'

    def __callback_set_contrast(self, req: SetImageParameterRequest):
        """
        Callback to set the contrast of the image stream.
        :param req:  The request containing the contrast factor
        :return:  The response indicating success or failure
        """
        self.__stream.contrast = req.factor
        return CommandStatus.SUCCESS, 'Contrast set successfully'

    def __publish_is_active(self, _: rospy.timer.TimerEvent):
        """
        Publish the is_active parameter.
        :param _: The timer event
        """
        self.__stream_active_publisher.publish(Bool(data=self.__stream.is_active))

    def __publish_compressed_stream(self, frame: np.ndarray):
        """
        Publish the compressed image stream. This callback is called by the stream when a new frame is available.
        :param frame: The frame to publish
        :type frame: numpy.ndarray
        """
        self.__compressed_stream_publisher.publish(self.__generate_msg_from_frame(frame))

    def __publish_debug_topics_callback(self, _: rospy.timer.TimerEvent):
        """
        Publish the debug topics. To reduce the load on the CPU, we only publish (and thus computing the images) on the
        debug topics if there is at least one subscriber.
        :param _: The timer event
        """
        for publisher, topic_info in self.__activated_debug_publishers.items():
            if not topic_info['is_active']:
                continue
            try:
                publisher.publish(topic_info['publish_method']())
            except Exception as e:
                rospy.logerr(f'Failed to publish debug topic {publisher.resolved_name}: {e}')

    def __generate_msg_from_frame(self, frame, compression_quality: float = None) -> CompressedImage:
        """
        Generate a CompressedImage message from a frame. It encodes the image in JPEG format.
        :param frame: The frame to compress
        :param compression_quality: The compression quality to use. If None, use the classical stream value.
        The value must be between 0 and 100.
        :return: The CompressedImage message
        """
        if compression_quality is None:
            compression_quality = self.__stream_compression_quality

        _, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), compression_quality])
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpg'
        msg.data = encoded_img.tobytes()
        return msg

    def __get_image_channel_as_msg(self, color: ObjectColor) -> CompressedImage:
        """
        Get the image with the specified color channel and return it as a CompressedImage message.
        :param color: The color channel to get
        :return: The CompressedImage message
        """
        return self.__generate_msg_from_frame(
            colors.filter_channel(self.__stream.get_image(), color),
            compression_quality=self.__debug_compression_quality,
        )

    def __get_image_w_markers_as_msg(self) -> (bool, CompressedImage):
        """
        Get the image with the markers drawn on it and return it as a CompressedImage message.
        :return: A tuple containing a boolean indicating if the markers were found and the CompressedImage message
        """
        found, frame = markers.draw_markers(self.__stream.get_image())
        return found, self.__generate_msg_from_frame(
            frame,
            compression_quality=self.__debug_compression_quality,
        )


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_vision', anonymous=False, log_level=rospy.INFO)

    try:
        vision_node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
