# Libs
import rospy

from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import ImageEnhance
from PIL import Image as PILImage

from threading import Lock, Thread

from fonctions_camera import generate_msg_from_image

# Messages
from niryo_robot_msgs.msg import CommandStatus
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from niryo_robot_vision.msg import ImageParameters

# Service
from niryo_robot_msgs.srv import SetBool
from niryo_robot_vision.srv import SetImageParameter


class VideoStream(object):
    def __init__(self, calibration_object, publisher_compressed_stream):
        self._calibration_object = calibration_object
        self._publisher_compressed_stream = publisher_compressed_stream

        self._undistort_stream = rospy.get_param('~undistort_stream')
        self._display = rospy.get_param("~display")
        self._frame_rate = rospy.get_param("~frame_rate")
        self._subsample_read = rospy.get_param("~subsampling")

        self._actualization_rate_ros = rospy.Rate(1.1 * int(self._frame_rate))

        self._saturation = 1.0
        self._contrast = 1.0
        self._brightness = 1.0

        # - Publisher about Running/Stopped Stream
        self._running = False
        self._should_run = False

        self._publisher_camera_stream_running = rospy.Publisher('~video_stream_is_active',
                                                                Bool, queue_size=2)

        self._publisher_stream_parameters = rospy.Publisher('~video_stream_parameters',
                                                            ImageParameters, queue_size=2, latch=True)
        self._publish_image_parameters()

        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~is_active_rate")),
                    self._publish_is_active)
        # - SERVICES
        self.__service_start_stop = rospy.Service('~start_stop_video_streaming',
                                                  SetBool, self._callback_start_stop)

        rospy.Service('~set_saturation', SetImageParameter, self._callback_set_saturation)
        rospy.Service('~set_brightness', SetImageParameter, self._callback_set_brightness)
        rospy.Service('~set_contrast', SetImageParameter, self._callback_set_contrast)

    # -- CALLBACKS
    def _callback_start_stop(self, req):
        command = ["stopping", "starting"][int(req.value)]
        if command == "stopping":
            if not self._running:
                message = "The video stream is already stopped. Cannot stop it"
                rospy.logwarn("Vision Node - " + message)
                return CommandStatus.VIDEO_STREAM_ON_OFF_FAILURE, message

            else:
                self._should_run = False

                for _ in range(30):
                    if not self._running:
                        break
                    rospy.sleep(0.1)
                else:
                    message = "Failed to stop the video stream !"
                    rospy.loginfo("Vision Node - " + message)

                    return CommandStatus.VIDEO_STREAM_ON_OFF_FAILURE, message

                message = "Stream stopped"
                rospy.loginfo("Vision Node - " + message)

                return CommandStatus.SUCCESS, message

        else:
            if self._running:
                message = "The video stream is already running. Cannot start it"
                rospy.logwarn("Vision Node - " + message)

                return CommandStatus.VIDEO_STREAM_ON_OFF_FAILURE, message
            else:
                rospy.loginfo("Vision Node - Trying to launch the stream ...")
                self._should_run = True

                # Updating last read time to avoid instant stop
                self.__last_time_read = rospy.get_time()

                for _ in range(100):
                    if self._running:
                        break
                    rospy.sleep(0.1)
                else:
                    message = "Failed to relaunch the stream !"
                    rospy.loginfo("Vision Node - " + message)

                    return CommandStatus.VIDEO_STREAM_ON_OFF_FAILURE, message

                message = "Stream relaunched !"
                rospy.loginfo("Vision Node - " + message)

                return CommandStatus.SUCCESS, message

    def _callback_set_saturation(self, req):
        self._saturation = req.factor
        self._publish_image_parameters()
        return CommandStatus.SUCCESS, "Success"

    def _callback_set_brightness(self, req):
        self._brightness = req.factor
        self._publish_image_parameters()
        return CommandStatus.SUCCESS, "Success"

    def _callback_set_contrast(self, req):
        self._contrast = req.factor
        self._publish_image_parameters()
        return CommandStatus.SUCCESS, "Success"

    def _publish_is_active(self, _):
        self._publisher_camera_stream_running.publish(self._running)

    def _publish_image_parameters(self):
        self._publisher_stream_parameters.publish(self._brightness, self._contrast, self._saturation)

    # -- METHODS

    def read_undistorted(self):
        raise NotImplementedError

    def read_raw_img(self):
        raise NotImplementedError

    def start(self):
        self._should_run = True
        t = Thread(target=self._loop, name="Camera Thread")
        t.start()

    def _loop(self):
        raise NotImplementedError


class WebcamStream(VideoStream):

    def __init__(self, calibration_object, publisher_compressed_stream):
        super(WebcamStream, self).__init__(calibration_object, publisher_compressed_stream)

        self.__cam_port = rospy.get_param("~camera_port")
        self.__last_time_read = None
        self.__frame_raw = None
        self.__frame_undistort = None

        self.__lock_image = Lock()

        # Stop Stream handling
        self.__actualization_rate_no_stream = rospy.Rate(rospy.get_param("~restart_rate"))
        self.__max_restart_time = rospy.get_param("~max_restart_time")

        self.__count_grab_failed = 0

    # -- PUBLIC

    def read_undistorted(self):
        if not self._running or not self._should_run:
            return None
        with self.__lock_image:
            ret, frame = self.__video_stream.retrieve()

        frame = self.adjust_image(frame)
        if ret is False:
            return None
        if self._calibration_object.is_set():
            return self._calibration_object.undistort_image(frame)
        else:
            return frame

    def read_raw_img(self):
        if not self._running or not self._should_run:
            return None
        with self.__lock_image:
            ret, frame = self.__video_stream.retrieve()
        if ret is False:
            return None
        return frame

    # -- Private
    def __setup_stream_settings(self):
        # Set compression format
        self.__video_stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # Set Buffer size
        # -- Not available in opencv 3.4 -- #
        # self.__video_stream.set(cv2.CAP_PROP_BUFFERSIZE, rospy.get_param("~buffer_size"))

        # Set image size
        w, h = rospy.get_param("~frame_size")
        self.__video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.__video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        # Set frame rate
        self.__video_stream.set(cv2.CAP_PROP_FPS, self._frame_rate)

    def adjust_image(self, img):
        if self._brightness == self._contrast == self._saturation == 1.:
            return img

        im_pil = PILImage.fromarray(img)

        if self._brightness != 1.:
            brightness_filter = ImageEnhance.Brightness(im_pil)
            im_pil = brightness_filter.enhance(self._brightness)

        if self._contrast != 1.:
            contrast_filter = ImageEnhance.Contrast(im_pil)
            im_pil = contrast_filter.enhance(self._contrast)

        if self._saturation != 1.:
            color_filter = ImageEnhance.Color(im_pil)
            im_pil = color_filter.enhance(self._saturation)

        # For reversing the operation:
        im_np = np.asarray(im_pil)
        return im_np

    def _loop(self):
        while not rospy.is_shutdown():
            if not self._should_run:
                self._running = False
                self.__actualization_rate_no_stream.sleep()
                continue
            self.__video_stream = cv2.VideoCapture(self.__cam_port)
            if not self.__video_stream.isOpened():
                rospy.logwarn_throttle(10.0, "Vision Node - Failed to open video stream. Check camera is well plugged.")
                self._running = False
                if self.__last_time_read is None or \
                        (rospy.get_time() - self.__last_time_read) > self.__max_restart_time:
                    self._should_run = False
                self.__actualization_rate_no_stream.sleep()
                continue

            rospy.loginfo("Vision Node - Video Stream Open")
            self.__setup_stream_settings()
            self._running = True
            self._should_run = True

            index_read = -1
            while not rospy.is_shutdown() and self._should_run:
                index_read = (index_read + 1) % self._subsample_read
                with self.__lock_image:
                    bool_grabbed = self.__video_stream.grab()
                if not bool_grabbed:
                    rospy.logwarn("Vision Node - Image not grabbed. Camera may have been unplugged.")
                    rospy.logwarn("Vision Node - Closing Video Stream")
                    break
                if index_read == 0:
                    _, frame = self.__video_stream.retrieve()
                    self.__last_time_read = rospy.get_time()

                    self.__frame_raw = frame
                    if self._undistort_stream:
                        self.__frame_undistort = self._calibration_object.undistort_image(frame)
                    else:
                        self.__frame_undistort = None
                    used_image = self.__frame_undistort if self._undistort_stream else self.__frame_raw
                    used_image = self.adjust_image(used_image)
                    if self._display:
                        cv2.imshow("Video Stream", used_image)
                        cv2.waitKey(1)
                    result, msg = generate_msg_from_image(used_image)

                    if not result:
                        continue
                    rospy.logdebug("Vision Node - Publishing an image")
                    try:
                        self._publisher_compressed_stream.publish(msg)
                    except rospy.ROSException:
                        return
                self._actualization_rate_ros.sleep()

            self.__video_stream.release()
            rospy.loginfo("Vision Node - Video Stream Stopped")
            self._running = False


class GazeboStream(VideoStream):

    def __init__(self, calibration_object, publisher_compressed_stream):
        super(GazeboStream, self).__init__(calibration_object, publisher_compressed_stream)

        self.__image_raw_sub = rospy.Subscriber('/gazebo_camera/image_raw', Image,
                                                self.__callback_sub_image_raw)
        self.__image_compressed_sub = rospy.Subscriber('/gazebo_camera/image_raw/compressed', CompressedImage,
                                                       self.__callback_sub_image_compressed)
        self.__bridge = CvBridge()
        self.__last_image_raw = None
        self.__last_image_compressed_msg = CompressedImage()

        self._running = True
        self._should_run = True

    def __callback_sub_image_raw(self, image_message):
        self.__last_image_raw = self.__bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")

    def __callback_sub_image_compressed(self, msg):
        self.__last_image_compressed_msg = msg

    def read_undistorted(self):
        if self.__last_image_raw is None:
            return None
        if self._calibration_object.is_set():
            return self._calibration_object.undistort_image(self.__last_image_raw)
        else:
            return self.__last_image_raw

    def read_raw_img(self):
        return self.__last_image_raw

    def _loop(self):
        while not rospy.is_shutdown():
            if not self._should_run:
                self._running = False
                self._actualization_rate_ros.sleep()
                continue

            self._running = True
            if self.__last_image_raw is not None:
                cv2.imshow("Video Stream", self.__last_image_raw)
                cv2.waitKey(1)

            try:
                self._publisher_compressed_stream.publish(self.__last_image_compressed_msg)
            except rospy.ROSException:
                return
            self._actualization_rate_ros.sleep()
