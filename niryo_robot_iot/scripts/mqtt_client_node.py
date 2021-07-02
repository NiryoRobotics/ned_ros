#!/usr/bin/env python

# Libs
import rospy
import logging

import os
import subprocess
from paho.mqtt.client import connack_string, CONNACK_ACCEPTED, MQTT_ERR_SUCCESS

from mqtt_client.MqttSubscriber import MqttSubscriber
from mqtt_client.MqttClient import MqttClient
from mqtt_client.MqttPublisher import MqttPublisher

# Message / Services
from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_iot.srv import GetSerial
from niryo_robot_iot.srv import MqttPublish


class MqttClientNode:
    def __init__(self):
        rospy.logdebug("MQTT Client - Entering in Init")

        # - Ros Params
        self.__aws_endpoint = rospy.get_param('~aws_endpoint')
        rospy.logdebug("MqttClientNode.Init - aws_endpoint: %s", self.__aws_endpoint)

        self.__aws_port = rospy.get_param('~aws_port')
        rospy.logdebug("MqttClientNode.Init - aws_port: %s", self.__aws_port)

        conf_location = os.path.expanduser(rospy.get_param('~conf_location'))
        rospy.logdebug("MqttClientNode.Init - conf_location: %s", conf_location)

        self.__certs_location = "{}/certs".format(conf_location)
        self.__credentials_location = "{}/credentials".format(conf_location)

        self.__still_alive_publish_duration = rospy.get_param('~still_alive_publish_duration')
        rospy.logdebug("MqttClientNode.Init - still_alive_publish_duration: %s", self.__still_alive_publish_duration)

        # - Serial

        self.__serial = self.get_serial()

        # - Credentials

        self.__ca_file = os.path.join(self.__certs_location, rospy.get_param('~ca_name'))
        rospy.logdebug("MqttClientNode.Init - ca_file: %s", self.__ca_file)

        cert_filename = '{}{}'.format(self.__serial, rospy.get_param('~cert_suffix'))
        self.__cert_file = os.path.join(self.__certs_location, cert_filename)
        rospy.logdebug("MqttClientNode.Init - cert_file: %s", self.__cert_file)

        key_filename = '{}{}'.format(self.__serial, rospy.get_param('~key_suffix'))
        self.__key_file = os.path.join(self.__certs_location, key_filename)
        rospy.logdebug("MqttClientNode.Init - key_file: %s", self.__key_file)

        self.wait_for_credentials_creation()
        self.wait_for_internet_connection()

        self.__user_id = self.get_user_id()

        # - MQTT Client
        self.__mqtt_client = MqttClient(self.__serial, self.__ca_file, self.__cert_file, self.__key_file)
        self.__mqtt_client.set_callbacks(self.__on_connect_callback, self.__on_disconnect_callback,
                                         self.__on_message_callback)

        # - MQTT Publisher
        self.__mqtt_publisher = MqttPublisher(self.__mqtt_client.client, self.__serial, self.__user_id)

        # - MQTT Subscriber
        s3_credentials_location = rospy.get_param('~s3_credentials_location')
        s3_bucket = rospy.get_param('~private_bucket')
        s3_programs_dir = rospy.get_param('~s3_programs_dir')
        robot_programs_dir = rospy.get_param('/niryo_robot_programs_manager/programs_dir',
                                             default="niryo_robot_programs")

        rospy.logdebug("MqttClientNode.Init - s3_bucket: %s", s3_bucket)
        rospy.logdebug("MqttClientNode.Init - s3_programs_dir: %s", s3_programs_dir)
        rospy.logdebug("MqttClientNode.Init - robot_programs_dir: %s", robot_programs_dir)

        self.__subscriber = MqttSubscriber(s3_credentials_location, s3_bucket,
                                           robot_programs_dir, s3_programs_dir, self.__user_id)
        self.__subscribe_topics = self.generate_subscribed_topics_dict()

        # Service for publish
        rospy.Service('~mqtt_publish', MqttPublish, self.__mqtt_publish_callback)

        # - End of Init
        self.start()

        # Publisher Still Alive
        rospy.Timer(rospy.Duration(self.__still_alive_publish_duration), self.__publish_still_alive)

        # Stop on ROS shutdown
        rospy.on_shutdown(self.stop)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("MQTT Client - Node Started")

    # - Init
    @staticmethod
    def get_serial():
        service_name = "/niryo_robot_serial_number/get"
        rospy.wait_for_service(service_name)
        get_serial_service = rospy.ServiceProxy(service_name, GetSerial)
        serial = get_serial_service().message
        return serial

    def generate_subscribed_topics_dict(self):
        topic_prefix = 'topic/{}'.format(self.__serial)

        subscribe_topics = {
            '{}/aws_credentials'.format(topic_prefix): self.__subscriber.get_aws_credentials,
            '{}/programs_manager/execute_program'.format(topic_prefix): self.__subscriber.execute_program,
            '{}/programs_manager/stop_program'.format(topic_prefix): self.__subscriber.stop_program,
            '{}/programs_manager/robot_to_aws'.format(topic_prefix): self.__subscriber.upload_program_to_aws,
            '{}/programs_manager/aws_to_robot'.format(topic_prefix): self.__subscriber.download_program_from_aws,
            '{}/pong'.format(topic_prefix): self.__subscriber.receive_pong,
        }
        return subscribe_topics

    def wait_for_credentials_creation(self):
        # We NEED to wait for the credentials since the first time the robot will
        # start it won't have them but will still need to communicate with AWS later

        rate = rospy.Rate(0.1)
        while not os.path.isfile(self.__ca_file) or not os.path.isfile(self.__cert_file) \
                or not os.path.isfile(self.__key_file):
            rospy.logwarn_once('MQTT Client - Waiting for the certificates, re-checking every 10s...')
            rospy.logdebug('MQTT Client - Waiting for the certificates, re-checking in 10s...')
            rate.sleep()

    @staticmethod
    def test_internet_connection():
        dev_null = open(os.devnull, 'w')
        try:
            ping_process = subprocess.check_call(['ping', '-c', '1', '8.8.8.8'], stdout=dev_null, stderr=dev_null)
            result = ping_process == 0
        except subprocess.CalledProcessError:
            result = False

        dev_null.close()
        return result

    def wait_for_internet_connection(self):
        # This is to prevent the node to crash if there is no internet connection
        rate = rospy.Rate(0.1)
        while not self.test_internet_connection():
            rospy.logwarn_once('MQTT Client - Waiting for an internet connection, re-checking every 10s...')
            rospy.logdebug('MQTT Client - Waiting for an internet connection, re-checking in 10s...')
            rate.sleep()

    # - MQTT Callbacks
    @staticmethod
    def __on_connect_callback(_client, _userdata, _flags, rc):
        if rc == CONNACK_ACCEPTED:
            rospy.logdebug("MQTT Client - Connected")
        else:
            rospy.logwarn('MQTT Client - Connection failed : {}'.format(connack_string(rc)))

    @staticmethod
    def __on_disconnect_callback(_client, _userdata, rc):
        if rc != MQTT_ERR_SUCCESS:
            rospy.logwarn("MQTT Client - Unexpected disconnection")
        else:
            rospy.logdebug("MQTT Client - Disconnected successfully")

    @staticmethod
    def __on_message_callback(_client, _userdata, message):
        rospy.logwarn("MQTT Client - Unhandled payload received. "
                      "This means there is no callback function for this topics but it's still subscribed")
        rospy.logwarn("\ttopic: {}".format(message.topic))
        rospy.logwarn("\tpayload: {}".format(message.payload))

    # - ROS callbacks

    def __publish_still_alive(self, _):
        self.__mqtt_publisher.ping_still_alive()

    def __mqtt_publish_callback(self, req):
        try:
            pub_func = getattr(self.__mqtt_publisher, req.command)
        except AttributeError:
            return (CommandStatus.MQTT_PUBLISH_FUNCTION_DOESNT_EXIST,
                    "The publisher doesn't have the function {}".format(req.command))

        try:
            pub_func(*req.args)
        except TypeError as e:
            return CommandStatus.MQTT_PUBLISH_FUNCTION_INVALID_ARGUMENTS, str(e)

        return CommandStatus.SUCCESS, "Successfully published"

    # - Regular functions

    def get_user_id(self):
        try:
            f = open(os.path.join(self.__credentials_location, 'user_id'), 'r')
        except IOError:
            return "000000000000000000000000"
        # reads the 24 first characters of the file as a user_id is 24 char long.
        # This is to prevent trailing newline characters
        user_id_ = f.read(24)
        f.close()
        return user_id_

    def open_client_connection(self):
        rospy.loginfo("MQTT Client - Trying to connect...")
        self.__mqtt_client.connect(self.__aws_endpoint, self.__aws_port)
        # Without this tempo, the client disconnect almost every time
        rospy.sleep(5)
        rospy.logdebug("MQTT Client - Subscribing to the topics...")
        self.__mqtt_client.subscribe_all(self.__subscribe_topics)
        rospy.logdebug("MQTT Client - Connected & Subscribed to the topics")
        for topic, callback_function in self.__subscribe_topics.items():
            rospy.logdebug("MQTT Client - Handling {} with {}".format(topic, callback_function.__name__))
            self.__mqtt_client.add_topic_callback(topic, callback_function)

    def close_client_connection(self):
        rospy.logdebug("MQTT Client - Unsubscribing to the topics...")
        self.__mqtt_client.unsubscribe_all(self.__subscribe_topics)
        rospy.logdebug("MQTT Client - Unsubscribed to the topic")
        rospy.logdebug("MQTT Client - Disconnecting...")
        self.__mqtt_client.disconnect()
        rospy.loginfo("MQTT Client - Unsubscribed & Disconnected")

    def retry_connection(self):
        self.__mqtt_client.disconnect()
        self.__mqtt_client.connect(self.__aws_endpoint, self.__aws_port)

    def start(self):
        self.open_client_connection()
        self.__mqtt_client.loop_start()

    def stop(self):
        self.__mqtt_client.loop_stop()
        self.close_client_connection()


if __name__ == "__main__":
    rospy.init_node('niryo_robot_mqtt_client', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = MqttClientNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
