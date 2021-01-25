import rospy

import os
import json
from S3Client import S3Client

# Message
from niryo_robot_programs_manager.msg import ProgramLanguage

# Services
from niryo_robot_msgs.srv import SetBool, Trigger
from niryo_robot_programs_manager.srv import ExecuteProgram, ExecuteProgramRequest


class MqttSubscriber:

    def __init__(self, s3_credentials_location, s3_bucket, robot_programs_dir, aws_programs_dir, user_id):
        self.__s3_credentials_location = s3_credentials_location
        self.__s3_client = S3Client(s3_bucket, s3_credentials_location, robot_programs_dir, aws_programs_dir, user_id)

    @staticmethod
    def __integrity_check(topic, raw_json, keys):
        try:
            values = json.loads(raw_json)
        except ValueError:
            rospy.logerr("MQTT Subscriber - Topic {}: unable to parse the JSON".format(topic))
            return {}

        rospy.logdebug("MQTT Subscriber - Integrity check for topic {}:".format(topic))
        is_good = True
        for key, value in values.iteritems():
            if key not in keys:
                is_good = False
                rospy.logerr('\t{}: {} -> not an accepted key'.format(key, value))
            else:
                rospy.logdebug('\t{}: {}'.format(key, value))
        if not is_good:
            return {}

        if len(keys) != len(values):
            rospy.logerr("Error: {} parameters expected, {} provided".format(len(keys), len(values)))
            rospy.logerr("\tExpected: {}".format(', '.join(keys)))
            rospy.logerr("\tProvided: {}".format(', '.join(values)))
            return {}

        return values

    @staticmethod
    def __call_ros_service(topic_name, service_type, success_message, req=None):
        rospy.wait_for_service(topic_name)
        service_function = rospy.ServiceProxy(topic_name, service_type)
        res = service_function(req) if req is not None else service_function()
        if res.status < 0:
            rospy.logwarn(res.message)
        else:
            rospy.logdebug(success_message)
        return res.status

    def set_learning_mode(self, _client, _userdata, message):
        ros_service = '/niryo_robot/activate_learning_mode'
        values = self.__integrity_check(message.topic, message.payload, ['value'])
        if not values:
            rospy.logerr("MQTT Subscriber - Aborting call to the ROS service {}".format(ros_service))
            return

        req = values["value"] in ['true', 'True', '1']

        self.__call_ros_service(
            topic_name=ros_service,
            service_type=SetBool,
            success_message='Learning mode successfully set to {}'.format(values['value']),
            req=req
        )

    def execute_program(self, _client, _userdata, message):
        ros_service = '/niryo_robot_programs_manager/execute_program'
        values = self.__integrity_check(message.topic, message.payload, ['name', 'language'])
        if not values:
            rospy.logerr("MQTT Subscriber - Aborting call to the ROS service {}".format(ros_service))
            return

        req = ExecuteProgramRequest()
        req.execute_from_string = False
        req.name = values['name'].encode("ascii", "ignore")
        req.language = ProgramLanguage()
        req.language.used = values['language']

        self.__call_ros_service(
            topic_name=ros_service,
            service_type=ExecuteProgram,
            success_message='Program successfully started',
            req=req
        )

    def stop_program(self, _client, _userdata, message):
        self.__integrity_check(message.topic, message.payload, [])
        self.__call_ros_service(
            topic_name='/niryo_robot_programs_manager/stop_program',
            service_type=Trigger,
            success_message='Program stopped'
        )

    def upload_program_to_aws(self, _client, _userdata, message):
        rospy.logdebug('MQTT Subscriber - Uploading to AWS')
        values = self.__integrity_check(message.topic, message.payload, ['name', 'type', 'language'])
        if not values:
            rospy.logerr('MQTT Subscriber - Upload prog to AWS - Incorrect values given')
            return False

        try:
            result = self.__s3_client.upload_program(values['language'], values['name'])
        except KeyError:
            rospy.logerr('MQTT Subscriber - Upload prog to AWS - Invalid language given')
            return False
        if result:
            rospy.loginfo('MQTT Subscriber - Uploaded {} successfully'.format(values['name']))

    def download_program_from_aws(self, _client, _userdata, message):
        rospy.loginfo('MQTT Subscriber - Downloading from AWS')
        values = self.__integrity_check(message.topic, message.payload, ['name', 'type', 'language'])
        if not values:
            rospy.logerr('MQTT Subscriber - Download prog from AWS - Incorrect values given')
            return False

        try:
            result = self.__s3_client.download_program(values['language'], values['name'])
        except KeyError:
            rospy.logerr('MQTT Subscriber - Download prog from AWS - Invalid language given')
            return False
        if result:
            rospy.loginfo('MQTT Subscriber - Downloaded {} successfully'.format(values['name']))

    def get_aws_credentials(self, _client, _userdata, message):
        values = self.__integrity_check(message.topic, message.payload,
                                        ['access_key', 'secret_key', 'user_id', 'port_ssh'])
        if not values:
            rospy.logerr('MQTT Subscriber - Get AWS Credentials - Incorrect values given')
            return False

        try:
            f = open(os.path.join(os.path.expanduser(self.__s3_credentials_location), "s3"), 'w+')
        except IOError:
            rospy.logerr("MQTT Subscriber - Error while trying to open the file AWS credentials")
            return False

        f.write("[default]\naws_access_key_id = {}\naws_secret_access_key = {}\nregion = eu-west-3\n".format(
            values['access_key'], values['secret_key']))
        f.close()

        try:
            f = open(os.path.join(os.path.expanduser(self.__s3_credentials_location), "user_id"), 'w+')
        except IOError:
            rospy.logerr("MQTT Subscriber - Error while trying to open the file used_id")
            return False
        f.write(values['user_id'])
        f.close()

        rospy.loginfo('MQTT Subscriber - Successfully fetched the AWS credentials')

    def receive_pong(self, _client, _userdata, message):
        values = self.__integrity_check(message.topic, message.payload, ['id'])
        if not values:
            rospy.logerr('MQTT Subscriber - Pong - Incorrect values given')
            return False

        try:
            open('/tmp/ping_{}'.format(values['id']), 'w').close()
        except IOError:
            rospy.logerr('MQTT Subscriber - Pong - Unable to create the file')
            return False

        return True
