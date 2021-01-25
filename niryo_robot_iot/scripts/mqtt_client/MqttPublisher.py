import rospy

import subprocess

import re
import uuid
import json

from niryo_robot_programs_manager.srv import GetProgramList, GetProgramListRequest
from niryo_robot_programs_manager.msg import ProgramLanguage


class MqttPublisher:

    def __init__(self, client, serial, user_id):
        self.client = client
        self.__serial = serial
        self.__topics_prefix = 'topic/{}'.format(serial)
        self.__user_id = user_id

    def __prepare_payload(self, payload):
        payload.update({
            'user_id': self.__user_id,
            'serial': self.__serial
        })
        return json.dumps(payload)

    def __publish(self, topic, payload=None):
        rospy.logdebug('MQTT Publisher - Publishing on {}'.format(topic))
        if payload is None:
            payload = {}
        if len(payload) >= 130000:
            rospy.logwarn('MQTT Publisher - Error : payload overflown')
            return
        self.client.publish(topic, self.__prepare_payload(payload), 1)

    def __topic_builder(self, topic):
        return '{}/{}'.format(self.__topics_prefix, topic)

    def send_programs_list(self):
        program_list = rospy.ServiceProxy('/niryo_robot_programs_manager/get_program_list', GetProgramList)

        programs = {}
        req = GetProgramListRequest()

        req.language.used = ProgramLanguage.PYTHON2
        programs['python2'] = program_list(req).programs_names

        req.language.used = ProgramLanguage.PYTHON3
        programs['python3'] = program_list(req).programs_names

        req.language.used = ProgramLanguage.BLOCKLY
        programs['blockly'] = program_list(req).programs_names

        self.__publish(self.__topic_builder('programs_manager/get_program_list'), programs)

    def ping_still_alive(self):
        try:
            ip = subprocess.check_output(['curl', '-s', 'https://api.ipify.org'])
        except subprocess.CalledProcessError:
            ip = '0.0.0.0'

        if not re.match(r'^([0-2]?[0-9]{1,2}\.){3}[0-2]?[0-9]{1,2}$', ip):
            ip = '0.0.0.0'

        self.__publish(
            self.__topic_builder('still_alive'),
            {
                'ip': ip,
            }
        )

    def send_ping(self, seq=None):
        if seq is None:
            seq = uuid.uuid1()
        self.__publish(self.__topic_builder('ping'), {'id': seq})
