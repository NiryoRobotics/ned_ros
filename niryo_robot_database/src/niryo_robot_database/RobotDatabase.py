import time
import rospy
import uuid
from collections import defaultdict

from niryo_robot_msgs.msg import HardwareStatus, RobotState, SoftwareVersion

VERSION = '4.1.0'


class UnknownRobotDatabaseException(Exception):
    pass


class RobotDatabase:
    def __init__(self, dao):
        self.__dae = dao

        self.__joints_db = JointsDatabase(dao)
        self.__robot_version_db = RobotVersionDatabase(dao)

        rospy.Subscriber('/niryo_robot_hardware_interface/software_version', SoftwareVersion, self.__sw_callback)
        rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status', HardwareStatus, self.__hw_callback)

    def __sw_callback(self, msg):
        self.__robot_version_db.set_robot_version(rpi_image_version=msg.rpi_image_version,
                                                  ros_version=msg.ros_niryo_robot_version,
                                                  hardware_version=msg.robot_version)

        self.__joints_db.set_joints_versions(msg.motor_names, msg.stepper_firmware_versions)

    def __hw_callback(self, msg):
        self.__joints_db.set_joints_motors(msg.motor_names, msg.motor_types, msg.hardware_errors_message)


class JointsDatabase:
    def __init__(self, dao):
        self.__dao = dao
        self.__table_name = 'joints'
        self.__joints = defaultdict(lambda: {'version': '', 'motor': '', 'error': ''})

    def set_joints_versions(self, joint_names, fw_versions):
        for joint_name, fw_version in zip(joint_names, fw_versions):
            joint = self.__joints[joint_name]
            if joint['version'] != fw_version:
                joint.update({'version': fw_version})
                self.set_joint(joint_name, joint['motor'], joint['version'], joint['error'])

    def set_joints_motors(self, joint_names, motor_types, motor_errors):
        for joint_name, motor, error in zip(joint_names, motor_types, motor_errors):
            joint = self.__joints[joint_name]
            if joint['motor'] != motor or joint['error'] != error:
                joint.update({'motor': motor, 'error': error})
                self.set_joint(joint_name, joint['motor'], joint['version'], joint['error'])

    def set_joints(self):
        for joint_name, joint in self.__joints.items():
            self.set_joint(joint_name, joint['motor'], joint['version'], joint['error'])

    def exists(self, joint_name):
        query = 'SELECT joint_name FROM {} WHERE joint_name = :joint_name'.format(self.__table_name)
        result = self.__dao.execute(query, {'joint_name': joint_name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['joint_name']

    def get_joint(self, name):
        query = 'SELECT value, type FROM {} WHERE name = :name'.format(self.__table_name)
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownRobotDatabaseException()
        return result['motor'], result['firmware_version'], result['error_code'], result['update_date']

    def set_joint(self, joint_name, motor, firmware_version, error_code=''):
        settings_exists, settings_id = self.exists(joint_name)

        params = {
            'motor': motor,
            'firmware_version': firmware_version,
            'error_code': error_code,
            'update_date': time.strftime("%Y%m%d-%H%M%S"),
            'joint_name': joint_name,
        }

        if settings_exists:
            query = 'UPDATE {} SET ' \
                    'motor=:motor, firmware_version=:firmware_version, ' \
                    'error_code=:error_code, update_date=:update_date ' \
                    'WHERE joint_name=:joint_name'.format(self.__table_name)
        else:
            query = 'INSERT INTO {} VALUES (:joint_name, :motor, :firmware_version, :error_code, :update_date)'.format(
                self.__table_name)

        self.__dao.execute(query, params)

    def dict_to_update_request(self, params, primary_key):
        query = 'UPDATE {} SET motor = '.format(self.__table_name)
        for param_name in params:
            if param_name != primary_key:
                query += '{}: {}, '.format(param_name, param_name)
        if len(params) > 1:
            query = query[:-2]
        query += 'WHERE {} = :{}'.format(primary_key, primary_key)


class RobotVersionDatabase:
    def __init__(self, dao):
        self.__dao = dao
        self.__table_name = 'robot_version'

        self.rpi_image_version = None
        self.ros_version = None
        self.hardware_version = None

    def set_robot_version(self, rpi_image_version, ros_version, hardware_version):
        if self.rpi_image_version != rpi_image_version:
            self.rpi_image_version = rpi_image_version
            self.set('rpi_image_version', rpi_image_version)

        if self.ros_version != ros_version:
            self.ros_version = ros_version
            self.set('ros_version', ros_version if ros_version else VERSION)

        if self.hardware_version != hardware_version:
            self.hardware_version = hardware_version
            self.set('hardware_version', hardware_version)

    def exists(self, name):
        query = 'SELECT id FROM {} WHERE name=:name'.format(self.__table_name)
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['id']

    def get(self, name):
        query = 'SELECT value, type FROM {} WHERE name=:name'.format(self.__table_name)
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownRobotDatabaseException()
        return result['value']

    def set(self, name, value):
        settings_exists, settings_id = self.exists(name)

        params = {
            'name': name,
            'value': value,
            'update_date': time.strftime("%Y%m%d-%H%M%S"),
        }

        if settings_exists:
            query = 'UPDATE {} SET value=:value, update_date=:update_date WHERE name=:name'.format(self.__table_name)
        else:
            query = 'INSERT INTO {} VALUES (:id, :name, :value, :update_date)'.format(self.__table_name)
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
