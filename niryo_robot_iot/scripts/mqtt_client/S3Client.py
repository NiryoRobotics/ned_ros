import os
import boto3

from niryo_robot_programs_manager.msg import ProgramLanguage


class S3Client:
    def __init__(self, bucket, s3_credentials_location, robot_programs_dir, aws_programs_dir, user_id, ):
        os.environ['AWS_CONFIG_FILE'] = s3_credentials_location

        self.__s3 = boto3.resource('s3')

        self.__client = self.__s3.Bucket(bucket)
        self.__robot_programs_location = os.path.expanduser('~/{}'.format(robot_programs_dir))
        self.__aws_programs_location = os.path.join("home", user_id, aws_programs_dir)

        self.__language_to_str_map = {
            ProgramLanguage.PYTHON2: 'python2',
            ProgramLanguage.PYTHON3: 'python3',
            ProgramLanguage.BLOCKLY: 'blockly',
        }
        for language in self.__language_to_str_map.values():
            dir_path = os.path.join(self.__robot_programs_location, language)
            if not os.path.isdir(dir_path):
                os.makedirs(dir_path)

    def upload_program(self, file_language, file_name):
        try:
            key = os.path.join(self.__aws_programs_location, self.__language_to_str_map[file_language], file_name)
            file_path = os.path.join(self.__robot_programs_location, self.__language_to_str_map[file_language],
                                     file_name)
            self.__client.upload_file(
                Key=key,
                Filename=file_path
            )
        except KeyError:
            return False

        return True

    def download_program(self, file_language, file_name):
        try:
            self.__client.download_file(
                Key='{}/{}/{}'.format(self.__aws_programs_location, self.__language_to_str_map[file_language], file_name),
                Filename='{}/{}/{}'.format(self.__robot_programs_location, self.__language_to_str_map[file_language],
                                           file_name)
            )
        except KeyError:
            return False

        return True
