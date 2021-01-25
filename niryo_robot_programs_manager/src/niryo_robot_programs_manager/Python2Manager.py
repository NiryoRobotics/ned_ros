import os
import subprocess

from niryo_robot_programs_manager.ProgramsFileManager import ProgramsFileManager, FileDoesNotExistException
from niryo_robot_programs_manager.programs_manager_enums import LanguageEnum

triple_quotes = '"""'
len_triple_quotes = len(triple_quotes)
line_jump = "\n"
len_line_jump = len(line_jump)


class Python2FileManager(ProgramsFileManager):
    associated_path = "python2/"

    def __init__(self, base_dir):
        programs_dir = os.path.join(base_dir, self.associated_path)
        super(Python2FileManager, self).__init__(progs_dir=programs_dir,
                                                 language=LanguageEnum.PYTHON2, extension=".py",
                                                 runnable=True)

        self.__process = None

    @staticmethod
    def _generate_raw_text(code, description):

        file_text = triple_quotes + "\n"
        file_text += description + "\n"
        file_text += triple_quotes + "\n"
        file_text += code + "\n"
        return file_text

    def execute(self, name):
        if not self.exists(name):
            raise FileDoesNotExistException
        try:
            self.__process = subprocess.Popen(['python', self._path_from_name(name)],
                                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            _stdout, stderr = self.__process.communicate()
        except Exception as e:
            return False, str(e)

        if stderr:
            return False, stderr

        return True, "Execution success"

    def stop_execution(self):
        if self.__process is None:
            return False, "No Process is running"
        try:
            self.__process.terminate()
        except Exception as e:
            return False, str(e)

        return True, "Execution stopped"

    def read(self, name):
        raw_file = self._read_raw_file(name)

        # Check if description
        if not raw_file.startswith(triple_quotes):
            return raw_file, ""
        # Find end of description
        index_second_quotes = raw_file.find(triple_quotes, len_triple_quotes)
        # Cut description
        description = raw_file[len_triple_quotes + len_line_jump:index_second_quotes]
        if description.endswith(line_jump):
            description = description[:-len_line_jump]

        # The remaining text is code
        code = raw_file[index_second_quotes + len_triple_quotes + len_line_jump:]

        return code, description
