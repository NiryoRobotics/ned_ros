import os

from niryo_robot_programs_manager.ProgramsFileManager import ProgramsFileManager
from niryo_robot_programs_manager.programs_manager_enums import LanguageEnum


class BlocklyManager(ProgramsFileManager):
    associated_path = "blockly/"

    def __init__(self, base_dir):
        programs_dir = os.path.join(base_dir, self.associated_path)
        super(BlocklyManager, self).__init__(progs_dir=programs_dir,
                                             language=LanguageEnum.BLOCKLY, extension=".xml",
                                             runnable=False)

    @staticmethod
    def _generate_raw_text(code, description):
        return code

    def read(self, name):
        return self._read_raw_file(name), ""
