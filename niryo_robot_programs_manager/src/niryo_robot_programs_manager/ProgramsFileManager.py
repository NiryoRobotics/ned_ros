#!/usr/bin/env python
import os
from typing import List


class ProgramFileException(Exception):
    """Base class for exceptions related to program files."""


class FileAlreadyExistException(ProgramFileException):
    """Exception raised when attempting to create a file that already exists."""


class FileDoesNotExistException(ProgramFileException):
    """Exception raised when attempting to access a file that does not exist."""


class FileNotRunnableException(ProgramFileException):
    """Exception raised when attempting to run a file that is not runnable."""


class ProgramsFileManager(object):
    """
    A manager for handling program files.

    This class provides methods to create, read, update, and delete program files.

    :param programs_dir: The directory where program files are stored.
    :type programs_dir: str
    :param extension: The file extension for program files.
    :type extension: str
    """

    def __init__(self, programs_dir: str, extension: str) -> None:
        """
        Initialize the ProgramsFileManager.

        :param programs_dir: The directory where program files are stored.
        :type programs_dir: str
        :param extension: The file extension for program files.
        :type extension: str
        """
        self.__programs_dir: str = os.path.abspath(os.path.expanduser(programs_dir)) + "/"
        if not os.path.isdir(self.__programs_dir):
            os.makedirs(self.__programs_dir)

        self.__extension: str
        self.__suffix: str
        self.__init_extension_n__suffix(extension)

    def __init_extension_n__suffix(self, extension: str) -> None:
        """
        Initialize the file extension and suffix.

        :param extension: The file extension for program files.
        :type extension: str
        """
        if extension[0] == ".":
            self.__extension = extension[1:]
            self.__suffix = extension
        else:
            self.__extension = extension
            self.__suffix = "." + extension

    def _name_from_filename(self, filename: str) -> str:
        """
        Extract the file name from the given filename.

        :param filename: The full filename including extension.
        :type filename: str
        :return: The extracted file name.
        :rtype: str
        """
        if filename.endswith(self.__suffix):
            return filename[:-len(self.__suffix)]
        return filename

    def _filename_from_name(self, name: str) -> str:
        """
        Generate the full filename from the given file name.

        :param name: The file name without extension.
        :type name: str
        :return: The full filename with extension.
        :rtype: str
        """
        return name + self.__suffix

    def path_from_name(self, name: str) -> str:
        """
        Get the full file path from the given file name.

        :param name: The file name without extension.
        :type name: str
        :return: The full file path.
        :rtype: str
        """
        return self.__programs_dir + self._filename_from_name(name)

    def read(self, name: str) -> str:
        """
        Read the content of the specified file.

        :param name: The name of the file to read.
        :type name: str
        :raises: ProgramFileException: If the file does not exist or cannot be read.
        :return: The content of the file.
        :rtype: str
        """
        if not self.exists(name):
            raise FileDoesNotExistException(f"File '{name}' does not exist")

        with open(self.path_from_name(name), 'r') as f:
            try:
                return f.read()
            except Exception as e:
                raise ProgramFileException(f"Could not read object '{name}': {e}")

    # - Public
    def create(self, name: str, code: str) -> None:
        """
        Create a new file with the given name and content.

        :param name: The name of the file to create.
        :type name: str
        :param code: The content of the file.
        :type code: str
        :raises: ProgramFileException: If the file cannot be created or written.
        """
        if len(name) == 0:
            name = 'untitled'

        # Getting path
        file_path = self.path_from_name(name)

        # Generating lines which should be written
        file_lines = code.split('\n') + []
        with open(file_path, 'w') as f:
            try:
                for file_line in file_lines:
                    f.write(file_line + "\n")
            except Exception as e:
                raise ProgramFileException("Could not write program" + str(e))

    def remove(self, name: str) -> None:
        """
        Remove the specified file.

        :param name: The name of the file to remove.
        :type name: str
        :raises: ProgramFileException: If the file cannot be removed.
        """
        try:
            os.remove(self.path_from_name(name))
        except OSError as e:
            raise ProgramFileException(f"Could not remove object '{name}': {e}")

    def get_all_names(self, with_suffix: bool = False) -> List[str]:
        """
        Get a list of all file names available in the storage.

        :param with_suffix: If True, include file extensions in the names.
        :type with_suffix: bool
        :raises: ProgramFileException: If the list of files cannot be retrieved.
        :return: A list of file names.
        :rtype: list[str]
        """
        try:
            filenames = sorted(os.listdir(self.__programs_dir))
        except OSError as e:
            raise ProgramFileException(f"Could not retrieve files: {e}")
        if with_suffix:
            return [f for f in filenames if f.endswith(self.__suffix)]
        else:
            return [self._name_from_filename(f) for f in filenames if f.endswith(self.__suffix)]

    def exists(self, name: str) -> bool:
        """
        Check if a file with a certain name exists.

        :param name: The name of the file to check.
        :type name: str
        :return: True if the file exists, else False.
        :rtype: bool
        """
        return os.path.isfile(self.path_from_name(name))
