#!/usr/bin/env python
import os
import subprocess
import time
from threading import Lock


class ProgramFileException(Exception):
    pass


class FileAlreadyExistException(ProgramFileException):
    pass


class FileDoesNotExistException(ProgramFileException):
    pass


class FileNotRunnableException(ProgramFileException):
    pass


class ExecutionException(ProgramFileException):
    pass


class ProgramsFileManager(object):

    def __init__(self, programs_dir, extension, runnable=False, bin_path=''):
        self._programs_dir = os.path.abspath(os.path.expanduser(programs_dir)) + "/"
        if not os.path.isdir(self._programs_dir):
            os.makedirs(self._programs_dir)

        self._runnable = runnable
        self.__bin_path = bin_path

        self.__process = None
        self.__execution_lock = Lock()

        self._extension, self._suffix = self._init_extension_n_suffix(extension)

    def _init_extension_n_suffix(self, extension):
        if extension[0] == ".":
            return extension[1:], extension
        return extension, "." + self._extension

    def _name_from_filename(self, filename):
        if filename.endswith(self._suffix):
            return filename[:-len(self._suffix)]
        return filename

    def _filename_from_name(self, name):
        return name + self._suffix

    def _path_from_name(self, name):
        return self._programs_dir + self._filename_from_name(name)

    def read(self, name):
        """
        Read file

        :param name: file name
        :type name: str

        :raises: ProgramFileException: if any error

        :return: The file
        :rtype: str
        """
        if not self.exists(name):
            raise ProgramFileException("File '{}' does not exist".format(name))

        with open(self._path_from_name(name), 'r') as f:
            try:
                return f.read()
            except Exception as e:
                raise ProgramFileException("Could not read object '{}' : {}".format(name, e))

    @staticmethod
    def _generate_raw_text(code):
        raise NotImplementedError

    # - Public
    def create(self, name, code):

        if len(name) == 0:
            name = 'untitled'

        # Getting path
        file_path = self._path_from_name(name)

        # Generating lines which should be written
        file_lines = code.split('\\n') + []
        with open(file_path, 'w') as f:
            try:
                for file_line in file_lines:
                    f.write(file_line + "\n")
            except Exception as e:
                raise ProgramFileException("Could not write program" + str(e))

    def get_saved_at(self, name):
        """
        Return the modification time of the file
        :param name: the program's file name
        :type name: str
        :return: the modification time of the file
        :rtype: str
        """
        if not self.exists(name):
            raise ProgramFileException("File '{}' does not exist".format(name))

        return time.ctime(os.path.getmtime(self._path_from_name(name)))

    def remove(self, name):
        """
        Remove file

        :param name: file name
        :type name: str
        :raises: ProgramFileException: if any error

        :return: None
        """
        try:
            os.remove(self._path_from_name(name))
        except OSError as e:
            raise ProgramFileException("Could not remove object '{}' : {}".format(name, e))

    def get_all_names(self, with_suffix=False):
        """
        Get all filenames available in storage

        :param with_suffix:
        :raises: ProgramFileException: if any error

        :return: list of filenames
        :rtype: list[str]
        """
        try:
            filenames = sorted(os.listdir(self._programs_dir))
        except OSError as e:
            raise ProgramFileException("Could not retrieve files. " + str(e))
        if with_suffix:
            return [f for f in filenames if f.endswith(self._suffix)]
        else:
            return [self._name_from_filename(f) for f in filenames if f.endswith(self._suffix)]

    def get_all_names_with_subdirectory(self):
        list_ = self.get_all_names(with_suffix=True)
        return [self.__bin_path + name for name in list_]

    def exists(self, name):
        """
        Check if a file with a certain name exists

        :param name: file name
        :type name: str
        :return: True if file exists, else False
        :rtype: bool
        """
        return os.path.isfile(self._path_from_name(name))

    @property
    def runnable(self):
        return self._runnable

    @property
    def is_running(self):
        return self.__execution_lock.locked()

    def execute_from_path(self, path, output_stream):
        with self.__execution_lock:
            try:
                self.__process = subprocess.Popen([self.__bin_path, path],
                                                  stdout=subprocess.PIPE,
                                                  stderr=subprocess.PIPE,
                                                  encoding='utf-8')
                while True:
                    output = self.__process.stdout.read(1)
                    if output == '' and self.__process.poll() is not None:
                        break
                    if output != '':
                        output_stream.put(output)
            except Exception as e:
                raise ExecutionException(str(e))

    def execute_from_name(self, name, output_stream):
        if not self.exists(name):
            raise FileDoesNotExistException
        if not self.runnable:
            raise FileNotRunnableException

        return self.execute_from_path(self._path_from_name(name), output_stream)

    def stop_execution(self):
        if self.__process is None:
            return

        try:
            self.__process.terminate()
        except Exception as e:
            raise ProgramFileException(str(e))
