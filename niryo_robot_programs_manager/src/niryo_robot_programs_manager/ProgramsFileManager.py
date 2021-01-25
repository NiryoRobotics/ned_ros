#!/usr/bin/env python
import os


class ProgramFileException(Exception):
    pass


class FileAlreadyExistException(ProgramFileException):
    pass


class FileDoesNotExistException(ProgramFileException):
    pass


class ProgramsFileManager(object):
    associated_path = ""

    def __init__(self, progs_dir, language, extension, runnable):
        self._progs_dir = os.path.abspath(os.path.expanduser(progs_dir)) + "/"
        if not os.path.isdir(self._progs_dir):
            os.makedirs(self._progs_dir)

        self._language = language
        self._runnable = runnable

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
        return self._progs_dir + self._filename_from_name(name)

    def _read_raw_file(self, name):
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
    def _generate_raw_text(code, description):
        raise NotImplementedError

    # - Public
    def create(self, name, code, description, allow_overwrite):
        # Getting path
        file_path = self._path_from_name(name)

        if not allow_overwrite and os.path.isfile(file_path):
            raise FileAlreadyExistException("File already exist and overwrite permission is not given")

        # Generate raw string
        file_raw_text = self._generate_raw_text(code, description)
        # Generating lines which should be written
        file_lines = file_raw_text.split('\\n')
        with open(file_path, 'w') as f:
            try:
                for file_line in file_lines:
                    f.write(file_line + "\n")
            except Exception as e:
                raise ProgramFileException("Could not write program" + str(e))

    def read(self, name):
        """
        Read

        :param name:
        :type name: str
        :return: Code, description
        :rtype: (str, str)
        """
        raise NotImplementedError

    def read_description(self, name):
        """
        Read description
        """
        return self.read(name)[1]

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
            filenames = sorted(os.listdir(self._progs_dir))
        except OSError as e:
            raise ProgramFileException("Could not retrieve files. " + str(e))
        if with_suffix:
            return [f for f in filenames if f.endswith(self._suffix)]
        else:
            return [self._name_from_filename(f) for f in filenames if f.endswith(self._suffix)]

    def get_all_names_with_description(self):
        list_name = self.get_all_names()
        list_description = [self.read_description(name=n) for n in list_name]
        return list_name, list_description

    def get_all_names_with_subdirectory(self):
        list_ = self.get_all_names(with_suffix=True)
        return [self.associated_path + name for name in list_]

    def exists(self, name):
        """
        Check if a file with a certain name exists

        :param name: file name
        :type name: str
        :return: True if file exists, else False
        :rtype: bool
        """
        return os.path.isfile(self._path_from_name(name))

    def execute(self, name):
        if self._runnable:
            return False, "Execution not implemented"
        else:
            return False, "Execution not runnable"

    def stop_execution(self):
        if self._runnable:
            return False, "Stop execution not implemented"
        else:
            return False, "Execution not runnable, why do you want to stop it ?"

    @property
    def runnable(self):
        return self._runnable
