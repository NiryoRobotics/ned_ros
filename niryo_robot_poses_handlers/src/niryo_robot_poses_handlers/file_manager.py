#!/usr/bin/env python

import os
import json
import pickle


class NiryoRobotFileException(Exception):
    pass


class FileManager(object):
    """
    Manages the creation, storage and loading of objects that implement the functions
    from_dict and to_dict.

    !!! This class is an abstract class

    :raises NiryoRobotFileException:

    """
    object_type = None

    def __init__(self, base_dir, extension, protected_names=None):
        self._base_dir = os.path.expanduser(base_dir)
        self._protected_names = protected_names if protected_names is not None else []
        if not self._base_dir.endswith('/'):
            self._base_dir += '/'
        if not os.path.exists(self._base_dir):
            os.makedirs(self._base_dir)

        self._extension = extension
        self._suffix = "." + self._extension

    # - Private
    def _name_from_filename(self, filename):
        if filename.endswith(self._suffix):
            return filename[:-len(self._suffix)]
        return filename

    def _filename_from_name(self, name):
        return name + self._suffix

    def _path_from_name(self, name):
        return self._base_dir + self._filename_from_name(name)

    def _write(self, name, object_):
        if name in self._protected_names:
            raise NiryoRobotFileException("Object '{}' is protected and cannot be written".format(name))
        with open(self._path_from_name(name), 'w') as f:
            try:
                f.write(json.dumps(object_.to_dict(), indent=2))
            except Exception as e:
                raise NiryoRobotFileException("Could not write object. " + str(e))

    # - Public
    def read(self, name):
        """
        Read file

        :param name: file name
        :type name: str

        :raises: NiryoRobotFileException: if any error

        :return: An object corresponding to object_type
        :rtype: object_type
        """
        if not self.exists(name):
            raise NiryoRobotFileException("Object '{}' does not exist".format(name))

        with open(self._path_from_name(name), 'r') as f:
            try:
                return self.object_type.from_dict(json.loads(f.read()))
            except Exception as e:
                raise NiryoRobotFileException("Could not read object '{}' : {}".format(name, e))

    def read_description(self, name):
        if self.check_file_format(name) is False:
            return None

        obj = self.read(name)
        return obj.description

    def remove(self, name):
        """
        Remove file

        :param name: file name
        :type name: str
        :raises: NiryoRobotFileException: if any error

        :return: None
        """
        if name in self._protected_names:
            raise NiryoRobotFileException("Object '{}' is protected and cannot be removed".format(name))
        try:
            os.remove(self._path_from_name(name))
        except OSError as e:
            raise NiryoRobotFileException("Could not remove object '{}' : {}".format(name, e))

    def check_file_format(self, file):
        try:
            obj = self.read(file)
            if obj is None:
                return False
        except Exception:
            return False

        return True

    def get_all_names(self):
        """
        Get all filenames available in storage

        :raises: NiryoRobotFileException: if any error

        :return: list of filenames
        :rtype: list[str]
        """
        try:
            files_tmp = sorted(os.listdir(self._base_dir))
            file_names = []
            for file in files_tmp:
                file_name = self._name_from_filename(file)
                if self.check_file_format(file_name):
                    file_names.append(file)
        except OSError as e:
            raise NiryoRobotFileException("Could not retrieve files. " + str(e))
        return [self._name_from_filename(f) for f in file_names if f.endswith(self._suffix)]

    def get_all_names_w_description(self):
        """
        Get all filenames available in storage + their descriptions

        :return: list of filenames
        :rtype: list[str]
        """
        list_name = self.get_all_names()
        list_description = [self.read_description(name=n) for n in list_name]
        return list_name, list_description

    def exists(self, name):
        """
        Check if a file with a certain name exists

        :param name: file name
        :type name: str
        :return: True if file exists, else False
        :rtype: bool
        """
        return os.path.isfile(self._path_from_name(name))

    def check_exist(self, name):
        if not self.exists(name):
            raise NiryoRobotFileException(f"File {self._path_from_name(name)} doesnt exist")


class PickleFileManager(FileManager):
    def _write(self, name, object_):
        if name in self._protected_names:
            raise NiryoRobotFileException("Object '{}' is protected and cannot be written".format(name))
        with open(self._path_from_name(name), 'bw') as f:
            try:
                pickle.dump(object_.to_dict(), f)
            except Exception as e:
                raise NiryoRobotFileException("Could not write object. " + str(e))

    # - Public
    def read(self, name):
        """
        Read file

        :param name: file name
        :type name: str

        :raises: NiryoRobotFileException: if any error

        :return: An object corresponding to object_type
        :rtype: object_type
        """
        if not self.exists(name):
            raise NiryoRobotFileException("Object '{}' does not exist".format(name))

        with open(self._path_from_name(name), 'br') as f:
            return self.object_type.from_dict(pickle.load(f))
