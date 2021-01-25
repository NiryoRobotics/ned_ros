#!/usr/bin/env python

import os
import json


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
    def __name_from_filename(self, filename):
        if filename.endswith(self._suffix):
            return filename[:-len(self._suffix)]
        return filename

    def __filename_from_name(self, name):
        return name + self._suffix

    def __path_from_name(self, name):
        return self._base_dir + self.__filename_from_name(name)

    def _write(self, name, object_):
        if name in self._protected_names:
            raise NiryoRobotFileException("Object '{}' is protected and cannot be written".format(name))
        with open(self.__path_from_name(name), 'w') as f:
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

        with open(self.__path_from_name(name), 'r') as f:
            try:
                return self.object_type.from_dict(json.loads(f.read()))
            except Exception as e:
                raise NiryoRobotFileException("Could not read object '{}' : {}".format(name, e))

    def read_description(self, name):
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
            os.remove(self.__path_from_name(name))
        except OSError as e:
            raise NiryoRobotFileException("Could not remove object '{}' : {}".format(name, e))

    def get_all_names(self):
        """
        Get all filenames available in storage

        :raises: NiryoRobotFileException: if any error

        :return: list of filenames
        :rtype: list[str]
        """
        try:
            filenames = sorted(os.listdir(self._base_dir))
        except OSError as e:
            raise NiryoRobotFileException("Could not retrieve files. " + str(e))
        return [self.__name_from_filename(f) for f in filenames if f.endswith(self._suffix)]

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
        return os.path.isfile(self.__path_from_name(name))
