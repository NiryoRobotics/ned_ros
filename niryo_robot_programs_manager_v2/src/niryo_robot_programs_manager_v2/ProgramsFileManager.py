from contextlib import contextmanager
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import List, Generator


class ProgramFileException(Exception):
    """Base class for exceptions related to program files."""


class FileAlreadyExistException(ProgramFileException):
    """Exception raised when attempting to create a file that already exists."""


class FileDoesNotExistException(ProgramFileException):
    """Exception raised when attempting to access a file that does not exist."""


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
        self.__programs_dir: Path = Path(programs_dir).expanduser()
        if not self.__programs_dir.is_dir():
            self.__programs_dir.mkdir()

        self.__extension: str = extension

    def _path_from_name(self, name: str, check_exists=True) -> Path:
        """
        Get the full file path from the given file name.

        :param name: The file name without extension.
        :type name: str
        :param check_exists: if True, check if the generated path is a file
        :type check_exists: bool
        :return: The full file path.
        :rtype: str
        """
        file_path = self.__programs_dir.joinpath(name).with_suffix(self.__extension)
        if check_exists and not file_path.is_file():
            raise FileDoesNotExistException(f'File with name "{name}" does not exist')
        return file_path

    def read(self, name: str) -> str:
        """
        Read the content of the specified file.

        :param name: The name of the file to read.
        :type name: str
        :raises: ProgramFileException: If the file does not exist or cannot be read.
        :return: The content of the file.
        :rtype: str
        """
        file_path = self._path_from_name(name)

        try:
            return file_path.read_text()
        except Exception as e:
            raise ProgramFileException(f"Could not read object '{name}': {e}")

    def write(self, name: str, code: str, overwrite_allowed: bool = False) -> None:
        """
        Write the content of `code` inside a file named `name`

        :param name: The name of the file to write to.
        :type name: str
        :param code: The content of the file.
        :type code: str
        :param overwrite_allowed: If True and a file already exist with this name, it will be overwritten
        :type overwrite_allowed: bool
        :raises: ProgramFileException: If the file cannot be created or written.
        """
        if len(name) < 1:
            raise ProgramFileException('Name cannot be empty')

        if self.exists(name) and not overwrite_allowed:
            raise FileAlreadyExistException(f'File "{name}" already exist')

        file_path = self._path_from_name(name, check_exists=False)
        try:
            file_path.write_text(code)
        except Exception as e:
            raise ProgramFileException("Could not write program" + str(e))

    def remove(self, name: str) -> None:
        """
        Remove the specified file.

        :param name: The name of the file to remove.
        :type name: str
        :raises: ProgramFileException: If the file cannot be removed.
        """
        file_path = self._path_from_name(name)
        try:
            file_path.unlink()
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
        return [f if with_suffix else f.stem for f in self.__programs_dir.iterdir()]

    def exists(self, name: str) -> bool:
        """
        Check if a file with a certain name exists.

        :param name: The name of the file to check.
        :type name: str
        :return: True if the file exists, else False.
        :rtype: bool
        """
        try:
            self._path_from_name(name)
        except FileDoesNotExistException:
            return False
        return True

    def get_file_path(self, name: str) -> str:
        return str(self._path_from_name(name))

    @contextmanager
    def temporary_file(self, code: str) -> Generator[str, None, None]:
        with NamedTemporaryFile(suffix=self.__extension, mode='w') as program_file:
            program_file.write(code)
            program_file.flush()
            yield program_file.name
