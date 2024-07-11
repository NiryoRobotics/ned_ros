import subprocess
from threading import Lock, Event
from typing import Optional

import rospy


class ExecutionException(Exception):
    """Exception raised for errors during program execution."""


class PythonRunner:
    """
    A class for running programs asynchronously.

    This class provides methods to start and stop python programs
    It captures the program's standard output and provides methods to check the
    execution status and retrieve the program output.
    """
    def __init__(self) -> None:
        """
        Initialize the ProgramRunner.

        """
        self.__output: str = ''
        self.__process: Optional[subprocess.Popen] = None
        self.__execution_lock: Lock = Lock()

    @property
    def output(self) -> str:
        """
        Get the standard output of the executed program.

        :return: The standard output of the program.
        :rtype: str
        """
        return self.__output

    @property
    def is_running(self) -> bool:
        """
        Check if a program is currently running.

        :return: True if a program is running, else False.
        :rtype: bool
        """
        return self.__execution_lock.locked()

    @property
    def exit_status(self) -> Optional[int]:
        """
        Get the exit status of the executed program.

        :return: The exit status of the program if it has exited, else None.
        :rtype: Optional[int]
        """
        if self.__process is None:
            return None
        return self.__process.poll()

    def start(self, program_path: str, execution_started_event: Event = None) -> None:
        """
        Start the specified program.

        :param program_path: The path to the program file to be executed.
        :type program_path: str
        :param execution_started_event: An event which notify when the execution has started
        :type execution_started_event: Event
        :raises: ExecutionException: If an error occurs during program execution.
        """
        with self.__execution_lock:
            if execution_started_event is not None:
                execution_started_event.set()
            self.__output = ''
            try:
                self.__process = subprocess.Popen(['python3', '-u', program_path],
                                                  stdout=subprocess.PIPE,
                                                  stderr=subprocess.STDOUT,
                                                  encoding='utf-8')
                while True:
                    output = self.__process.stdout.read(1)
                    # Break the loop if the process ended and all the stdout has been read
                    if self.__process.poll() is not None and output == '':
                        break
                    self.__output += output
            except Exception as e:
                rospy.logerr(str(e))
                raise ExecutionException(str(e))

    def stop(self) -> None:
        """
        Stop the currently running program.

        :raises: ExecutionException: If an error occurs while stopping the program.
        """
        if self.__process is None:
            return

        try:
            self.__process.terminate()
        except Exception as e:
            rospy.logerr(str(e))

        # Wait for the process to terminate
        WAIT_TIMEOUT_SECONDS = 3
        try:
            self.__process.wait(WAIT_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            # Kill the process tree
            self.__process.kill()
