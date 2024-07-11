#!/usr/bin/env python

import rospy

# Communication imports
import select
import socket

from .communication_functions import create_socket_server, receive_dict, dict_to_packet
from .command_interpreter import CommandInterpreter

# Threading
from threading import Thread, Lock
import queue


class TcpServer:

    def __init__(self, ip_address='', port=40001, on_client_connection_cb=None, on_client_disconnection_cb=None):
        # Server TCP
        self.__ip_address = ip_address
        self.__port = port

        try:
            self.__server = create_socket_server(ip_address='', port=self.__port)
            self.__server.listen(1)
            self.__client = None
            self.__is_client_connected = False
            self.__on_client_connection_cb = on_client_connection_cb
            self.__on_client_disconnection_cb = on_client_disconnection_cb

            # Thread
            self.__loop_thread = Thread(target=self.__loop, name="TCP Server Loop thread")
            self.__command_executor_thread = Thread(target=self.__command_executor_loop,
                                                    name="TCP Server Command Executor thread")
            self.__is_running = True
            self.__is_busy_lock = Lock()
            # Interpreter
            self.__interpreter = CommandInterpreter()
            self.__queue = queue.Queue(1)

        except socket.error as err:
            rospy.logerr("UserInterface.init : TCP server unable to start : %s", err)
            self.__server = None

    def __del__(self):
        self.quit()

    def start(self):
        if self.__server is not None:
            self.__loop_thread.start()
            self.__command_executor_thread.start()
            return self
        else:
            return None

    def quit(self):
        self.__is_running = False
        if self.__loop_thread.is_alive():
            self.__loop_thread.join()
        if self.__command_executor_thread.is_alive():
            self.__command_executor_thread.join()
        if self.__client is not None:
            self.__shutdown_client()
        if self.__server is not None:
            self.__server.close()

    def __loop(self):
        rospy.loginfo("TCP Server - Started on port {}".format(self.__port))
        inputs = [self.__server]
        while self.__is_running is True:
            readable, writable, exceptional = select.select(inputs, [], [], 0.5)
            for s in readable:
                if self.__is_client_connected is False and s is self.__server:
                    self.__accept_client()
                    inputs.append(self.__client)
                elif s is not self.__server:
                    self.__client_socket_event(inputs)

    # -- CLIENTS

    def __accept_client(self):
        self.__client, address = self.__server.accept()
        self.__is_client_connected = True
        rospy.loginfo("TCP Server - Client connected from IP address: {}/{}".format(address[0], address[1]))
        if self.__on_client_connection_cb is not None:
            self.__on_client_connection_cb()

    def __client_socket_event(self, inputs):
        dict_command_received = self.__read_command()
        if dict_command_received is not None:
            with self.__is_busy_lock:
                self.__queue.put(dict_command_received)
        else:
            rospy.loginfo("TCP Server - Client disconnect")
            self.__is_client_connected = False
            self.__shutdown_client()
            inputs.remove(self.__client)
            if self.__on_client_disconnection_cb is not None:
                self.__on_client_disconnection_cb()

    def __shutdown_client(self):
        if self.__client is not None:
            try:
                self.__client.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            self.__client.close()

    # -- COMMANDS
    # LOOP which execute commands in queue
    def __command_executor_loop(self):
        while self.__is_running is True:
            try:
                dict_command_received = self.__queue.get(block=True, timeout=0.5)
                with self.__is_busy_lock:
                    self.__treat_command(dict_command_received)
            except queue.Empty:
                pass

    def __read_command(self):
        try:
            received_dict = receive_dict(sckt=self.__client)
        except (socket.error, ValueError) as e:
            rospy.loginfo("TCP Server - Error while receiving answer: {}".format(e))
            return None
        if not received_dict:
            return None
        return received_dict

    def __treat_command(self, dict_command_received):
        try:
            result_data = self.__interpreter.interpret_command(dict_command_received)
        except Exception as e:
            rospy.logerr("tcp server - {}".format(e), exc_info=True)
            command_name = dict_command_received["command"]
            self.__answer_error(command_name, e)
        else:
            self.__send(result_data)

    # -- ANSWER AND SEND

    def __answer_error(self, command_name, error):
        str_err = "An error occured while executing the last command : {}".format(error)
        dict_error = self.__interpreter.generate_dict_failure(command_name, str_err)
        packet_error = dict_to_packet(dict_error)
        self.__send(packet_error)

    def __send(self, result_data: bytes):
        if self.__client is not None:
            try:
                self.__client.sendall(result_data)
            except socket.error as e:
                rospy.loginfo("TCP Server - Error while sending answer to client: {}".format(e))
