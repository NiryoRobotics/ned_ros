"""
File containing functions for communication with Socket !
"""

import sys
import ast
import json
import socket
import struct

from .const_communication import READ_SIZE, DEFAULT_PACKET_SIZE_INFOS


# --- RECEPTION -- #
def receive_data(sckt, packet_size_infos, buffer_size):
    """
    Receive msg which cannot be contained in only one buffer

    :param sckt: the socket
    :param packet_size_infos: Format de l'objet du message : permet de savoir sur cb de bytes est codee la taille
    :param buffer_size: buffer size for reading socket's buffer
    :return: Bytes corresponding to received packet !
    """
    nbr_bytes = packet_size_infos["nbr_bytes"]
    received_data = sckt.recv(nbr_bytes)
    if len(received_data) != nbr_bytes:
        return None
    size_packet = int(struct.unpack(packet_size_infos["type"], received_data)[0])
    received_big_data = "" if sys.version_info[0] == 2 else b""

    while len(received_big_data) < size_packet:
        received_big_data += sckt.recv(min(buffer_size, size_packet - len(received_big_data)))

    return received_big_data if sys.version_info[0] == 2 else received_big_data.decode()


def receive_dict(sckt, packet_size_infos=DEFAULT_PACKET_SIZE_INFOS, buffer_size=READ_SIZE):
    """
    Receive json through a Socket

    :param sckt: the socket
    :param packet_size_infos: Format de l'objet du message : permet de savoir sur cb de bytes est codee la taille
    :param buffer_size: buffer size for reading socket's buffer
    :return:
    """
    msg = receive_data(sckt, buffer_size=buffer_size, packet_size_infos=packet_size_infos)
    return data_to_dict(msg)


def receive_dict_w_payload(sckt, packet_size_infos=DEFAULT_PACKET_SIZE_INFOS, buffer_size=READ_SIZE):
    """
    Receive json through a Socket then extract payload content.
    Payload can typically be an heavy file like an image

    :param sckt: the socket
    :param packet_size_infos: Format de l'objet du message : permet de savoir sur cb de bytes est codee la taille
    :param buffer_size: buffer size for reading socket's buffer
    :return: dict of packet's JSON, payload
    """
    dict_data = receive_dict(sckt, buffer_size=buffer_size, packet_size_infos=packet_size_infos)

    payload_size = dict_data["payload_size"]
    received_payload = "" if sys.version_info[0] == 2 else b""
    while len(received_payload) < payload_size:
        received_payload += sckt.recv(buffer_size)

    return dict_data, received_payload


# - JSON - #
def data_to_dict(data):
    """
    Convert a string representing a JSON to a JSON

    :param data: String representing a JSON
    :return: dict of this JSON
    """
    if data is None:
        return None
    # if type(data) == bytes:
    #     data = data.decode()
    return ast.literal_eval(data)


def dict_to_packet(dict_obj, packet_size_infos=DEFAULT_PACKET_SIZE_INFOS):
    """
    Convert dict to packet

    :type dict_obj: dict
    :param packet_size_infos:
    :return: packet
    """
    json_obj = json.dumps(dict_obj)

    if sys.version[0] == 3:
        json_obj = json_obj.encode()
    packet = "" if sys.version_info[0] == 2 else b""
    packet += struct.pack(packet_size_infos["type"], len(json_obj))
    packet += json_obj
    return packet


# # --- SOCKET HANDLING --- #

def create_socket_server(ip_address, port):
    """
    Everything in the title

    :param ip_address: IP Address
    :param port: Port
    :return: Socket object
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip_address, port))
    return s
