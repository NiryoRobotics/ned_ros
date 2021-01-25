TCP_PORT = 40001  # Port used for TCP communication
READ_SIZE = 512  # Buffer used for receiving TCP packet

# - Infos on the JSON size
# nbr_bytes : number of bytes on which the size is coded
# type : size coding type
DEFAULT_PACKET_SIZE_INFOS = {
    "nbr_bytes": 2,
    "type": '@H',
}
