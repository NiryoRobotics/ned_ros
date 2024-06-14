READ_SIZE = 512  # Buffer used for receiving TCP packet

# - Infos on the JSON size
# nbr_bytes : number of bytes on which the size is coded
# type : size coding type
DEFAULT_PACKET_SIZE_INFOS = {
    "nbr_bytes": 2,
    "type": '@H',
}

minimum_client_version = '1.2.0'

HANDSHAKE_MSG_CLIENT_OUTDATED = (
    f"You're using an outdated version of PyNiryo. "
    f"The minimum client version is v{minimum_client_version}. "
    f"To fully benefit from the server features it's advised to upgrade your PyNiryo library.")
