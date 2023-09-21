from typing import List


def address_range(n_addresses: int, offset: int = 0) -> List[int]:
    """
    Generate a list of addresses in a specified range.

    Args:
        n_addresses (int): The number of addresses to generate.
        offset (int, optional): The starting offset for the address range. Default is 0.

    Returns:
        List[int]: A list of consecutive addresses starting from `offset` up to `n_addresses + offset - 1`.

    Example:
        >>> address_range(5)
        [0, 1, 2, 3, 4]
        >>> address_range(3, 10)
        [10, 11, 12]
    """
    return [*range(offset, n_addresses + offset)]
