from typing import Callable, Dict, Tuple


def dynamic_addressing(offset: int, count: int, callback: Callable, step=1) -> Dict[int, Callable]:
    """
        Generate a dictionary for dynamically addressing a series of callbacks with varying addresses.

        :param offset: The starting address value.
        :type offset: int
        :param count: The number of callback-address pairs to generate.
        :type count: int
        :param callback: The callback function to associate with each address.
        :type callback: Callable
        :param step: The step size between consecutive addresses. Defaults to 1.
        :type step: int, optional

        :return: A dictionary where keys are integer addresses and values are callback functions associated with those addresses.
        :rtype: Dict[int, Callable]
    """
    addressing = {}
    for address in range(offset, (step * count) + offset, step):
        addressing[address] = lambda ix=address - offset: callback(ix)
    return addressing


def dynamic_addressing_w_write(offset: int,
                               count: int,
                               callbacks: Tuple[Callable, Callable],
                               step=1) -> Dict[int, Tuple[Callable, Callable]]:
    addressing = dynamic_addressing(offset, count, callbacks[0], step)
    for address, read_callback in addressing.items():
        write_callback = lambda *args, ix=address - offset, **kwargs: callbacks[1](ix, *args, **kwargs)
        addressing[address] = (read_callback, write_callback)
    return addressing
