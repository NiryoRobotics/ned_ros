from typing import Callable


class WrapperAddress:

    def __init__(self, read: Callable, write: Callable = None):
        self.read = read
        self.write = write

    @classmethod
    def dynamic_addressing(cls, offset: int, count: int, read: Callable, write: Callable = None, step=1):
        """
        Generate a dictionary for dynamically addressing a callback with varying addresses.

        :param offset: The starting address value.
        :type offset: int
        :param count: The number of callback-address pairs to generate.
        :type count: int
        :param read: The read callback function to associate with each address.
                     It must take in parameter an int representing the current index - offset within the range
        :type read: (int) -> bool
        :param write: The write callback function to associate with each address.
                      *args and **kwargs will be forwarded to it when called
        :type write: Callable
        :param step: The step size between consecutive addresses. Defaults to 1.
        :type step: int, optional

        :returns: A dictionary where keys are integer addresses and values are WrapperAddress instances
                  associated with those addresses.
        :rtype: Dict[int, Callable]
        """
        wrapper_addresses = {}
        for address in range(offset, (step * count) + offset, step):
            wrapper_addresses[address] = cls(lambda ix=address - offset: read(ix),
                                             (lambda *args, ix=address - offset, **kwargs: write(ix, *args, **kwargs)))
        return wrapper_addresses
