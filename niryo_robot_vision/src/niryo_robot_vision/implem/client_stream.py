from .abstract_stream import AbstractStream


class Stream(AbstractStream):

    def __init__(self):
        ...

    @property
    def is_active(self) -> bool:
        return False
