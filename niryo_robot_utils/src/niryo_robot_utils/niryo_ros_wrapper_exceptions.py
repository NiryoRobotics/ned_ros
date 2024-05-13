#!/usr/bin/env python

# Improved version, to simply know the status code and message
class NiryoRosWrapperException(Exception):
    def __init__(self, *args: object, status=None, message=None) -> None:
        super().__init__(*args)
        self.status = status
        self.message = message
