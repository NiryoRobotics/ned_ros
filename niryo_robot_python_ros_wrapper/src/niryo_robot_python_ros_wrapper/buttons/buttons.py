from ._base import BaseButtonRosWrapper, ButtonRosWrapperException


class CustomButtonRosWrapperException(ButtonRosWrapperException): pass
class FreeMotionButtonRosWrapperException(ButtonRosWrapperException): pass
class SaveButtonRosWrapperException(ButtonRosWrapperException): pass


class CustomButtonRosWrapper(BaseButtonRosWrapper):
    _error_class = CustomButtonRosWrapperException
    
    def __init__(self, hardware_version='ned2'):
        super().__init__("custom", hardware_version)


class FreeMotionButtonRosWrapper(BaseButtonRosWrapper):
    _error_class = FreeMotionButtonRosWrapperException
    
    def __init__(self, hardware_version='ned2'):
        super().__init__("free_drive", hardware_version)


class SaveButtonRosWrapper(BaseButtonRosWrapper):
    _error_class = SaveButtonRosWrapperException
    
    def __init__(self, hardware_version='ned2'):
        super().__init__("save_pos", hardware_version)
