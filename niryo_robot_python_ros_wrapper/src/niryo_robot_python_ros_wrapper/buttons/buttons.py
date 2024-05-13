from ._base import BaseButtonRosWrapper


class CustomButtonRosWrapper(BaseButtonRosWrapper):
    def __init__(self, hardware_version='ned2'):
        super().__init__("custom", hardware_version)


class FreeMotionButtonRosWrapper(BaseButtonRosWrapper):
    def __init__(self, hardware_version='ned2'):
        super().__init__("free_drive", hardware_version)


class SaveButtonRosWrapper(BaseButtonRosWrapper):
    def __init__(self, hardware_version='ned2'):
        super().__init__("save_pos", hardware_version)
