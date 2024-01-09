from niryo_robot_python_ros_wrapper import ObjectShape, ObjectColor


class CommonStore:

    def __init__(self):
        self.linear_mode = False

        self.is_executing_command = False

        # vision
        self.workspace_name = ''
        self.height_offset = 0
        self.shape = ObjectShape.ANY
        self.color = ObjectColor.ANY

        self.relative_x = 0
        self.relative_y = 0
        self.relative_yaw = 0

        self.user_store = {key: 0 for key in range(100)}

    def set_linear_mode(self, linear_mode):
        self.linear_mode = linear_mode

    def set_is_executing_command(self, is_executing_command):
        self.is_executing_command = is_executing_command

    def set_workspace_name(self, workspace_name):
        self.workspace_name = workspace_name

    def set_height_offset(self, height_offset):
        self.height_offset = height_offset

    def set_shape(self, shape):
        self.shape = shape

    def set_color(self, color):
        self.color = color

    def set_relative_x(self, relative_x):
        self.relative_x = relative_x

    def set_relative_y(self, relative_y):
        self.relative_y = relative_y

    def set_relative_yaw(self, relative_yaw):
        self.relative_yaw = relative_yaw

    def set_user_store(self, ix, value):
        self.user_store[ix] = value
