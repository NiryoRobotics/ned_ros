#!/usr/bin/env python

import rospy

from client_jog_interface_example import JogClient

from niryo_robot_arm_commander.srv import JogShiftRequest

from pynput.mouse import Button, Listener, Controller


class MyMouseListener:

    def __init__(self):
        self.clicked = False

        self.last_x_send = None
        self.actual_x = None

        self.last_y_send = None
        self.actual_y = None

        self.last_z_send = 0
        self.actual_z = 0

        self.mouse = Controller()

        self.listener = Listener(
            on_move=self.on_move,
            on_click=self.on_click,
            on_scroll=self.on_scroll).start()

    def __del__(self):
        self.listener.stop()

    def on_move(self, x, y):
        if self.last_x_send is None:
            self.last_x_send = x
            self.last_y_send = y
        self.actual_x = x
        self.actual_y = y

    def on_click(self, _x, _y, button, pressed):
        if not pressed:
            # self.mouse.release(button)
            if button == Button.right:
                self.clicked = True
            elif button == Button.left:
                self.actual_x = 960
                self.actual_y = 1500
                self.mouse.position = (self.actual_x, self.actual_y)
                self.actual_z = 0
                self.reset()

    def on_scroll(self, _x, _y, _dx, dy):
        self.actual_z += dy

    def reset(self):
        self.last_x_send = self.actual_x
        self.last_y_send = self.actual_y
        self.last_z_send = self.actual_z

    def get_x_diff(self):
        if self.last_x_send is None:
            return 0
        return self.actual_x - self.last_x_send

    def get_y_diff(self):
        if self.last_y_send is None:
            return 0
        return self.actual_y - self.last_y_send

    def get_z_diff(self):
        return self.actual_z - self.last_z_send

    def __str__(self):
        return ''


if __name__ == "__main__":
    import time

    # Creating Client Object
    jc = JogClient()
    jc.set_jog(True)

    mouse_listener = MyMouseListener()

    actualization_rate = rospy.Rate(10)
    while not mouse_listener.clicked:
        init = time.time()
        mouse_dx = mouse_listener.get_x_diff()
        mouse_dy = mouse_listener.get_y_diff()
        mouse_dz = mouse_listener.get_z_diff()
        mouse_listener.reset()
        robot_dx = - float(mouse_dy) * 1e-4
        robot_dy = - float(mouse_dx) * 1e-4
        robot_dz = float(mouse_dz) * 5e-3
        jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                             shift_values=[robot_dx, robot_dy, robot_dz, 0.0, 0.0, 0.0])

        actualization_rate.sleep()
