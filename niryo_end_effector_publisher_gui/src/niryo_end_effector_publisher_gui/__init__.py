import rospy

from end_effector_interface.msg import EEButtonStatus

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget


class EndEffectorPublisherGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self):
        super(EndEffectorPublisherGui, self).__init__()

        self.button_publisher = EndEffectorPublisher()

        self.setWindowTitle('End Effector Publisher')
        self.vlayout = QVBoxLayout(self)

        # font = QFont("Helvetica", 9, QFont.Bold)

        self.free_motion_button = QPushButton('Free Motion', self)
        self.free_motion_button.setCheckable(True)
        self.free_motion_button.clicked.connect(self.free_motion_button_event)
        self.vlayout.addWidget(self.free_motion_button)

        self.save_button = QPushButton('Save', self)
        self.save_button.setCheckable(True)
        self.save_button.clicked.connect(self.save_button_event)
        self.vlayout.addWidget(self.save_button)

        self.custom_button = QPushButton('Custom', self)
        self.custom_button.setCheckable(True)
        self.custom_button.clicked.connect(self.custom_button_event)
        self.vlayout.addWidget(self.custom_button)

        self.setLayout(self.vlayout)

    def save_button_event(self, _event):
        if self.save_button.isChecked():
            self.button_publisher.save_button.press()
        else:
            self.button_publisher.save_button.release()

    def custom_button_event(self, _event):
        if self.custom_button.isChecked():
            self.button_publisher.custom_button.press()
        else:
            self.button_publisher.custom_button.release()

    def free_motion_button_event(self, _event):
        if self.free_motion_button.isChecked():
            self.button_publisher.free_motion_button.press()
        else:
            self.button_publisher.free_motion_button.release()


class EndEffectorButton(object):
    STATE_TO_STR = {
        0: "HANDLE_HELD_ACTION",
        1: "LONG_PUSH_ACTION",
        2: "SINGLE_PUSH_ACTION",
        3: "DOUBLE_PUSH_ACTION",
        100: "NO_ACTION"
    }

    def __init__(self, topic_name):
        self.state = EEButtonStatus.NO_ACTION
        self.press_time = rospy.Time(0)
        self.press_cpt = 0

        self.topic_name = topic_name
        self.topic = rospy.Publisher(topic_name, EEButtonStatus, queue_size=10, latch=True)

        self.publish(EEButtonStatus.NO_ACTION)

    def press(self):
        current_time = rospy.Time.now()
        if (current_time - self.press_time).to_sec() < 0.5:
            self.press_cpt += 1
        else:
            self.press_cpt = 1
        self.press_time = current_time
        self.state = EEButtonStatus.HANDLE_HELD_ACTION

    def release(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.press_time).to_sec()
        if self.state == EEButtonStatus.LONG_PUSH_ACTION:
            self.press_cpt = 0
            self.state = EEButtonStatus.NO_ACTION
        elif elapsed_time < 0.5 and self.press_cpt == 2:
            self.state = EEButtonStatus.DOUBLE_PUSH_ACTION
            self.publish(EEButtonStatus.DOUBLE_PUSH_ACTION)
        else:
            self.state = EEButtonStatus.NO_ACTION

    def update(self):
        current_time = rospy.Time.now()
        if self.state not in [EEButtonStatus.LONG_PUSH_ACTION, EEButtonStatus.SINGLE_PUSH_ACTION] and (
                current_time - self.press_time).to_sec() > 0.5 and self.press_cpt == 1:
            if self.state == EEButtonStatus.NO_ACTION:
                self.state = EEButtonStatus.SINGLE_PUSH_ACTION
                self.publish(EEButtonStatus.SINGLE_PUSH_ACTION)
            elif (current_time - self.press_time).to_sec() > 2:
                self.state = EEButtonStatus.LONG_PUSH_ACTION
                self.publish(EEButtonStatus.LONG_PUSH_ACTION)

        if self.state in [EEButtonStatus.HANDLE_HELD_ACTION, EEButtonStatus.LONG_PUSH_ACTION]:
            self.publish(EEButtonStatus.HANDLE_HELD_ACTION)
        else:
            self.publish(EEButtonStatus.NO_ACTION)

    def publish(self, value):
        # if value not in [EEButtonStatus.NO_ACTION, EEButtonStatus.HANDLE_HELD_ACTION]:
        #     print(self.topic_name.split('/')[-1], self.STATE_TO_STR[value], self.press_cpt)
        self.topic.publish(EEButtonStatus(value))


class EndEffectorPublisher(object):

    def __init__(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_buttons_state)

        self.custom_button = EndEffectorButton(
            '/niryo_robot_hardware_interface/end_effector_interface/custom_button_status')
        self.save_button = EndEffectorButton(
            '/niryo_robot_hardware_interface/end_effector_interface/save_pos_button_status')
        self.free_motion_button = EndEffectorButton(
            '/niryo_robot_hardware_interface/end_effector_interface/free_drive_button_status')

    def update_buttons_state(self, *_):
        self.custom_button.update()
        self.save_button.update()
        self.free_motion_button.update()
