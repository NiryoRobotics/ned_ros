#!/usr/bin/env python

import rospy
import time
from niryo_robot_led_ring.srv import LedUser, LedUserRequest
from niryo_robot_led_ring.msg import LedRingStatus, LedRingAnimation
from niryo_robot_status.msg import RobotStatus

from std_msgs.msg import ColorRGBA


class Test_Led_ring:
    def __init__(self):
        # set the autonomous running robot status
        self.__robot_status_pub = rospy.Publisher('/niryo_robot_status/robot_status', RobotStatus, latch=True,
                                                  queue_size=10)
        # TODO : change the name of the file
        running_status = RobotStatus()
        running_status.robot_status = RobotStatus.RUNNING_AUTONOMOUS
        running_status.logs_status = RobotStatus.NONE
        self.__robot_status_pub.publish(running_status)

        time.sleep(1)

        self.solid()
        self.blackout()
        self.flashing()
        self.alternate()
        self.chase()
        self.wipe()
        self.rainbow()
        self.rainbow_chase()
        self.rainbow_cycle()
        self.go_up()
        self.go_up_and_down()

    def blackout(self):
        print 'NONE (turn off leds)'
        # test SOLID color
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()
        cmd.animation_mode.animation = LedRingAnimation.NONE
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def solid(self):
        print 'SOLID YELLOW'
        # test SOLID color
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()
        cmd.animation_mode.animation = LedRingAnimation.SOLID
        cmd.color = ColorRGBA()  # yellow
        cmd.color.r = 241
        cmd.color.g = 234
        cmd.color.b = 11
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def flashing(self):
        print 'FLASHING GREEN ENDLESSLY 4HZ'
        # test FLASHING
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.FLASHING
        cmd.color = ColorRGBA()  # green
        cmd.color.r = 49
        cmd.color.g = 244
        cmd.color.b = 22
        cmd.frequency = 4
        cmd.wait_answer = True
        cmd.iterations = 15
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def alternate(self):
        print 'ALTERNATE BLUE PURPLE RED 5 TIMES'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.ALTERNATE
        color1 = ColorRGBA()  # blue
        color1.r = 22
        color1.g = 244
        color1.b = 244
        color2 = ColorRGBA()  # purple
        color2.r = 140
        color2.g = 70
        color2.b = 220
        color3 = ColorRGBA()  # red
        color3.r = 246
        color3.g = 31
        color3.b = 99
        cmd.colors_list = [color1, color2, color3]
        cmd.iterations = 5
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def chase(self):
        print 'CHASE WHITE ENDLESSLY'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.CHASE
        color = ColorRGBA()  # white
        color.r = 255
        color.g = 255
        color.b = 255
        cmd.color = color
        cmd.speed_ms = 200
        cmd.iterations = 100
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def wipe(self):
        print 'WIPE PINK'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.COLOR_WIPE
        color = ColorRGBA()  # pink
        color.r = 220
        color.g = 32
        color.b = 87
        cmd.color = color
        cmd.speed_ms = 100
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def rainbow(self):
        print 'RAINBOW 4 TIME'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.RAINBOW
        cmd.speed_ms = 10
        cmd.iterations = 4
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def rainbow_cycle(self):
        print 'RAINBOW CYLE 2 TIME'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.RAINBOW_CYLE
        cmd.speed_ms = 5
        cmd.iterations = 2
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def rainbow_chase(self):
        print 'RAINBOW CHASE 4 TIME'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.RAINBOW_CHASE
        cmd.speed_ms = 40  # speed of the chase, not the rainbow
        cmd.iterations = 4
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def go_up(self):
        print 'GO UP 5 TIME'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.GO_UP
        cmd.color = ColorRGBA()  # blue
        cmd.color.r = 70
        cmd.color.g = 90
        cmd.color.b = 230
        cmd.iterations = 5
        cmd.speed_ms = 20
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def go_up_and_down(self):
        print 'GO UP AND DOWN 5 TIME'
        cmd = LedUserRequest()
        cmd.animation_mode = LedRingAnimation()

        cmd.animation_mode.animation = LedRingAnimation.GO_UP_AND_DOWN
        cmd.color = ColorRGBA()  # blue
        cmd.color.r = 70
        cmd.color.g = 90
        cmd.color.b = 230
        cmd.iterations = 5
        cmd.speed_ms = 20
        cmd.wait_answer = True
        print(self.__call_service('/niryo_robot_led_ring/set_user_animation', LedUser, cmd))
        print '\n'

    def __call_service(self, service_name, service_msg_type, *args):
        service_timeout = 3
        # Connect to service
        rospy.wait_for_service(service_name, service_timeout)
        # Call service
        service = rospy.ServiceProxy(service_name, service_msg_type)
        response = service(*args)
        return response


if __name__ == '__main__':
    rospy.init_node('led_ring_user_test', anonymous=False, log_level=rospy.INFO)
    try:
        node = Test_Led_ring()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
