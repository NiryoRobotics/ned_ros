import rospy
from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math

LED_ANGLE_OFFSET = 1.57


class LedRingSimulation:
    """
    Object which implements control method for the Led ring
    """

    def __init__(self, led_count):
        self.led_count = led_count  # Number of leds
        self.need_publish = True

        self.use_mesh = rospy.get_param("~use_mesh")
        self.led_marker_collada_path = rospy.get_param("~simulation_led_mesh_path")

        if rospy.get_param("/niryo_robot_led_ring/simulation_mode"):
            self.led_ring_markers_publish_rate = rospy.get_param("~simulation_led_ring_markers_publish_rate")
        else:
            self.led_ring_markers_publish_rate = rospy.get_param("~led_ring_markers_publish_rate")

        # for simulation mode
        self.markers_array = [self.new_marker(led_id) for led_id in range(self.led_count)]

        self.publish_markers_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.led_ring_markers_publish_rate),
                                                 self.__publish_makers_cb)

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=20, latch=True)
        self.show_led_ring_markers()

    def __publish_makers_cb(self, _):
        if self.need_publish:
            self.need_publish = False
            self.marker_pub.publish(self.markers_array)

    def set_all_led_markers(self, color_per_marker):
        """
        color_per_markers is a list of ColorRGBA values, of lenght : number of leds
        """
        for led_id, color_rgba_255 in enumerate(color_per_marker):
            self.markers_array[led_id].color = self.get_rgba_color_for_markers(color_rgba_255)

        self.need_publish = True

    def set_one_led_marker(self, led_id, color_rgba_255):
        """
        Color is a ColorRGBA values. Index is the index of the led ring pixel (marker) we want to change
        """
        self.markers_array[led_id].color = self.get_rgba_color_for_markers(color_rgba_255)

    def show_led_ring_markers(self):
        """
        Publish on the visualization marker array topic. Equivalent to strip.show, but in simulation.
        """
        self.need_publish = True

    def new_marker(self, position):
        """
        Return a positioned marker, equivalent to a Led pixel, but in simulation
        """
        led_ring_marker = Marker()
        led_ring_marker.header.frame_id = "led_ring_link"
        # led_ring_marker.header.stamp = rospy.Time.now()
        led_ring_marker.ns = "led ring"
        led_ring_marker.id = position
        led_ring_marker.action = led_ring_marker.ADD

        led_ring_marker.color = ColorRGBA(51, 51, 51, 0)
        led_ring_marker.lifetime = rospy.Duration(0)

        angle = (position * 2 * math.pi / self.led_count) + LED_ANGLE_OFFSET
        if self.use_mesh:
            led_ring_marker.type = led_ring_marker.MESH_RESOURCE
            led_ring_marker.scale.x = 1
            led_ring_marker.scale.y = 1
            led_ring_marker.scale.z = 1

            led_ring_marker.pose.position.z = -0.0
            led_ring_marker.pose.orientation = Quaternion(
                *quaternion_from_euler(0, 0, angle))

            led_ring_marker.mesh_use_embedded_materials = False
            led_ring_marker.mesh_resource = self.led_marker_collada_path

        else:
            led_ring_marker.type = led_ring_marker.CUBE
            led_ring_marker.scale.x = 0.01641
            led_ring_marker.scale.y = 0.0011
            led_ring_marker.scale.z = 0.002

            led_ring_marker.pose.position.z = 0
            led_ring_marker.pose.position.x = 0.0775 * math.cos(angle)
            led_ring_marker.pose.position.y = 0.0775 * math.sin(angle)
            led_ring_marker.pose.orientation = Quaternion(
                *quaternion_from_euler(0, 0, angle + math.pi / 2))

        return led_ring_marker

    @staticmethod
    def get_rgba_color_for_markers(color_255):
        """
        color_255 is a ColorRGBA object with values between 0 and 255.
        Return a ColorRGBA object with values between 0 and 1
        """
        ratio = 1.0 / 255.0
        color = ColorRGBA()
        color.r = color_255.r * ratio
        color.g = color_255.g * ratio
        color.b = color_255.b * ratio
        color.a = 0.8  # a little bit translucent

        return color
