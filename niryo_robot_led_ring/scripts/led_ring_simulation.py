#!/usr/bin/env python

import rospy
import time
import rospkg
import os

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math


class LedRingSimulation:
    """
    Object which implements control method for the Led ring
    """

    def __init__(self, led_count):
        self.led_count = led_count  # Number of leds
        self.led_marker_collada_path = rospy.get_param("~simulation_led_mesh_path")

        # for simulation mode
        self.markers_array = [self.new_marker(led_id) for led_id in range(self.led_count)]

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=20, latch=True)
        self.show_led_ring_markers()

    def set_all_led_markers(self, color_per_marker):
        """
        color_per_markers is a list of ColorRGBA values, of lenght : number of leds
        """
        for led_id, color_rgba_255 in enumerate(color_per_marker):
            self.markers_array[led_id].color = self.get_rgba_color_for_markers(color_rgba_255)

        self.marker_pub.publish(self.markers_array)

    def set_one_led_marker(self, led_id, color_rgba_255):
        """
        Color is a ColorRGBA values. Index is the index of the led ring pixel (marker) we want to change
        """
        self.markers_array[led_id].color = self.get_rgba_color_for_markers(color_rgba_255)

    def show_led_ring_markers(self):
        """
        Publish on the visualization marker array topic. Equivalent to strip.show, but in simulation.
        """
        self.marker_pub.publish(self.markers_array)

    def new_marker(self, position):
        """
        Return a positioned marker, equivalent to a Led pixel, but in simulation
        """
        led_ring_marker = Marker()
        led_ring_marker.header.frame_id = "shoulder_link"
        #led_ring_marker.header.stamp = rospy.Time.now()
        led_ring_marker.ns = "led ring"
        led_ring_marker.id = position
        led_ring_marker.type = led_ring_marker.MESH_RESOURCE
        led_ring_marker.action = led_ring_marker.ADD

        led_ring_marker.scale.x = 1
        led_ring_marker.scale.y = 1
        led_ring_marker.scale.z = 1

        led_ring_marker.color = ColorRGBA(51, 51, 51, 0)

        led_ring_marker.mesh_use_embedded_materials = False
        led_ring_marker.mesh_resource = self.led_marker_collada_path

        led_ring_marker.pose.position.z = -0.012
        led_ring_marker.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, position * 2 * math.pi / self.led_count))

        led_ring_marker.lifetime = rospy.Duration(0)

        return led_ring_marker

    @staticmethod
    def get_rgba_color_for_markers(color_255):
        """
        color_255 is a ColorRGBA object with values between 0 and 255. Return a ColorRGBA object with values between 0 and 1
        """
        ratio = 1.0 / 255.0
        color = ColorRGBA()
        color.r = color_255.r * ratio
        color.g = color_255.g * ratio
        color.b = color_255.b * ratio
        color.a = 0.8  # a little bit translucid

        return color
