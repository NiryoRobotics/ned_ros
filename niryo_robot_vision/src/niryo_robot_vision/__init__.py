import cv2

import rospy
from sensor_msgs.msg import CompressedImage

publishers = {}


def pub_debug_img(img, topic='~debug'):
    """
    Publish an image to the specified debug topic.

    :param img: The image to be published.
    """
    global publishers
    if topic not in publishers:
        publishers[topic] = rospy.Publisher(topic, CompressedImage, queue_size=1, latch=True)

    _, encoded_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'jpg'
    msg.data = encoded_img.tobytes()
    publishers[topic].publish(msg)
