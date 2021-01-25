import rospy
from sensor_msgs.msg import CompressedImage
from niryo_robot_vision.image_functions import compress_image


def generate_msg_from_image(img, compression_quality=90):
    """
    Generate ROS CompressedImage message from an OpenCV Image
    :param img: OpenCV Image
    :param compression_quality: integer between 1 - 100. The higher it is, the less information will be lost,
    but the heavier the compressed image will be
    :return: success, msg
    """
    result, compressed_img = compress_image(img, quality=compression_quality)
    if not result:
        rospy.logerr("Failed to generate CompressedImage message from image")
        return False, None

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpg"
    msg.data = compressed_img

    return True, msg
