from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['niryo_robot_led_ring', 'niryo_robot_led_ring.api'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
