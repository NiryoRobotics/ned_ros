from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['niryo_robot_rpi', 'niryo_robot_rpi.common', 'niryo_robot_rpi.fake_ned2',
              'niryo_robot_rpi.fake_ned_one', 'niryo_robot_rpi.ned2',
              'niryo_robot_rpi.ned2.hardware', 'niryo_robot_rpi.ned_one'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
