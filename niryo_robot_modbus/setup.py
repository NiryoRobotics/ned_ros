from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['niryo_robot_modbus', 'niryo_robot_modbus.old', 'niryo_robot_modbus.addressing'],
                             package_dir={'': 'src'})

setup(**d)
