from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['niryo_robot_utils', 'niryo_robot_utils.dataclasses', 'niryo_robot_utils.end_of_production_tests'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'})

setup(**d)
