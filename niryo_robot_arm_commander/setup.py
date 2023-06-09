from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['niryo_robot_arm_commander'],
    packages=find_packages(exclude=['test']),
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
