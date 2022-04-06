from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['niryo_robot_tools_commander', 'niryo_robot_tools_commander.api'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
