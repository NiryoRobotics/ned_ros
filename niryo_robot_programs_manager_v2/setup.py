from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['niryo_robot_programs_manager_v2'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
