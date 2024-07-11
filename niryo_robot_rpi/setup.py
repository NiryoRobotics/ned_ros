from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'niryo_robot_rpi',
        'niryo_robot_rpi.common',
        'niryo_robot_rpi.panel_v1',
        'niryo_robot_rpi.panel_v1.fake_panel',
        'niryo_robot_rpi.panel_v2',
        'niryo_robot_rpi.panel_v2.fake_panel',
        'niryo_robot_rpi.panel_v2.hardware',
    ],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'})

setup(**d)
