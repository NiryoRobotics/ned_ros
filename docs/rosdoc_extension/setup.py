from setuptools import setup, find_packages

setup(
    name="niryo_rosdoc",
    version="0.1",
    packages=find_packages(),
    install_requires=['sphinx', 'PyYaml'],
)