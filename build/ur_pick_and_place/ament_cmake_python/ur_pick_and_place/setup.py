from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_pick_and_place',
    version='0.0.0',
    packages=find_packages(
        include=('ur_pick_and_place', 'ur_pick_and_place.*')),
)
