from setuptools import find_packages, setup

package_name = 'vision_pick_init'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javier-rojas',
    maintainer_email='javier990507@gmail.com',
    description='Package containing the vision system that detect strawberroies and outputs x,y,z position w.r.t the realsense camera',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = vision_pick_init.vision_node:main',
        ],
    },
)
