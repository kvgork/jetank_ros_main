from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetank_ros_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    # Declares pytest as a test dependency via extras_require (NOT the legacy
    # tests_require, which modern setuptools silently drops). colcon-ros only
    # runs pytest when it can see a 'test' dependency here; otherwise it falls
    # back to `setup.py test` (unittest) and collects zero tests.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='koen',
    maintainer_email='gorkom.projects@gmail.com',
    description='Main integration package for JeTank AI robot platform',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test_drive = jetank_ros_main.scripts.test_drive:main',
            'test_cameras = jetank_ros_main.scripts.test_cameras:main',
        ],
    },
)
