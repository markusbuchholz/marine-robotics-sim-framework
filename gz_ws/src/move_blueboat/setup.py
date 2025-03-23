# Copyright 2024, Markus Buchholz

from setuptools import setup
import os
from glob import glob

package_name = 'move_blueboat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include any other directories here
    ],
    install_requires=['setuptools', 'tf2_ros'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Update this if you have any executables
            #'robot_controller = move_blueboat.robot_controller:main',

            
            

            
            
            
            
            
            
            

        ],
    },
)
