import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cobot1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.example')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woody',
    maintainer_email='woody.myung@gmail.com',
    description='Cobot control package with modular architecture',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'motion_executor         = cobot1.nodes.motion_executor:main',
            'robot_status_publisher  = cobot1.nodes.robot_status_publisher:main',
            'task_controller         = cobot1.nodes.task_controller:main',
            'task_cli                = cobot1.nodes.task_cli:main',
            'ui_bridge               = cobot1.nodes.ui_bridge:main',
        ],
    },
)
