import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'technocrats_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        # Install all CSV map files into the ROS 2 package share directory
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*.csv')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabeha',
    maintainer_email='fabeha@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = technocrats_controller.controller_node:main',
        ],
    },
)