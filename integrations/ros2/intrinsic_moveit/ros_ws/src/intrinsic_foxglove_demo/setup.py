import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'intrinsic_foxglove_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Foxglove',
    maintainer_email='support@foxglove.dev',
    description='Foxglove demo driver for Intrinsic MoveIt grasp planning on mock hardware.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grasp_demo_driver = intrinsic_foxglove_demo.grasp_demo_driver:main',
        ],
    },
)
