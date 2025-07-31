import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'euroc_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Foxglove',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'firefly_state_publisher = euroc_slam.firefly_state_publisher:main',
        ],
    },
)
