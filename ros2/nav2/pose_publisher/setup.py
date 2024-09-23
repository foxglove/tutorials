from setuptools import find_packages, setup

package_name = 'pose_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jlmv.96@gmail.com',
    description='PoseStamped republisher with simulation time',
    license='MPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = pose_publisher.pose_publisher:main',
        ],
    },
)
