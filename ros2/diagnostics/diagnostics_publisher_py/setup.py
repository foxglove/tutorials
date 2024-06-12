from setuptools import find_packages, setup

package_name = 'diagnostics_publisher_py'

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
    maintainer='José Luis Millán-Valbuena',
    maintainer_email='jlmv.96@gmail.com',
    description='A package to publish a basic diagnostics message in Python',
    license='MPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diagnostics_publisher_py=diagnostics_publisher_py.main:main",
        ],
    },
)
