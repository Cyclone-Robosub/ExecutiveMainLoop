#Created by Tanishq Dwivedi

#Create Python Package with ROS to command robot
from setuptools import setup

package_name = 'python_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cyclone RoboSub UC Davis',
    description='Python Commander Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_commander = python_commander.main:main',
        ],
    },
)