#Created by Tanishq Dwivedi

#Create Python Package with ROS for reasearch_cyclone topic
from setuptools import setup

package_name = 'research_cyclone'

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
    description='Research Cyclone Data Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'research_node = research_cyclone.main:main',
        ],
    },
)