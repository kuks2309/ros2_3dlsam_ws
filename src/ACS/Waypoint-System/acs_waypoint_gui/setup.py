from setuptools import setup
import os
from glob import glob

package_name = 'acs_waypoint_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'acs_gui_node = acs_waypoint_gui.acs_gui_node:main',
            'acs_test_node = acs_waypoint_gui.acs_test_node:main',
        ],
    },
)
