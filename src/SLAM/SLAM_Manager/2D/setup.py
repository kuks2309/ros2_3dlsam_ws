from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'slam_manager_2d'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['ui/*.ui'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('slam_manager_2d/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='amap@todo.todo',
    description='ROS2 2D SLAM Launch Manager with Qt GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_manager_2d = slam_manager_2d.slam_manager_2d_node:main',
        ],
    },
)
