from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_delivery_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Autonomous drone delivery with YOLOv8 person detection',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'payload_manager = drone_delivery_system.payload_manager:main',
            'coordinate_mission_controller = drone_delivery_system.coordinate_mission_controller:main',
            'yolo_search_controller = drone_delivery_system.yolo_search_controller:main',
        ],
    },
)
