from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'manriix_camera'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'templates'),
            glob('templates/*')),
        (os.path.join('share', package_name, 'static/css'),
            glob('static/css/*')),                                    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hype',
    maintainer_email='lkulasooriya97@gmail.com',
    description='ROS2 camera control system for Manriix photography robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = manriix_camera.camera_node:main',
        ],
    },
)
