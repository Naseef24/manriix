from setuptools import setup
import os
from glob import glob

package_name = 'ultrasonic'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hype',
    maintainer_email='hype@hypeinsight.com',
    description='Safety nodes for ultrasonic sensor array',
    license='MIT',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'estop_node = ultrasonic_safety.estop_node:main',
    #         'costmap_node = ultrasonic_safety.costmap_node:main',
    #         'test_publisher = ultrasonic_safety.test_publisher:main',
    #     ],
    # },
    entry_points={
        'console_scripts': [
            'estop_node = ultrasonic.estop_node:main',
            'costmap_node = ultrasonic.costmap_node:main',
            'test_publisher = ultrasonic.test_publisher:main',
            'ultrasonic_serial_node = ultrasonic.ultrasonic_serial_node:main',
        ],
    },    
)