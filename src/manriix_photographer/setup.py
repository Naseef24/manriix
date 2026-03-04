from setuptools import setup
from glob import glob
import os

package_name = 'manriix_photographer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),        
    ],
    install_requires=['setuptools', 'python3-paho-mqtt'],
    zip_safe=True,
    maintainer='hype',
    maintainer_email='hype@example.com',
    description='ROS2 Vision Bridge for Manriix AI Photographer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_vision_bridge = manriix_photographer.ros2_vision_bridge:main',
            'ai_photographer_bridge.py = manriix_photographer.ai_photographer_bridge:main',
        ],
    },
)