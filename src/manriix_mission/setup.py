from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'manriix_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={
        package_name: [
            'web_assets/*.urdf',
            'web_assets/meshes/*',
        ],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hype',
    maintainer_email='hype@manriix.com',
    description='Mission control for Manriix photography robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_fusion_node = manriix_mission.human_fusion_node:main',
            'poi_manager_node = manriix_mission.poi_manager_node:main',
            'exploration_node = manriix_mission.exploration_node:main',
            'mission_controller_node = manriix_mission.mission_controller_node:main',
            'control_panel = manriix_mission.control_panel:main',
            'pose_publisher = manriix_mission.pose_publisher:main',
        ],
    },
)
