from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manriix_oak_obstacles'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hype',
    maintainer_email='hype@manriix.com',
    description='OAK-D on-chip edge obstacle detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oak_obstacle_node = manriix_oak_obstacles.oak_obstacle_node:main',
            'detection_to_pointcloud_node = manriix_oak_obstacles.detection_to_pointcloud_node:main',
            'passthrough_filter = manriix_oak_obstacles.pass_through:main',
            'segformer_test_node = manriix_oak_obstacles.segformer_test_node:main',
            'pointcloud_filter_node = manriix_oak_obstacles.pointcloud_filter_node:main',
            'cluster_tracking_node = manriix_oak_obstacles.cluster_tracking_node:main',
            'static_dynamic_classifier_node = manriix_oak_obstacles.static_dynamic_classifier_node:main',
            'people_tracker_node = manriix_oak_obstacles.people_tracker_node:main',
            'fusion_node = manriix_oak_obstacles.fusion_node:main',
            'motion_estimation_node = manriix_oak_obstacles.motion_estimation_node:main',
            'occupancy_grid_node = manriix_oak_obstacles.occupancy_grid_node:main',
            'detection_overlay_node = manriix_oak_obstacles.detection_overlay_node:main',
            'fov_marker_node = manriix_oak_obstacles.fov_marker_node:main',
        ],
    },
)