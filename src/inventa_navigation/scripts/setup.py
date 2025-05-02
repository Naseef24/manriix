#ROS2 Script
#not used here in ROS1
from setuptools import setup

package_name = 'inventa_bot'

setup(
    name = package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/'+package_name, ['package.xml']),    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # added
    entry_points={
        'console_scripts': [
            'get_coords = robot.get_coords:listener'
        ],
    },
)

