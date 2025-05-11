from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_drone_colab'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/block_box'), glob('models/block_box/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/rover'), glob('models/rover/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanav',
    maintainer_email='kanav@todo.todo',
    description='ROS 2 package for coordinating drone and rover simulations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_node = rover_drone_colab.arm_node:main',
            'arm_rover = rover_drone_colab.arm_rover:main',
            'drone_controller_node = rover_drone_colab.drone_controller_node:main',
            'rover_controller_node = rover_drone_colab.rover_controller_node:main',
        ],
    },
)

