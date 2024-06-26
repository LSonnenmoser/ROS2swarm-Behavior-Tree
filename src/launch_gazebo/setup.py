import os
from glob import glob
from setuptools import setup

package_name = 'launch_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name,'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name,'maps'), glob('maps/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Marian Begemann',
    author_email='marian.begemann(at)student.uni-luebeck.de',
    maintainer='Tanja Kaiser',
    maintainer_email='kaiser(at)iti.uni-luebeck.de',
    license='apache-2.0',
    description='Launch script for gazebo simulation and the ros2swarm',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'add_bot_node = launch_gazebo.add_bot_node:main',
                    'ground_truth_publisher = launch_gazebo.ground_truth_publisher:main',
        ],
    },
)
