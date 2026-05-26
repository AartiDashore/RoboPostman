from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robopostman'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='RoboPostman autonomous delivery robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = robopostman.mission_manager:main',
            'obstacle_spawner = robopostman.obstacle_spawner:main',
            'scoring_trail = robopostman.scoring_trail:main',
            'camera_detector = robopostman.camera_detector:main',
            'keyboard_teleop = robopostman.keyboard_teleop:main',
            'mode_switch_gui = robopostman.mode_switch_gui:main',
        ],
    },
)
