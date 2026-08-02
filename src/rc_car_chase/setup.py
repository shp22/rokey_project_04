from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rc_car_chase'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Webcam long-range approach + onboard-camera close-range follow of an RC car for TurtleBot4',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webcam_locator_node = rc_car_chase.webcam_locator_node:main',
            'chase_controller_node = rc_car_chase.chase_controller_node:main',
            'nav_goal_bridge_node = rc_car_chase.nav_goal_bridge_node:main',
            'calibrate_webcam_homography = rc_car_chase.calibrate_webcam_homography:main',
        ],
    },
)
