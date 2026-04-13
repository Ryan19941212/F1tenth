from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpc_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'waypoints'),
            glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f1tenth',
    maintainer_email='f1tenth@todo.com',
    description='Linear MPC controller for F1Tenth trajectory tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node = mpc_controller.mpc_node:main',
        ],
    },
)
