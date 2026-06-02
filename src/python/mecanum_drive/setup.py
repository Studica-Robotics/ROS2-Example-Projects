from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanum_drive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Taylor',
    maintainer_email='james.taylor@studica.com',
    description='Mecanum holonomic drive controller for a 4-motor robot using Studica Titan motor controllers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mecanum_drive_node = mecanum_drive.mecanum_drive_node:main',
            'odometry_node      = mecanum_drive.odometry_node:main',
        ],
    },
)
