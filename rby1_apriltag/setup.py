from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rby1_apriltag'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rainbow Robotics',
    maintainer_email='rainbow@rainbow-robotics.com',
    description='Target AprilTag filter and pose stabilizer for Rainbow Robotics RBY1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_tag_filter = rby1_apriltag.target_tag_filter:main',
        ],
    },
)
