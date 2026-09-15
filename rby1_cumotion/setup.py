from glob import glob

from setuptools import find_packages, setup

setup(
    name='rby1_cumotion',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rby1_cumotion']),
        ('share/rby1_cumotion', ['package.xml']),
        ('share/rby1_cumotion/launch', glob('launch/*.launch.py')),
        ('share/rby1_cumotion/config', glob('config/*.yaml')),
        ('share/rby1_cumotion/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Rainbow Robotics',
    maintainer_email='rainbow@rainbow-robotics.com',
    description='cuMotion integration and arm motion example for RBY1',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'prepare_model = rby1_cumotion.model:main',
        'move_arm = rby1_cumotion.move_arm:main',
        'prepare_sim = rby1_cumotion.prepare_sim:main',
    ]},
)
