from glob import glob
from setuptools import find_packages, setup


package_name = 'cuvslam_rgbd'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bianbu',
    maintainer_email='bianbu@spacemit.com',
    description='CPU/RVV cuVSLAM RGB-D odometry for ROS 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rgbd_odometry = cuvslam_rgbd.rgbd_odometry_node:main',
            'safe_square_motion = cuvslam_rgbd.safe_square_motion:main',
        ],
    },
)
