from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jobot_mono_follow'

data_files = [
    (os.path.join('share', package_name, 'jobot_mono_follow_cv/data'), glob('jobot_mono_follow/jobot_mono_follow_cv/data/*'))
]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhaoqi',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = jobot_mono_follow.hello_node:main',
            'agv_follow_node = jobot_mono_follow.agv_follow_node:main',
            'agv_follow_node_old = jobot_mono_follow.agv_follow_node_old:main'
        ],
    },
)
