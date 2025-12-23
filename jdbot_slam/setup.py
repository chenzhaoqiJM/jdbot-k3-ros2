from setuptools import find_packages, setup
import os
import glob

package_name = 'jdbot_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob.glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhaoqi',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = jdbot_slam.hello_node:main'
        ],
    },
)
