from setuptools import find_packages, setup

package_name = 'jdbot_voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice_cmd.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhaoqi',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_cmd_node = jdbot_voice_control.voice_cmd_node:main',
        ],
    },
)
