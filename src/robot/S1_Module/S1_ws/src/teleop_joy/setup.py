from setuptools import setup
import os
from glob import glob

package_name = 'teleop_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='matthies.brouwer@student.hu.nl',
    description='Joy teleop with twist publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_pub = teleop_joy.joystick_ros2:main',
            'twist_pub = teleop_joy.teleop_joy:main',
        ],
    },
)
