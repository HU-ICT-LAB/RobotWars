from setuptools import setup

package_name = 'S1_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='matthies.brouwer@student.hu.nl',
    description='Drivers for the robomaster S1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'S1_module = S1_drivers.S1_module:main',
        ],
    },
)
