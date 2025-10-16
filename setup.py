import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'roomba_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'pyserial',
        'setuptools',
        ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='NikolausLaj, dudajulian',
    maintainer_email='nikolaus.lajtai@gmx.at, julian.v.duda@gmail.com',
    description='Interface node for steering iRobot Roombas via their serial connector.',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'roomba_ros2_driver = roomba_ros2_driver.roomba_ros2_driver:main'
        ],
    },
)

