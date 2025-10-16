from setuptools import find_packages, setup
import glob

package_name = 'roomba_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pycreate2', 'pyserial'],
    zip_safe=True,
    maintainer='arva_master',
    maintainer_email='nikolaus.lajtai@gmx.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roomba_ros2_driver = roomba_ros2_driver.roomba_ros2_driver:main'
        ],
    },
)

