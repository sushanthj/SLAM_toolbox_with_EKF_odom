from setuptools import setup
import os
from glob import glob

package_name = 'slam_toolbox_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sush',
    maintainer_email='sushanth.jayanth@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'static_broadcaster = slam_toolbox_tf2.static_broadcaster:main',
          'odom_to_base_broadcaster = slam_toolbox_tf2.odom_to_base_broadcaster:main',
          'base_to_link = slam_toolbox_tf2.base_to_link:main',
          'base_link_to_lidar = slam_toolbox_tf2.base_link_to_lidar:main',
        ],
    },
)
