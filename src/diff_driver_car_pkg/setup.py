from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'diff_driver_car_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'model'), glob("model/*.*")),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel2404',
    maintainer_email='manuel2404@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diff_drive_simulator = diff_driver_car_pkg.diff_drive_simulator:main',
            'goto_point = diff_driver_car_pkg.goto_point:main',
            'keyboard_teleop = diff_driver_car_pkg.keyboard_teleop:main',
        ],
    },
)
