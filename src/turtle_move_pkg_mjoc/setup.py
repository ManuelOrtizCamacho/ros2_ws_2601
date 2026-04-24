from setuptools import find_packages, setup

package_name = 'turtle_move_pkg_mjoc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel2404',
    maintainer_email='manuel2404@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_move_node_mjoc = turtle_move_pkg_mjoc.turtle_move_node_mjoc:main',
        ],
    },
)
