import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


pkg_folder = 'modelrobot_pkg_mjoc'
rviz_file = 'rviz_config.rviz'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_rviz_config_path = os.path.join(pkg_path + '/model/' + rviz_file)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Absolute path to rviz config file')


    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Launch!
    return LaunchDescription([
        rviz_arg,
        rviz_node,
    ])