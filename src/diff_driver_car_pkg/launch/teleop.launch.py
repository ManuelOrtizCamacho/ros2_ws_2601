"""
teleop.launch.py
================
Lanza solo el simulador + teleoperación por teclado.

Uso:
  ros2 launch diff_drive_pkg teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    simulator_node = Node(
        package='diff_drive_pkg',
        executable='diff_drive_simulator.py',
        name='diff_drive_simulator',
        output='screen',
        parameters=[{
            'wheel_radius':  0.05,
            'wheel_base':    0.30,
            'update_rate':   50.0,
            'max_wheel_vel': 1.0,
        }]
    )

    teleop_node = Node(
        package='diff_drive_pkg',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',   # Abre en ventana separada para capturar teclado
    )

    return LaunchDescription([
        simulator_node,
        teleop_node,
    ])
