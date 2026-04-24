"""
diff_drive.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('diff_driver_car_pkg')
    xacro_file = os.path.join(pkg, 'model', 'diff_model_robot.xacro')
    rviz_config = os.path.join(pkg, 'rviz',  'diff_simulator.rviz')   # ← NUEVO

    # ── Argumentos desde CLI ──────────────────────────────────────────────
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='3.14',
        description='Coordenada X del punto objetivo [m]'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='-2.2',
        description='Coordenada Y del punto objetivo [m]'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.025',
        description='Radio de las ruedas [m]'
    )
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base', default_value='0.0925',
        description='Distancia L entre ruedas [m]'
    )
    wheel_length_arg = DeclareLaunchArgument(
        'wheel_length', default_value='0.0125',
        description='Ancho de la rueda [m]'
    )

    # ── Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_file,              # ← espacio importante
                ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
                ' wheel_base:=',   LaunchConfiguration('wheel_base'),
                ' wheel_length:=', LaunchConfiguration('wheel_length'),
            ])
        }]
    )

    # ── Simulador ─────────────────────────────────────────────────────────
    simulator_node = Node(
        package='diff_driver_car_pkg',
        executable='diff_drive_simulator',
        name='diff_drive_simulator',
        output='screen',
        parameters=[{
            'wheel_radius':  LaunchConfiguration('wheel_radius'),
            'wheel_base':    LaunchConfiguration('wheel_base'),
            'update_rate':   50.0,
            'max_wheel_vel': 0.5,
        }]
    )

    # ── Goto point ────────────────────────────────────────────────────────
    goto_node = Node(
        package='diff_driver_car_pkg',
        executable='goto_point',
        name='goto_point',
        output='screen',
        parameters=[{
            'goal_x':          LaunchConfiguration('goal_x'),
            'goal_y':          LaunchConfiguration('goal_y'),
            'kp_linear':       0.5,
            'kp_angular':      2.0,
            'dist_tolerance':  0.05,
            'angle_threshold': 0.15,
            'max_linear_vel':  0.4,
            'max_angular_vel': 1.2,
        }]
    )
    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_node = Node(                               # ← NUEVO
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        wheel_radius_arg,
        wheel_base_arg,
        wheel_length_arg,
        robot_state_publisher,
        simulator_node,
        goto_node,
        rviz_node, 
    ])