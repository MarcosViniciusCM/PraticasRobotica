"""Pratica 03 - simulacao base.

Reaproveita o bringup da pratica 02 (Gazebo + robo + diff_drive_controller +
ponte /cmd_vel e /odom), sem o teleop e com o YAML de limites de velocidade
deste pacote (o 8 com Omega = 0.5 passa de 1 m/s).

Os launches dos exercicios incluem este arquivo; ele tambem pode ser usado
sozinho, deixando o Gazebo aberto enquanto os nos sao testados com ros2 run.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = os.path.join(
        get_package_share_directory('my_robot_control'), 'launch', 'bringup.launch.py')
    controllers = os.path.join(
        get_package_share_directory('my_robot_trajectory'), 'config',
        'diff_drive_controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre o gzclient'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'teleop': 'false',
                'controllers_file': controllers,
            }.items(),
        ),
    ])
