"""Pratica 04 - simulacao base com sensores.

Substitui a simulacao simples pela simulacao contendo o modelo com
os 4 sensores (IMU, GPS, Camera e Lidar 2D) e o no de conversao gps_to_xyz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensors_launch = os.path.join(
        get_package_share_directory('my_robot_sensors'), 'launch', 'sensors.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre o gzclient'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abre o RViz2 com layout dos sensores'),
        DeclareLaunchArgument('plotjuggler', default_value='false',
                              description='Abre o PlotJuggler com layout dos sensores'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'plotjuggler': LaunchConfiguration('plotjuggler'),
                'teleop': 'false',
            }.items(),
        ),
    ])
