"""Pratica 05.1 - Exercicio 2.1: trajetoria em 8 controlada com o estado estimado.

Igual ao figure8.launch.py da pratica 03, mas o figure8_tracker le
/odometry/filtered (saida do filtro de Kalman) em vez de /odom.

    ros2 launch my_robot_estimation figure8_estimated.launch.py
    ros2 launch my_robot_estimation figure8_estimated.launch.py noise_scale:=5.0
    ros2 launch my_robot_estimation figure8_estimated.launch.py state_topic:=/odom/noisy   # controle com o dado bruto
    ros2 launch my_robot_estimation figure8_estimated.launch.py plotjuggler:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    here = get_package_share_directory('my_robot_estimation')
    traj = get_package_share_directory('my_robot_trajectory')

    estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'estimation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'sim': LaunchConfiguration('sim'),
            'noise_scale': LaunchConfiguration('noise_scale'),
        }.items())

    tracker = Node(
        package='my_robot_trajectory', executable='figure8_tracker', output='screen',
        parameters=[os.path.join(traj, 'config', 'figure8.yaml'),
                    {'use_sim_time': True,
                     'laps': ParameterValue(LaunchConfiguration('laps'), value_type=float),
                     'Omega': ParameterValue(LaunchConfiguration('Omega'), value_type=float),
                     'd': ParameterValue(LaunchConfiguration('d'), value_type=float),
                     'pid_x.kp': ParameterValue(LaunchConfiguration('kp'), value_type=float),
                     'pid_y.kp': ParameterValue(LaunchConfiguration('kp'), value_type=float),
                     'wait_for_trigger': ParameterValue(LaunchConfiguration('wait_for_trigger'),
                                                        value_type=bool)}],
        # o controle passa a ler a pose estimada
        remappings=[('odom', LaunchConfiguration('state_topic'))])

    logger = Node(
        package='my_robot_estimation', executable='state_logger.py', output='screen',
        parameters=[{'use_sim_time': True, 'file': LaunchConfiguration('log_file')}],
        condition=IfCondition(LaunchConfiguration('log')))

    plotjuggler = Node(
        package='plotjuggler', executable='plotjuggler', output='screen',
        arguments=['-n', '-l', os.path.join(here, 'config', 'plotjuggler_estimation.xml')],
        condition=IfCondition(LaunchConfiguration('plotjuggler')))

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('noise_scale', default_value='1.0'),
        DeclareLaunchArgument('state_topic', default_value='/odometry/filtered',
                              description='estado usado pelo controle'),
        DeclareLaunchArgument('laps', default_value='2.0'),
        DeclareLaunchArgument('d', default_value='0.3', description='ponto de controle a frente do eixo [m]'),
        DeclareLaunchArgument('kp', default_value='2.0', description='ganho P do seguimento'),
        DeclareLaunchArgument('Omega', default_value='0.25', description='frequencia do 8 [rad/s] - 0.5 o robo nao segue'),
        DeclareLaunchArgument('wait_for_trigger', default_value='false'),
        DeclareLaunchArgument('log', default_value='true', description='grava o CSV do experimento'),
        DeclareLaunchArgument('log_file', default_value='/tmp/estado.csv'),
        DeclareLaunchArgument('plotjuggler', default_value='false'),
        estimation,
        tracker,
        logger,
        plotjuggler,
    ])
