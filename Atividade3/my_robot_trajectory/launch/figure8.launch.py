"""Pratica 03 - Exercicio 2.1: trajetoria em 8.

    ros2 launch my_robot_trajectory figure8.launch.py                    # realimentado, Kff do YAML
    ros2 launch my_robot_trajectory figure8.launch.py mode:=open_loop    # malha aberta
    ros2 launch my_robot_trajectory figure8.launch.py kff:=0.0           # so realimentacao
    ros2 launch my_robot_trajectory figure8.launch.py Omega:=0.8
    ros2 launch my_robot_trajectory figure8.launch.py plotjuggler:=true  # abre o PlotJuggler

Os argumentos mode, kff, Omega e laps, se passados, sobrescrevem o YAML
(config/figure8.yaml); vazios, vale o YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

OVERRIDES = {'mode': str, 'kff': float, 'Omega': float, 'laps': float}


def tracker_node(context, pkg_share):
    params = {'use_sim_time': True}
    for name, cast in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            params[name] = ParameterValue(cast(value), value_type=cast)
    for name in ('label', 'results_file'):
        params[name] = ParameterValue(LaunchConfiguration(name).perform(context),
                                      value_type=str)
    shutdown_when_done = (
        LaunchConfiguration('shutdown_when_done').perform(context).lower() == 'true')
    params['shutdown_when_done'] = shutdown_when_done
    tracker = Node(
        package='my_robot_trajectory',
        executable='figure8_tracker',
        output='screen',
        parameters=[LaunchConfiguration('config_file').perform(context), params],
    )
    actions = [tracker]
    if shutdown_when_done:
        # usado pelos scripts de experimento: fim do 8 derruba a simulacao
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=tracker, on_exit=[EmitEvent(event=Shutdown(reason='8 concluido'))])))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_trajectory')
    layout = os.path.join(pkg_share, 'config', 'plotjuggler_figure8.xml')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='false = nao sobe o Gazebo (ja esta rodando)'),
        DeclareLaunchArgument('gui', default_value='true', description='Abre o gzclient'),
        DeclareLaunchArgument('config_file',
                              default_value=os.path.join(pkg_share, 'config', 'figure8.yaml')),
        DeclareLaunchArgument('mode', default_value='', description='feedback | open_loop'),
        DeclareLaunchArgument('kff', default_value='', description='ganho do feedforward'),
        DeclareLaunchArgument('Omega', default_value='', description='frequencia do 8 [rad/s]'),
        DeclareLaunchArgument('laps', default_value='', description='numero de voltas'),
        DeclareLaunchArgument('label', default_value=''),
        DeclareLaunchArgument('results_file', default_value='',
                              description='CSV onde acrescentar o RMSE'),
        DeclareLaunchArgument('shutdown_when_done', default_value='false'),
        DeclareLaunchArgument('plotjuggler', default_value='false',
                              description='Abre o PlotJuggler com o layout do 8'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
        OpaqueFunction(function=tracker_node, args=[pkg_share]),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            arguments=['-l', layout],
            output='screen',
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),
    ])
