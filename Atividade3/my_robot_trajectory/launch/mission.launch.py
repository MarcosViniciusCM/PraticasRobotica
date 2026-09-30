"""Pratica 03 - Exercicio 1.2: seguimento de missao.

Sobe a simulacao, o controlador de pose escolhido e o mission_follower, que
envia os waypoints de config/mission.yaml um a um.

    ros2 launch my_robot_trajectory mission.launch.py                        # continuo
    ros2 launch my_robot_trajectory mission.launch.py controller:=three_step # 3 manobras
    ros2 launch my_robot_trajectory mission.launch.py mission_file:=/outro/arquivo.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_trajectory')
    controller = LaunchConfiguration('controller')

    mission = Node(
        package='my_robot_trajectory',
        executable='mission_follower',
        output='screen',
        parameters=[
            LaunchConfiguration('mission_file'),
            {
                'use_sim_time': True,
                'label': ParameterValue(controller, value_type=str),
                'results_file': ParameterValue(
                    LaunchConfiguration('results_file'), value_type=str),
                'shutdown_when_done': ParameterValue(
                    LaunchConfiguration('shutdown_when_done'), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='continuous',
                              description='continuous | three_step'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='false = nao sobe o Gazebo (ja esta rodando)'),
        DeclareLaunchArgument('gui', default_value='true', description='Abre o gzclient'),
        DeclareLaunchArgument('mission_file',
                              default_value=os.path.join(pkg_share, 'config', 'mission.yaml'),
                              description='YAML com os waypoints'),
        DeclareLaunchArgument('results_file', default_value='',
                              description='CSV onde acrescentar o resumo da missao'),
        DeclareLaunchArgument('shutdown_when_done', default_value='false',
                              description='Encerra o mission_follower ao final'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'pose_control.launch.py')),
            launch_arguments={
                'controller': controller,
                'sim': LaunchConfiguration('sim'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
        mission,
        # usado por scripts/compare_mission.sh: fim da missao derruba a simulacao
        RegisterEventHandler(
            OnProcessExit(target_action=mission,
                          on_exit=[EmitEvent(event=Shutdown(reason='missao concluida'))]),
            condition=IfCondition(LaunchConfiguration('shutdown_when_done')),
        ),
    ])
