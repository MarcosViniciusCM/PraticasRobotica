"""Pratica 02 - Exercicio 1.1.

Sobe o Gazebo Classic e carrega apenas o modelo URDF do robo, sem a pilha de
controle (use_ros2_control:=false). Serve para validar a conversao do modelo
SDF da pratica 01 para URDF antes de configurar os controladores.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # ------------------------------------------------- caminhos do Gazebo
    # O gzclient e um processo separado e NAO recebe os caminhos que o
    # gzserver.launch.py monta internamente. Como quem desenha o mesh do chassi
    # e o gzclient, sem isto o "model://my_robot_control/..." nao resolve e o
    # chassi fica invisivel - aparecem so as rodas e a roda boba, que sao
    # primitivas (cilindro/esfera) e nao dependem de arquivo externo.
    # Estas acoes rodam antes dos processos, entao valem para gzserver e gzclient.
    model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', os.path.dirname(pkg_share))

    # Caminhos padrao do Gazebo Classic. Normalmente viriam de
    # /usr/share/gazebo/setup.sh, que o setup.bash do ROS 2 nao carrega - sem
    # eles o Gazebo reclama que nao acha a biblioteca de shaders e os materiais.
    gazebo_models = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')
    gazebo_resources = AppendEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')


    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control]),
        value_type=str,
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # O gzclient precisa subir DEPOIS que o gzserver ja estiver de pe: iniciar
    # os dois ao mesmo tempo causa uma condicao de corrida em que a GUI tenta
    # criar a camera do usuario antes do servidor ter publicado a cena, travando
    # o processo com uma assertion em rendering::Camera. O rasterizador por
    # software evita a mesma trava quando a GPU virtual do WSL falha ao criar
    # essa camera; sem efeito em Linux nativo com GPU real.
    gzclient = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
                output='screen',
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'OGRE_RTT_MODE': 'Copy',
                },
            )
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_robot',
                    '-z', '0.1',
                    '-timeout', '60',
                    # converte package:// para model:// no URDF recebido: o Gazebo
                    # Classic so resolve model://, e o mesh do chassi (STL) usa package://.
                    '-package_to_model',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Carrega o modelo com a interface ros2_control (exercicio 1.2)',
        ),
        model_path,
        gazebo_models,
        gazebo_resources,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
