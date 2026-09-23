"""Pratica 02 - Exercicio 2.1 (launch unico do sistema completo).

Sobe, em uma unica chamada:
  - Gazebo Classic (gzserver + gzclient);
  - robot_state_publisher com o modelo URDF;
  - o robo no simulador (spawn_entity), que por sua vez carrega o
    controller_manager via plugin gazebo_ros2_control;
  - os controladores: joint_state_broadcaster e diff_drive_controller;
  - o teleop_twist_keyboard, em uma janela xterm propria (precisa de stdin).

Uso:
    ros2 launch my_robot_control bringup.launch.py
    ros2 launch my_robot_control bringup.launch.py teleop:=false   # sem teclado
    ros2 launch my_robot_control bringup.launch.py gui:=false      # sem gzclient
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    gui = LaunchConfiguration('gui')

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

    teleop = LaunchConfiguration('teleop')

    # O caminho do yaml e injetado no xacro, para que o plugin
    # gazebo_ros2_control leia exatamente o arquivo instalado deste pacote.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    # ------------------------------------------------------------------ Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # gzclient atrasado: subir junto com o gzserver causa uma condicao de corrida
    # em que a GUI tenta criar a camera antes da cena existir. O rasterizador por
    # software evita a mesma trava quando a GPU virtual do WSL falha.
    gzclient = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
                output='screen',
                condition=IfCondition(gui),
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'OGRE_RTT_MODE': 'Copy',
                },
            )
        ],
    )

    # ------------------------------------------------------------------- Robo
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-z', '0.1',
            '-timeout', '120',
            # converte package:// para model:// no URDF recebido: o Gazebo
            # Classic so resolve model://, e o mesh do chassi (STL) usa package://.
            '-package_to_model',
        ],
        output='screen',
    )

    delayed_spawn = TimerAction(period=16.0, actions=[spawn_entity])

    # ----------------------------------------------------------- Controladores
    # Os spawners so podem rodar depois que o robo existe no Gazebo, porque e o
    # plugin do modelo que instancia o controller_manager. Por isso encadeamos
    # os spawners na saida do spawn_entity, em vez de usar tempos fixos.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # ------------------------------------------------------------- Topicos
    # Expoe /cmd_vel e /odom (os nomes pedidos no roteiro) a partir dos topicos
    # relativos do diff_drive_controller. Ver scripts/diff_drive_bridge.py.
    diff_drive_bridge = Node(
        package='my_robot_control',
        executable='diff_drive_bridge.py',
        name='diff_drive_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'controller_name': 'diff_drive_controller',
            'cmd_vel_in': '/cmd_vel',
            'odom_out': '/odom',
        }],
    )

    # ---------------------------------------------------------------- Teleop
    # O teleop_twist_keyboard le o teclado de stdin, entao precisa de um
    # terminal proprio quando roda dentro de um launch.
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -geometry 80x30 -title "teleop (use este terminal)" -e',
        output='screen',
        condition=IfCondition(teleop),
        parameters=[{
            'use_sim_time': True,
            'speed': 0.5,        # m/s inicial
            'turn': 1.0,         # rad/s inicial
        }],
    )

    # So liga a ponte e o teclado depois que o controlador esta ativo, senao os
    # primeiros comandos sao publicados no vazio.
    start_bridge_and_teleop = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[diff_drive_bridge, teleop_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre o gzclient (interface grafica do Gazebo)'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Abre o teleop_twist_keyboard em um xterm'),
        model_path,
        gazebo_models,
        gazebo_resources,
        gzserver,
        gzclient,
        robot_state_publisher,
        delayed_spawn,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        start_bridge_and_teleop,
    ])
