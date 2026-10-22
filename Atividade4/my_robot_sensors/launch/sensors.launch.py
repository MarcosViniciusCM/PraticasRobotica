"""Pratica 04 - Launch dos Sensores (Exercicios 1.1 e 1.2).

Sobe:
  - Gazebo Classic com o modelo do robo contendo IMU, GPS, Camera RGB e Lidar 2D;
  - robot_state_publisher com o modelo URDF completo dos sensores;
  - controladores ros2_control (joint_state_broadcaster e diff_drive_controller);
  - ponte diff_drive_bridge para /cmd_vel e /odom;
  - no gps_to_xyz para conversao de /gps/fix para /gps/pose e /gps/odom;
  - opcionalmente RViz2 (rviz:=true) e PlotJuggler (plotjuggler:=true).
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
    pkg_share = get_package_share_directory('my_robot_sensors')
    pkg_control_share = get_package_share_directory('my_robot_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'sensors.rviz')
    plotjuggler_config = os.path.join(pkg_share, 'config', 'plotjuggler_sensors.xml')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    plotjuggler = LaunchConfiguration('plotjuggler')
    teleop = LaunchConfiguration('teleop')

    # Variaveis de ambiente para localizacao de modelos no Gazebo
    model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', os.path.dirname(pkg_control_share))
    gazebo_models = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')
    gazebo_resources = AppendEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    gzclient = TimerAction(
        period=6.0,
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
            '-package_to_model',
        ],
        output='screen',
    )

    delayed_spawn = TimerAction(period=12.0, actions=[spawn_entity])

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

    # Conversor GPS (lat, lon, alt) -> (X, Y, Z)
    gps_converter = Node(
        package='my_robot_sensors',
        executable='gps_to_xyz',
        name='gps_to_xyz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_first_fix_as_datum': True,
            'frame_id': 'odom',
        }],
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -geometry 80x30 -title "teleop (use este terminal)" -e',
        output='screen',
        condition=IfCondition(teleop),
        parameters=[{
            'use_sim_time': True,
            'speed': 0.5,
            'turn': 1.0,
        }],
    )

    start_bridge_and_aux = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[diff_drive_bridge, gps_converter, teleop_node],
        )
    )

    # Visualizacao RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
    )

    # Visualizacao PlotJuggler
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['-l', plotjuggler_config],
        condition=IfCondition(plotjuggler),
    )

    return LaunchDescription([
        model_path,
        gazebo_models,
        gazebo_resources,
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre o gzclient'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abre o RViz2 com layout dos sensores'),
        DeclareLaunchArgument('plotjuggler', default_value='false',
                              description='Abre o PlotJuggler com layout dos sensores'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='Abre o teleop_twist_keyboard'),
        gzserver,
        gzclient,
        robot_state_publisher,
        delayed_spawn,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        start_bridge_and_aux,
        rviz_node,
        plotjuggler_node,
    ])
