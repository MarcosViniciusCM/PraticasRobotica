"""Pratica 05.2 - SLAM 2D com o slam_toolbox no Gazebo.

Sobe o Gazebo (sala + robo com Lidar), a odometria (verdade ou ruidosa), o slam_toolbox, o
percurso automatico e o registro CSV de pose.

    ros2 launch my_robot_slam slam.launch.py                                   # tudo ground truth
    ros2 launch my_robot_slam slam.launch.py lidar_noise:=0.03                 # ruido no Lidar
    ros2 launch my_robot_slam slam.launch.py lidar_noise:=0.03 odom:=noisy     # ruido no Lidar e na odometria
    ros2 launch my_robot_slam slam.launch.py rviz:=true gui:=true drive:=false # dirigir no teleop

Localizacao (exercicio 3.1, mapa ja salvo com serialize_map):
    ros2 launch my_robot_slam slam.launch.py localization:=true map_file:=/caminho/mapa \
        lidar_noise:=0.03 odom:=noisy drive:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _slam_node(context):
    here = get_package_share_directory('my_robot_slam')
    loc = LaunchConfiguration('localization').perform(context) == 'true'
    yaml = LaunchConfiguration('slam_params').perform(context) or ('slam_localization.yaml' if loc else 'slam_mapping.yaml')
    params = [os.path.join(here, 'config', yaml),
              {'use_sim_time': True}]
    if loc:
        pose = [float(v) for v in LaunchConfiguration('map_start_pose').perform(context).split(',')]
        params.append({'map_file_name': LaunchConfiguration('map_file').perform(context),
                       'map_start_pose': pose})
    return [Node(package='slam_toolbox',
                 executable='localization_slam_toolbox_node' if loc else 'async_slam_toolbox_node',
                 name='slam_toolbox', output='screen', parameters=params)]


def generate_launch_description():
    here = get_package_share_directory('my_robot_slam')
    est = get_package_share_directory('my_robot_estimation')
    ctrl = get_package_share_directory('my_robot_control')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'teleop': LaunchConfiguration('teleop'),
            'controllers_file': os.path.join(here, 'config', 'diff_drive_controller.yaml'),
            'world': os.path.join(here, 'worlds', 'room.world'),
            'gazebo_params_file': os.path.join(est, 'config', 'gazebo_params.yaml'),
            # o bringup monta "xacro <xacro_file> use_ros2_control:=..."; os argumentos do
            # xacro do Lidar entram junto com o caminho
            'xacro_file': [os.path.join(here, 'urdf', 'my_robot_slam.urdf.xacro'),
                           ' lidar_noise_std:=', LaunchConfiguration('lidar_noise')],
        }.items())

    odom = Node(
        package='my_robot_slam', executable='odom_tf_node.py', output='screen',
        parameters=[{'use_sim_time': True,
                     'mode': LaunchConfiguration('odom'),
                     'noise_scale': ParameterValue(LaunchConfiguration('odom_noise'), value_type=float)}])

    driver = Node(
        package='my_robot_slam', executable='waypoint_driver.py', output='screen',
        parameters=[os.path.join(here, 'config', 'waypoints.yaml'),
                    {'use_sim_time': True,
                     'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int)}],
        condition=IfCondition(LaunchConfiguration('drive')))

    logger = Node(
        package='my_robot_slam', executable='slam_logger.py', output='screen',
        parameters=[{'use_sim_time': True, 'file': LaunchConfiguration('log_file')}],
        condition=IfCondition(LaunchConfiguration('log')))

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(here, 'config', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='abre o gzclient'),
        DeclareLaunchArgument('teleop', default_value='false', description='teleop no xterm'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0',
                              description='desvio padrao do ruido do Lidar [m]; 0 = sem ruido'),
        DeclareLaunchArgument('odom', default_value='truth', description='truth | noisy'),
        DeclareLaunchArgument('odom_noise', default_value='1.0',
                              description='noise_scale da odometria ruidosa'),
        DeclareLaunchArgument('drive', default_value='true', description='percurso automatico'),
        DeclareLaunchArgument('laps', default_value='2'),
        DeclareLaunchArgument('log', default_value='true'),
        DeclareLaunchArgument('log_file', default_value='/tmp/slam.csv'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = usa um mapa salvo em vez de mapear'),
        DeclareLaunchArgument('slam_params', default_value='',
                              description='yaml de config/ (padrao slam_mapping ou slam_localization)'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='grafo de poses serializado (sem extensao)'),
        DeclareLaunchArgument('map_start_pose', default_value='0.0,0.0,0.0',
                              description='x,y,theta do robo no mapa ao ligar a localizacao'),
        sim,
        odom,
        OpaqueFunction(function=_slam_node),
        driver,
        logger,
        rviz,
    ])
