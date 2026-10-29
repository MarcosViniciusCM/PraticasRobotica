"""Pratica 05.1 - Exercicio 1.1: estimacao de estados com o filtro de Kalman.

Sobe o Gazebo com o robo + IMU + ground truth, os sensores ruidosos e o
ekf_filter_node do robot_localization, que publica /odometry/filtered.

    ros2 launch my_robot_estimation estimation.launch.py
    ros2 launch my_robot_estimation estimation.launch.py noise_scale:=4.0
    ros2 launch my_robot_estimation estimation.launch.py gui:=false sim:=false
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
    ctrl = get_package_share_directory('my_robot_control')
    traj = get_package_share_directory('my_robot_trajectory')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'teleop': 'false',
            'controllers_file': os.path.join(here, 'config', 'diff_drive_controller.yaml'),
            'world': os.path.join(traj, 'worlds', 'top_view.world'),
            # /clock a 100 Hz (padrao 10 Hz). Com use_sim_time o EKF so roda a cada
            # tick do /clock, entao com 10 Hz ele publicaria a 10 Hz e o controle
            # (disparado pelo /odometry/filtered) cairia junto.
            'gazebo_params_file': os.path.join(here, 'config', 'gazebo_params.yaml'),
            'xacro_file': os.path.join(here, 'urdf', 'my_robot_sensors.urdf.xacro'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    noisy = Node(
        package='my_robot_estimation', executable='noisy_sensors.py', output='screen',
        parameters=[{'use_sim_time': True,
                     'noise_scale': ParameterValue(LaunchConfiguration('noise_scale'),
                                                   value_type=float)}])

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_file')])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='false = nao sobe o Gazebo (ja esta rodando)'),
        DeclareLaunchArgument('gui', default_value='true', description='Abre o gzclient'),
        DeclareLaunchArgument('noise_scale', default_value='1.0',
                              description='multiplica o desvio padrao de todos os sensores'),
        DeclareLaunchArgument('ekf_file', default_value=os.path.join(here, 'config', 'ekf.yaml')),
        sim,
        noisy,
        ekf,
    ])
