"""Pratica 03 - Exercicio 1.1: controle de pose.

Sobe a simulacao e um dos controladores de pose. O alvo e lido de /goal_pose
(geometry_msgs/PoseStamped).

    ros2 launch my_robot_trajectory pose_control.launch.py
    ros2 launch my_robot_trajectory pose_control.launch.py controller:=three_step
    ros2 launch my_robot_trajectory pose_control.launch.py sim:=false   # Gazebo ja aberto

Envie um alvo em outro terminal, por exemplo:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
      "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0}, orientation: {z: 0.7071, w: 0.7071}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = {
    'continuous': ('pose_controller', 'pose_controller.yaml'),
    'three_step': ('three_step_controller', 'three_step_controller.yaml'),
}


def controller_node(context, pkg_share):
    choice = LaunchConfiguration('controller').perform(context)
    if choice not in CONTROLLERS:
        raise RuntimeError(f"controller deve ser um de {list(CONTROLLERS)} (recebido '{choice}')")
    executable, yaml = CONTROLLERS[choice]
    return [Node(
        package='my_robot_trajectory',
        executable=executable,
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', yaml), {'use_sim_time': True}],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_trajectory')
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='continuous',
                              description='continuous | three_step'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='false = nao sobe o Gazebo (ja esta rodando)'),
        DeclareLaunchArgument('gui', default_value='true', description='Abre o gzclient'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
        OpaqueFunction(function=controller_node, args=[pkg_share]),
    ])
