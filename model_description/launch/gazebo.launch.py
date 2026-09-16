import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'model_description'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Inicia gzserver carregando expressamente as bibliotecas ROS
    start_gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--verbose'],
        output='screen'
    )

    # Inicia a interface gráfica
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Publica a árvore de transformações e robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Insere o robô na simulação após o serviço estar ativo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        start_gzserver,
        start_gzclient,
        robot_state_publisher_node,
        spawn_entity
    ])
