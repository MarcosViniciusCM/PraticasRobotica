import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_model_description = get_package_share_directory('model_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_model_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        )
    )

    # O gzclient precisa subir DEPOIS que o gzserver ja estiver de pe: iniciar
    # os dois ao mesmo tempo causa uma condicao de corrida em que a GUI tenta
    # criar a camera do usuario antes do servidor ter publicado a cena, travando
    # o processo com uma assertion em rendering::Camera. O rasterizador por
    # software evita a mesma trava quando a GPU virtual do WSL (/dev/dxg, driver
    # d3d12) falha ao criar essa camera; sem efeito em Linux nativo com GPU real.
    # Os caminhos GAZEBO_MODEL_PATH/PLUGIN_PATH/RESOURCE_PATH ja vem corretos do
    # ambiente (setup.bash do gazebo) e nao devem ser sobrescritos aqui: fazer
    # isso perde o caminho padrao de midia do Gazebo e tambem trava a camera.
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
        parameters=[{'robot_description': robot_description}],
    )

    # Adiado ainda mais para dar tempo do gzclient (que ja teve seus 8s de
    # espera) terminar de inicializar a cena e a camera antes do robo aparecer.
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
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
