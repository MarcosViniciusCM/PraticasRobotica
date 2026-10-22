#!/usr/bin/env python3
"""Expoe os topicos /cmd_vel e /odom do diff_drive_controller.

Por que este no existe
----------------------
O diff_drive_controller usa topicos relativos ao proprio no:

    /diff_drive_controller/cmd_vel_unstamped   (entrada, geometry_msgs/Twist)
    /diff_drive_controller/odom                (saida,   nav_msgs/Odometry)

Nesta versao do ros2_control (2.54) os controladores sao criados com opcoes de
no proprias, e as regras de remapping passadas ao controller_manager (inclusive
pela tag <ros><remapping> do plugin gazebo_ros2_control) nao chegam ate eles.

Este no faz a ponte para os nomes padrao do ecossistema ROS, que sao os pedidos
no roteiro: publica em /cmd_vel para o controlador e republica a odometria em
/odom. Assim qualquer fonte de Twist (teleop_twist_keyboard, joy, rqt, nav2)
funciona sem configuracao extra.

Os nomes dos topicos sao parametros, entao da para reaproveitar o no se o
controlador for renomeado.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DiffDriveBridge(Node):

    def __init__(self):
        super().__init__('diff_drive_bridge')

        self.declare_parameter('controller_name', 'diff_drive_controller')
        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('odom_out', '/odom')

        controller = self.get_parameter('controller_name').value
        cmd_vel_in = self.get_parameter('cmd_vel_in').value
        odom_out = self.get_parameter('odom_out').value

        cmd_vel_out = f'/{controller}/cmd_vel_unstamped'
        odom_in = f'/{controller}/odom'

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_out, qos)
        self._cmd_sub = self.create_subscription(
            Twist, cmd_vel_in, self._on_cmd_vel, qos)

        self._odom_pub = self.create_publisher(Odometry, odom_out, qos)
        self._odom_sub = self.create_subscription(
            Odometry, odom_in, self._on_odom, qos)

        self.get_logger().info(f'{cmd_vel_in} -> {cmd_vel_out}')
        self.get_logger().info(f'{odom_in} -> {odom_out}')

    def _on_cmd_vel(self, msg):
        self._cmd_pub.publish(msg)

    def _on_odom(self, msg):
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = DiffDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
