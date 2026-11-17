#!/usr/bin/env python3
"""Dirige o robo por uma lista de pontos (x, y) para o SLAM mapear a sala.

Roda sempre o MESMO percurso, para as tres situacoes de ruido serem comparaveis. O controle
usa o ground truth (nao e parte do SLAM; em um robo real seria o teleop ou um planejador).
Em cada ponto: gira no lugar ate alinhar, depois anda em reta corrigindo o rumo.
Termina com 'TRAJETORIA CONCLUIDA' no log.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Driver(Node):

    def __init__(self):
        super().__init__('waypoint_driver')
        p = self.declare_parameter
        flat = p('waypoints', [0.0, 0.0]).value
        self.pts = list(zip(flat[0::2], flat[1::2]))
        self.laps = int(p('laps', 2).value)
        self.speed = p('speed', 0.4).value
        self.wmax = p('max_angular', 1.0).value
        self.tol = p('tolerance', 0.15).value
        self.delay = p('start_delay', 10.0).value
        self.lap = 0
        self.i = 0
        self.t0 = None
        self.done = False
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, qos)
        self.get_logger().info(f'{len(self.pts)} pontos x {self.laps} voltas, v={self.speed} m/s')

    def on_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        cmd = Twist()
        if self.done or t - self.t0 < self.delay:
            self.pub.publish(cmd)
            return
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        th = yaw_of(m.pose.pose.orientation)
        gx, gy = self.pts[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            self.i += 1
            if self.i == len(self.pts):
                self.i = 0
                self.lap += 1
                self.get_logger().info(f'volta {self.lap}/{self.laps} concluida')
                if self.lap >= self.laps:
                    self.done = True
                    self.get_logger().info('TRAJETORIA CONCLUIDA')
            self.pub.publish(cmd)
            return
        err = wrap(math.atan2(dy, dx) - th)
        if abs(err) > 0.3:
            cmd.angular.z = max(-self.wmax, min(self.wmax, 1.5 * err))
        else:
            cmd.linear.x = min(self.speed, 0.7 * dist + 0.08)
            cmd.angular.z = max(-self.wmax, min(self.wmax, 2.0 * err))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Driver()
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
