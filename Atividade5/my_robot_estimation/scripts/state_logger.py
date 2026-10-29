#!/usr/bin/env python3
"""Grava em CSV, a 50 Hz (cada ground truth), o estado de todas as fontes.

colunas: t, truth_{x,y,yaw}, raw_{x,y,yaw} (odometria ruidosa integrada),
meas_{x,y,yaw} (sensor de posicao absoluta), ekf_{x,y,yaw}, des_{x,y,yaw}.
Usado por scripts/analyze.py para os graficos e o RMSE contra o ground truth.
"""

import csv
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def triple(p):
    return (p.position.x, p.position.y, yaw_of(p.orientation))


class StateLogger(Node):

    def __init__(self):
        super().__init__('state_logger')
        path = self.declare_parameter('file', '/tmp/estado.csv').value
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        names = ['truth', 'raw', 'meas', 'ekf', 'des']
        self.w.writerow(['t'] + [f'{n}_{c}' for n in names for c in ('x', 'y', 'yaw')])
        self.last = {n: (math.nan,) * 3 for n in names[1:]}
        self.t0 = None
        q = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/odom/noisy',
                                 lambda m: self.last.__setitem__('raw', triple(m.pose.pose)), q)
        self.create_subscription(PoseWithCovarianceStamped, '/sensor/pose',
                                 lambda m: self.last.__setitem__('meas', triple(m.pose.pose)), q)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self.last.__setitem__('ekf', triple(m.pose.pose)), q)
        self.create_subscription(PoseStamped, '/figure8/desired_pose',
                                 lambda m: self.last.__setitem__('des', triple(m.pose)), q)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, q)
        self.get_logger().info(f'Gravando em {path}')

    def on_truth(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        row = [t - self.t0, *triple(m.pose.pose)]
        for n in ('raw', 'meas', 'ekf', 'des'):
            row += list(self.last[n])
        self.w.writerow(row)

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = StateLogger()
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
