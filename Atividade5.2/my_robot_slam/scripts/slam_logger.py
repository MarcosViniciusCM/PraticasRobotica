#!/usr/bin/env python3
"""Grava em CSV, a 10 Hz, a pose do robo segundo cada fonte.

colunas: t, truth_{x,y,yaw} (Gazebo), odom_{x,y,yaw} (TF odom->base_link: o que a odometria
sozinha diria) e slam_{x,y,yaw} (TF map->base_link: o que o SLAM estima). Se o SLAM ainda nao
publicou o map->odom, as colunas slam_* ficam vazias (nan).
Usado por scripts/analyze.py para o erro de pose contra o ground truth.
"""

import csv
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamLogger(Node):

    def __init__(self):
        super().__init__('slam_logger')
        path = self.declare_parameter('file', '/tmp/slam.csv').value
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t'] + [f'{n}_{c}' for n in ('truth', 'odom', 'slam') for c in 'xy'] +
                        [f'{n}_yaw' for n in ('truth', 'odom', 'slam')])
        self.truth = None
        self.t0 = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, QoSProfile(depth=10))
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Gravando em {path}')

    def on_truth(self, m):
        self.truth = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def lookup(self, parent):
        try:
            t = self.buf.lookup_transform(parent, 'base_link', Time())
        except Exception:
            return (math.nan,) * 3
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def tick(self):
        if self.truth is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        tr, od, sl = self.truth, self.lookup('odom'), self.lookup('map')
        self.w.writerow([now - self.t0, tr[0], tr[1], od[0], od[1], sl[0], sl[1], tr[2], od[2], sl[2]])

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SlamLogger()
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
