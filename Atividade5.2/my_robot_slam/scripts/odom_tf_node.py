#!/usr/bin/env python3
"""Publica o TF odom -> base_link que o SLAM usa como odometria.

  mode = truth   a pose vem direto do ground truth do Gazebo (/ground_truth/odom):
                 odometria perfeita, sem ruido nem deriva.
  mode = noisy   a pose e a INTEGRAL das velocidades da odometria das rodas (/odom) mais
                 ruido branco e vies (dead reckoning): sem nenhuma fonte absoluta, deriva
                 sem limite - quem corrige e o SLAM, comparando o Lidar com o mapa.

Todos os desvios padrao valem para noise_scale = 1 e sao multiplicados por noise_scale.
O ground truth so e usado no modo truth e nos graficos/metricas, nunca no modo noisy.
Tambem publica a mesma pose em /odom/slam (nav_msgs/Odometry).
"""

import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomTf(Node):

    def __init__(self):
        super().__init__('odom_tf_node')
        p = self.declare_parameter
        self.mode = p('mode', 'truth').value
        self.k = p('noise_scale', 1.0).value
        self.s_v = p('linear_std', 0.03).value            # m/s
        self.s_w = p('angular_std', 0.03).value           # rad/s
        self.v_bias = p('linear_scale_bias', 0.03).value  # +3 % (raio da roda errado)
        self.w_bias = p('angular_bias', 0.005).value      # rad/s (vies do giroscopio)
        self.rng = random.Random(p('seed', 7).value)

        qos = QoSProfile(depth=10)
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom/slam', qos)
        self.x = self.y = self.th = 0.0
        self.last_t = None
        if self.mode == 'truth':
            self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, qos)
            self.get_logger().info('odometria PERFEITA (ground truth)')
        else:
            self.create_subscription(Odometry, '/odom', self.on_wheels, qos)
            k = self.k
            self.get_logger().info(
                f'odometria RUIDOSA, noise_scale={k:.2f}: v std={k*self.s_v:.3f} m/s '
                f'(+{100*k*self.v_bias:.1f} % de escala), w std={k*self.s_w:.3f} rad/s '
                f'(vies {k*self.w_bias:.4f} rad/s)')

    def emit(self, stamp, child, v=0.0, w=0.0):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = child
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header = t.header
        o.child_frame_id = child
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        self.pub.publish(o)

    def on_truth(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.th = yaw_of(m.pose.pose.orientation)
        self.emit(m.header.stamp, 'base_link')

    def on_wheels(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0:
            return
        k = self.k
        v = m.twist.twist.linear.x * (1.0 + k * self.v_bias) + self.rng.gauss(0.0, k * self.s_v)
        w = m.twist.twist.angular.z + k * self.w_bias + self.rng.gauss(0.0, k * self.s_w)
        self.th = wrap(self.th + w * dt)
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.emit(m.header.stamp, 'base_link', v, w)


def main():
    rclpy.init()
    node = OdomTf()
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
