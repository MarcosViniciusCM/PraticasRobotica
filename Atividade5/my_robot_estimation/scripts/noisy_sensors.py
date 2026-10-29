#!/usr/bin/env python3
"""Sensores ruidosos para o exercicio de estimacao de estados.

O Gazebo entrega medidas quase perfeitas. Este no as estraga de forma
controlada, para o filtro de Kalman ter o que filtrar. Todos os desvios padrao
abaixo valem para noise_scale = 1 e sao multiplicados por noise_scale
("aumente o ruido dos sensores" = aumentar este parametro).

  entrada            saida                 o que simula
  /odom              /odom/noisy           odometria das rodas: velocidades com
                                           ruido + vies de escala; a pose e a
                                           INTEGRAL das velocidades ruidosas
                                           (dead reckoning), logo deriva
  /imu/out           /imu/data_noisy       giroscopio: ruido branco + vies
  /ground_truth/odom /sensor/pose          sensor de posicao absoluta (camera,
                                           GPS/RTK, marcadores): x, y, yaw reais
                                           + ruido branco, a 10 Hz
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class NoisySensors(Node):

    def __init__(self):
        super().__init__('noisy_sensors')
        p = self.declare_parameter
        self.scale = p('noise_scale', 1.0).value
        # desvios padrao com noise_scale = 1
        self.s_odom_v = p('odom_linear_std', 0.05).value        # m/s
        self.s_odom_w = p('odom_angular_std', 0.05).value       # rad/s
        self.odom_v_bias = p('odom_linear_scale_bias', 0.03).value  # +3 % (raio da roda errado)
        self.s_gyro = p('gyro_std', 0.02).value                 # rad/s
        self.gyro_bias = p('gyro_bias', 0.01).value             # rad/s
        self.s_pos = p('pose_xy_std', 0.05).value               # m
        self.s_yaw = p('pose_yaw_std', 0.05).value              # rad
        self.pose_rate = p('pose_rate', 10.0).value             # Hz
        seed = p('seed', 42).value
        import random
        self.rng = random.Random(seed)

        k = self.scale
        self.get_logger().info(
            f'noise_scale={k:.2f}: odom v/w std = {k*self.s_odom_v:.3f} m/s / '
            f'{k*self.s_odom_w:.3f} rad/s, giroscopio std = {k*self.s_gyro:.3f} rad/s, '
            f'posicao std = {k*self.s_pos:.3f} m, yaw std = {k*self.s_yaw:.3f} rad')

        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/noisy', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_noisy', qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/sensor/pose', qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.create_subscription(Imu, '/imu/out', self.on_imu, qos)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, qos)

        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.last_pose_t = None

    def g(self, std):
        return self.rng.gauss(0.0, std)

    # ---- odometria das rodas -------------------------------------------------
    def on_odom(self, m):
        t = (m.header.stamp.sec +
             m.header.stamp.nanosec * 1e-9)
        v = m.twist.twist.linear.x
        w = m.twist.twist.angular.z
        if self.last_t is None:
            # a odometria ruidosa parte da mesma pose inicial
            self.x = m.pose.pose.position.x
            self.y = m.pose.pose.position.y
            self.th = yaw_of(m.pose.pose.orientation)
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        k = self.scale
        vn = v * (1.0 + k * self.odom_v_bias) + self.g(k * self.s_odom_v)
        wn = w + self.g(k * self.s_odom_w)
        self.th = wrap(self.th + wn * dt)
        self.x += vn * math.cos(self.th) * dt
        self.y += vn * math.sin(self.th) * dt

        o = Odometry()
        o.header = m.header
        o.child_frame_id = m.child_frame_id
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.th / 2.0)
        o.pose.pose.orientation.w = math.cos(self.th / 2.0)
        o.twist.twist.linear.x = vn
        o.twist.twist.angular.z = wn
        # covariancia coerente com o ruido injetado (o filtro usa estes valores)
        sv = max(k * self.s_odom_v, 1e-3)
        sw = max(k * self.s_odom_w, 1e-3)
        o.twist.covariance[0] = sv * sv
        o.twist.covariance[35] = sw * sw
        o.pose.covariance[0] = o.pose.covariance[7] = 0.05
        o.pose.covariance[35] = 0.05
        self.odom_pub.publish(o)

    # ---- giroscopio ------------------------------------------------------------
    def on_imu(self, m):
        k = self.scale
        o = Imu()
        o.header = m.header
        o.header.frame_id = 'base_link'
        o.orientation_covariance[0] = -1.0   # sem orientacao: so a velocidade angular
        o.angular_velocity.z = m.angular_velocity.z + k * self.gyro_bias + self.g(k * self.s_gyro)
        sg = max(k * self.s_gyro, 1e-3)
        o.angular_velocity_covariance[0] = 1e3
        o.angular_velocity_covariance[4] = 1e3
        o.angular_velocity_covariance[8] = sg * sg
        o.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(o)

    # ---- sensor de posicao absoluta ----------------------------------------------
    def on_truth(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_pose_t is not None and t - self.last_pose_t < 1.0 / self.pose_rate:
            return
        self.last_pose_t = t
        k = self.scale
        sp = k * self.s_pos
        sy = k * self.s_yaw
        yaw = wrap(yaw_of(m.pose.pose.orientation) + self.g(sy))
        o = PoseWithCovarianceStamped()
        o.header.stamp = m.header.stamp
        o.header.frame_id = 'odom'
        o.pose.pose.position.x = m.pose.pose.position.x + self.g(sp)
        o.pose.pose.position.y = m.pose.pose.position.y + self.g(sp)
        o.pose.pose.orientation.z = math.sin(yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(yaw / 2.0)
        o.pose.covariance[0] = o.pose.covariance[7] = max(sp * sp, 1e-6)
        o.pose.covariance[35] = max(sy * sy, 1e-6)
        self.pose_pub.publish(o)


def main():
    rclpy.init()
    node = NoisySensors()
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
