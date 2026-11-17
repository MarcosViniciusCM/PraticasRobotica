#!/usr/bin/env python3
"""Salva o /map atual em <prefixo>.pgm + <prefixo>.yaml (formato do map_server).

    ros2 run my_robot_slam save_map.py /caminho/prefixo
"""

import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'map'
    rclpy.init()
    node = Node('save_map')
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', got.append, qos)
    while not got and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    m = got[0]
    w, h = m.info.width, m.info.height
    g = np.array(m.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 205, dtype=np.uint8)       # desconhecido: cinza
    img[g == 0] = 254                                # livre: branco
    img[g > 50] = 0                                  # ocupado: preto
    img = np.flipud(img)                             # PGM: linha 0 = topo
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    o = m.info.origin.position
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {prefix.split("/")[-1]}.pgm\nmode: trinary\nresolution: {m.info.resolution}\n'
                f'origin: [{o.x}, {o.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'mapa {w}x{h} ({m.info.resolution} m/celula) salvo em {prefix}.pgm/.yaml')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
