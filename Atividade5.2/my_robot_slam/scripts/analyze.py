#!/usr/bin/env python3
"""Metricas e graficos de uma rodada de SLAM.

    analyze.py <slam.csv> <mapa.pgm> --out DIR --label ROTULO [--append resumo.csv] [--title T]

- erro de pose (contra o ground truth): RMSE de posicao e maximo, da odometria sozinha (TF
  odom->base_link) e do SLAM (TF map->base_link). Frame do mapa = frame do mundo, porque o
  robo nasce em (0, 0, 0) e o mapa comeca ali.
- erro do mapa: cada celula ocupada do mapa e comparada com a geometria REAL da sala (paredes
  e obstaculos de worlds/room.world): distancia media ate a superficie mais proxima, e a
  cobertura (fracao da superficie real que o mapa mostrou).
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def true_surface(step=0.01):
    """Pontos amostrados nas superficies reais voltadas para dentro da sala (sala 8 x 6)."""
    pts = []

    def seg(x0, y0, x1, y1):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
        pts.extend(zip(np.linspace(x0, x1, n), np.linspace(y0, y1, n)))

    def rect(cx, cy, w, h):
        seg(cx - w / 2, cy - h / 2, cx + w / 2, cy - h / 2)
        seg(cx + w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        seg(cx + w / 2, cy + h / 2, cx - w / 2, cy + h / 2)
        seg(cx - w / 2, cy + h / 2, cx - w / 2, cy - h / 2)

    rect(0, 0, 8.0, 6.0)          # face interna das paredes
    rect(-1.5, 1.0, 1.0, 0.6)     # box_a
    rect(1.5, -1.0, 0.8, 0.8)     # box_b
    a = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    pts.extend(zip(2.0 + 0.4 * np.cos(a), 1.1 + 0.4 * np.sin(a)))   # cilindro
    return np.array(pts)


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)


def read_map(pgm):
    yaml = pgm.replace('.pgm', '.yaml')
    meta = {}
    for line in open(yaml):
        k, _, v = line.partition(':')
        meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    ox, oy = [float(s) for s in meta['origin'].strip('[]').split(',')[:2]]
    return read_pgm(pgm), res, ox, oy


def map_metrics(pgm):
    img, res, ox, oy = read_map(pgm)
    h = img.shape[0]
    rows, cols = np.nonzero(img == 0)
    occ = np.stack([ox + (cols + 0.5) * res, oy + (h - 1 - rows + 0.5) * res], axis=1)
    surf = true_surface()
    # distancia de cada celula ocupada a superficie real mais proxima (e vice-versa)
    d_occ = np.array([np.min(np.hypot(*(surf - p).T)) for p in occ]) if len(occ) else np.array([9.9])
    cover = np.array([np.min(np.hypot(*(occ - s).T)) if len(occ) else 9.9 for s in surf[::5]])
    return {'map_err_mean': float(np.mean(d_occ)), 'map_err_p95': float(np.percentile(d_occ, 95)),
            'map_cover_10cm': float(np.mean(cover < 0.10)), 'map_cells': len(occ)}, (img, res, ox, oy, occ)


def pose_metrics(d):
    # so amostras em que os dois TF ja existem
    ok = ~np.isnan(d['slam_x']) & ~np.isnan(d['odom_x'])
    e_slam = np.hypot(d['slam_x'] - d['truth_x'], d['slam_y'] - d['truth_y'])[ok]
    e_odom = np.hypot(d['odom_x'] - d['truth_x'], d['odom_y'] - d['truth_y'])[ok]
    dy = np.arctan2(np.sin(d['slam_yaw'] - d['truth_yaw']), np.cos(d['slam_yaw'] - d['truth_yaw']))[ok]
    return e_odom, e_slam, ok, dy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('pgm')
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='run')
    ap.add_argument('--title', default='')
    ap.add_argument('--append')
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    d = {k: np.array([float(r[k]) if r[k] not in ('', 'nan') else math.nan for r in rows]) for k in rows[0]}
    e_odom, e_slam, ok, dyaw = pose_metrics(d)
    mm, (img, res, ox, oy, occ) = map_metrics(a.pgm)
    m = {'rotulo': a.label,
         'odom_rmse': float(np.sqrt(np.mean(e_odom ** 2))), 'odom_max': float(np.max(e_odom)),
         'slam_rmse': float(np.sqrt(np.mean(e_slam ** 2))), 'slam_max': float(np.max(e_slam)),
         'slam_yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(dyaw ** 2)))), **mm}
    txt = (f"{a.label}\n  odometria sozinha : RMSE {m['odom_rmse']:.3f} m   max {m['odom_max']:.3f} m\n"
           f"  SLAM (map->base)  : RMSE {m['slam_rmse']:.3f} m   max {m['slam_max']:.3f} m   "
           f"yaw RMSE {m['slam_yaw_rmse_deg']:.2f} graus\n"
           f"  mapa              : erro medio {100*m['map_err_mean']:.1f} cm   p95 {100*m['map_err_p95']:.1f} cm   "
           f"cobertura(10cm) {100*m['map_cover_10cm']:.1f} %   celulas ocupadas {m['map_cells']}")
    print(txt)
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'metricas.txt'), 'w').write(txt + '\n')
    if a.append:
        new = not os.path.exists(a.append)
        with open(a.append, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(m))
            if new:
                w.writeheader()
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in m.items()})

    # ---- figura: mapa + trajetorias, erro no tempo ----
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
    ax[0].imshow(img, cmap='gray', extent=extent, origin='upper', vmin=0, vmax=255)
    sf = true_surface(0.02)
    ax[0].plot(sf[:, 0], sf[:, 1], '.', ms=1, color='tab:green', alpha=0.5, label='geometria real')
    ax[0].plot(d['truth_x'], d['truth_y'], 'g-', lw=1, label='trajetoria real')
    ax[0].plot(d['slam_x'], d['slam_y'], 'b--', lw=1, label='SLAM')
    ax[0].plot(d['odom_x'], d['odom_y'], 'r:', lw=1, label='odometria')
    ax[0].set_aspect('equal'); ax[0].set_xlim(-5, 5); ax[0].set_ylim(-4, 4)
    ax[0].set_title('mapa construido'); ax[0].legend(fontsize=7, loc='lower right')
    t = d['t']
    ax[1].plot(t[ok], e_odom, 'r', label='odometria sozinha')
    ax[1].plot(t[ok], e_slam, 'b', label='SLAM')
    ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('erro de posicao [m]'); ax[1].grid(alpha=.3); ax[1].legend()
    ax[1].set_title('erro de pose vs ground truth')
    ax[2].axis('off')
    ax[2].text(0, 0.95, a.title or a.label, fontsize=11, va='top', weight='bold')
    ax[2].text(0, 0.80, txt.split('\n', 1)[1], fontsize=9, va='top', family='monospace')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'resultado.png'), dpi=110)


if __name__ == '__main__':
    main()
