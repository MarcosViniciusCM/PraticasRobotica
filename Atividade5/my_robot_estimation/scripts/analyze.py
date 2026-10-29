#!/usr/bin/env python3
"""Metricas e graficos do experimento a partir do CSV do state_logger.

    ros2 run my_robot_estimation analyze.py /tmp/estado.csv [--out pasta] [--title texto]

Imprime o RMSE de posicao (m) e de yaw (rad) de cada fonte CONTRA O GROUND TRUTH:
  raw   odometria ruidosa integrada (dead reckoning)
  meas  sensor de posicao absoluta
  ekf   filtro de Kalman
e o erro de seguimento (ground truth x trajetoria desejada).
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

COR = {'truth': '#2ca02c', 'raw': '#d62728', 'meas': '#ff7f0e', 'ekf': '#1f77b4', 'des': 'k'}
NOME = {'truth': 'Estado real (ground truth Gazebo)', 'raw': 'Bruto: odometria ruidosa',
        'meas': 'Bruto: sensor de posicao', 'ekf': 'Estimado (Kalman)', 'des': 'Trajetoria desejada'}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    # so o trecho em que o 8 esta sendo executado (a referencia existe)
    ok = ~np.isnan(d['des_x'])
    return {k: d[k][ok] for k in d.dtype.names}


def rmse(d, a, b='truth'):
    e = np.hypot(d[f'{a}_x'] - d[f'{b}_x'], d[f'{a}_y'] - d[f'{b}_y'])
    ey = wrap(d[f'{a}_yaw'] - d[f'{b}_yaw'])
    return float(np.sqrt(np.nanmean(e ** 2))), float(np.sqrt(np.nanmean(ey ** 2))), e


def metrics(d):
    out = {}
    for s in ('raw', 'meas', 'ekf'):
        p, y, _ = rmse(d, s)
        out[s] = (p, y)
    p, y, _ = rmse(d, 'truth', 'des')
    out['track'] = (p, y)
    return out


def plots(d, outdir, title):
    t = d['t'] - d['t'][0]
    fig, ax = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={'width_ratios': [1, 1]})
    a = ax[0]
    for s, lw, al in (('raw', 0.8, 0.6), ('meas', 0.8, 0.6), ('truth', 2.0, 1), ('ekf', 1.4, 1)):
        a.plot(d[f'{s}_x'], d[f'{s}_y'], color=COR[s], lw=lw, alpha=al, label=NOME[s])
    a.plot(d['des_x'], d['des_y'], 'k--', lw=1.3, label=NOME['des'])
    a.set_aspect('equal'); a.grid(alpha=.3); a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
    a.set_title('Plano XY'); a.legend(fontsize=8, loc='upper right')
    for i, c in enumerate(('x', 'y')):
        pass
    b = ax[1]
    _, _, e_raw = rmse(d, 'raw'); _, _, e_meas = rmse(d, 'meas'); _, _, e_ekf = rmse(d, 'ekf')
    _, _, e_trk = rmse(d, 'truth', 'des')
    b.plot(t, e_raw, color=COR['raw'], label='odometria ruidosa'); b.plot(t, e_meas, color=COR['meas'], label='sensor de posicao')
    b.plot(t, e_ekf, color=COR['ekf'], lw=1.6, label='Kalman'); b.plot(t, e_trk, 'k--', lw=1, label='real x desejada (controle)')
    b.set_xlabel('t [s]'); b.set_ylabel('erro de posicao [m]'); b.grid(alpha=.3)
    b.set_title('Erro contra o ground truth'); b.legend(fontsize=8)
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(os.path.join(outdir, 'trajetoria_erro.png'), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for a, c, lab in zip(ax, ('x', 'y', 'yaw'), ('x [m]', 'y [m]', 'yaw [rad]')):
        for s, lw, al in (('raw', .8, .6), ('meas', .8, .6), ('truth', 2, 1), ('ekf', 1.3, 1)):
            v = d[f'{s}_{c}']
            a.plot(t, v if c != 'yaw' else np.unwrap(v), color=COR[s], lw=lw, alpha=al, label=NOME[s])
        v = d[f'des_{c}']
        a.plot(t, v if c != 'yaw' else np.unwrap(v), 'k--', lw=1, label=NOME['des'])
        a.set_ylabel(lab); a.grid(alpha=.3)
    ax[0].legend(fontsize=8, ncol=3); ax[-1].set_xlabel('t [s]'); fig.suptitle(title)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, 'series.png'), dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv'); ap.add_argument('--out', default='.'); ap.add_argument('--title', default='')
    ap.add_argument('--label', default='')
    ap.add_argument('--append', default='', help='CSV de resumo onde acrescentar a linha')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    d = load(a.csv)
    m = metrics(d)
    print(f'{a.label}  ({len(d["t"])} amostras, {d["t"][-1]-d["t"][0]:.1f} s)')
    for k, nome in (('raw', 'odometria ruidosa'), ('meas', 'sensor de posicao'),
                    ('ekf', 'Kalman'), ('track', 'real x desejada')):
        print(f'  {nome:20s} RMSE pos = {m[k][0]:.4f} m   yaw = {m[k][1]:.4f} rad')
    if a.append:
        new = not os.path.exists(a.append)
        with open(a.append, 'a') as f:
            if new:
                f.write('label,rmse_raw_m,rmse_meas_m,rmse_ekf_m,rmse_ekf_yaw_rad,rmse_raw_yaw_rad,track_rmse_m\n')
            f.write(f'{a.label},{m["raw"][0]:.4f},{m["meas"][0]:.4f},{m["ekf"][0]:.4f},'
                    f'{m["ekf"][1]:.4f},{m["raw"][1]:.4f},{m["track"][0]:.4f}\n')
    plots(d, a.out, a.title or a.label)


if __name__ == '__main__':
    main()
