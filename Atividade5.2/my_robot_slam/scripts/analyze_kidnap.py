#!/usr/bin/env python3
"""Erro do SLAM (map->base_link) contra o ground truth numa rodada de robo sequestrado.

    analyze_kidnap.py <slam.csv> --out DIR --label ROTULO

O instante do sequestro e detectado como o pulo do ground truth (> 1 m entre duas amostras).
Recuperou = o erro de posicao fica < 0.30 m ate o fim da rodada.
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='run')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    d = {k: np.array([float(r[k]) if r[k] not in ('', 'nan') else math.nan for r in rows]) for k in rows[0]}
    t = d['t']
    err = np.hypot(d['slam_x'] - d['truth_x'], d['slam_y'] - d['truth_y'])
    jump = np.hypot(np.diff(d['truth_x']), np.diff(d['truth_y']))
    k = int(np.argmax(jump)) + 1 if jump.max() > 1.0 else None
    lines = [a.label]
    if k is None:
        lines.append('  nao houve sequestro (ground truth sem pulo)')
    else:
        tk = t[k]
        pre = err[:k][~np.isnan(err[:k])]
        post = err[k:]
        lines.append(f'  sequestro em t = {tk:.1f} s: de ({d["truth_x"][k-1]:.2f}, {d["truth_y"][k-1]:.2f}) '
                     f'para ({d["truth_x"][k]:.2f}, {d["truth_y"][k]:.2f})')
        lines.append(f'  erro do SLAM antes : medio {np.nanmean(pre):.3f} m')
        lines.append(f'  erro logo apos     : {post[0]:.2f} m')
        lines.append(f'  erro 60 s depois   : {post[-1]:.2f} m   (maximo {np.nanmax(post):.2f} m)')
        ok = np.where(post > 0.30)[0]
        rec = len(ok) == 0 or ok[-1] < len(post) - 1
        t_rec = (t[k:][ok[-1] + 1] - tk) if (len(ok) and ok[-1] < len(post) - 1) else 0.0
        lines.append(('  RECUPEROU a localizacao' + (f' apos {t_rec:.1f} s' if t_rec else '')) if rec
                     else '  NAO recuperou a localizacao (erro continua > 0.30 m)')
    txt = '\n'.join(lines)
    print(txt)
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'metricas.txt'), 'w').write(txt + '\n')
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
    ax[0].plot(t, err, 'b')
    if k is not None:
        ax[0].axvline(t[k], color='r', ls='--', label='sequestro')
        ax[0].legend()
    ax[0].axhline(0.30, color='gray', ls=':')
    ax[0].set_xlabel('t [s de simulacao]'); ax[0].set_ylabel('erro de posicao do SLAM [m]')
    ax[0].grid(alpha=.3); ax[0].set_title(a.label)
    ax[1].plot(d['truth_x'], d['truth_y'], 'g.', ms=3, label='real')
    ax[1].plot(d['slam_x'], d['slam_y'], 'b.', ms=3, label='SLAM')
    ax[1].set_aspect('equal'); ax[1].set_xlim(-4.5, 4.5); ax[1].set_ylim(-3.5, 3.5)
    ax[1].legend(); ax[1].grid(alpha=.3); ax[1].set_title('posicao real x estimada')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'resultado.png'), dpi=110)


if __name__ == '__main__':
    main()
