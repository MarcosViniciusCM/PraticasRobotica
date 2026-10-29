#!/usr/bin/env python3
"""Monta o video da pratica 05.1 a partir dos resultados reais dos experimentos.

    python3 make_video.py [--out video/pratica_05_1.mp4]

Nao e gravacao de tela: as animacoes sao desenhadas com os CSV gravados pelo
state_logger durante as simulacoes (results/*/estado.csv), e o terminal mostrado
e a saida real dos comandos (results/captura/*.txt). Cada cena tem a legenda com
o texto da narracao; o mesmo texto sai em roteiro_video.md para ser lido em voz alta.
"""

import argparse
import csv
import os
import subprocess
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE) if os.path.basename(HERE) == 'scripts' else HERE
SRC = os.environ.get('PKG_SRC', os.path.expanduser('~/ros2_ws/src/Atividade4/my_robot_estimation'))
RES = os.path.join(SRC, 'results')

W, H, DPI, FPS = 1280, 720, 100, 25
BG, FG, MUTED = '#fafafa', '#1b1f24', '#5b6570'
C = {'truth': '#2ca02c', 'raw': '#d62728', 'meas': '#ff9f1c', 'ekf': '#1f77b4', 'des': '#111111'}
NAME = {'truth': 'Estado real (ground truth do Gazebo)', 'raw': 'Bruto: odometria das rodas (ruidosa)',
        'meas': 'Bruto: sensor de posição (ruidoso)', 'ekf': 'Estimado: filtro de Kalman',
        'des': 'Trajetória desejada'}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'axes.facecolor': 'white'})


# ----------------------------------------------------------------------------- dados
def load(label):
    d = np.genfromtxt(os.path.join(RES, label, 'estado.csv'), delimiter=',', names=True)
    ok = ~np.isnan(d['des_x'])
    return {k: d[k][ok] for k in d.dtype.names}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def summary():
    with open(os.path.join(RES, 'resumo.csv')) as f:
        return {r['label']: {k: float(v) for k, v in r.items() if k != 'label'}
                for r in csv.DictReader(f)}


# ----------------------------------------------------------------------------- video
class Writer:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.p = subprocess.Popen(
            ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgba',
             '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
             '-crf', '20', '-preset', 'medium', '-movflags', '+faststart', path],
            stdin=subprocess.PIPE)
        self.frames = 0

    def frame(self, fig, n=1):
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba()).tobytes()
        for _ in range(n):
            self.p.stdin.write(buf)
        self.frames += n

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def base_fig():
    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)
    return fig


def header(fig, tag, title):
    fig.text(0.035, 0.945, tag, fontsize=12, color='white', weight='bold', va='center',
             bbox=dict(boxstyle='round,pad=0.4', fc='#1f77b4', ec='none'))
    fig.text(0.035 + 0.012 + 0.0095 * len(tag), 0.945, title, fontsize=20, weight='bold',
             color=FG, va='center')


def caption(fig, text):
    fig.patches.append(FancyBboxPatch((0.0, 0.0), 1.0, 0.135, transform=fig.transFigure,
                                      boxstyle='square,pad=0', fc='#14181d', ec='none', zorder=5))
    wrapped = '\n'.join(textwrap.wrap(text, 118))
    fig.text(0.03, 0.067, wrapped, fontsize=13.5, color='white', va='center', zorder=6,
             linespacing=1.35)


def code_box(fig, rect, text, fs=11.5):
    ax = fig.add_axes(rect)
    ax.set_facecolor('#0f1720')
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.text(0.03, 0.96, text, family='DejaVu Sans Mono', fontsize=fs, color='#e6edf3',
            va='top', ha='left', linespacing=1.45, transform=ax.transAxes)
    return ax


# ----------------------------------------------------------------------------- cenas
def scene_title(w, scenes):
    fig = base_fig()
    fig.text(0.5, 0.62, 'Prática 05.1 — Estimação de estados', fontsize=34, weight='bold',
             ha='center', color=FG)
    fig.text(0.5, 0.52, 'Filtro de Kalman (robot_localization) e seguimento da trajetória em 8\n'
                        'com o estado estimado', fontsize=19, ha='center', color=MUTED, linespacing=1.5)
    fig.text(0.5, 0.33, 'Robótica · Prof. Davi Santos', fontsize=15, ha='center', color=MUTED)
    caption(fig, scenes['titulo'])
    w.frame(fig, 6 * FPS)
    plt.close(fig)


def box(ax, xy, wh, text, fc, ec='#444', fs=11, tc=FG, weight='normal'):
    x, y = xy
    ww, hh = wh
    ax.add_patch(FancyBboxPatch((x, y), ww, hh, boxstyle='round,pad=0.01,rounding_size=0.02',
                                fc=fc, ec=ec, lw=1.4))
    ax.text(x + ww / 2, y + hh / 2, text, ha='center', va='center', fontsize=fs, color=tc,
            weight=weight, linespacing=1.35)


def arrow(ax, a, b, color='#444', lw=1.6, style='-|>'):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=14, color=color, lw=lw))


def scene_arch(w, scenes):
    fig = base_fig()
    header(fig, 'Exercício 1.1', 'Estimando estados com o filtro de Kalman')
    ax = fig.add_axes([0.02, 0.16, 0.96, 0.72]); ax.set_xlim(-0.03, 1.03); ax.set_ylim(0, 1); ax.axis('off')
    box(ax, (0.00, 0.62), (0.20, 0.30), 'Gazebo\n(robô simulado)\n\nodometria das rodas\ngiroscópio da IMU\nground truth', '#e8f5e9')
    box(ax, (0.27, 0.62), (0.20, 0.30), 'noisy_sensors.py\n\nacrescenta ruído\n× noise_scale', '#fff3e0')
    box(ax, (0.27, 0.12), (0.20, 0.38), '/odom/noisy\n   vx, ω\n/imu/data_noisy\n   ω (giroscópio)\n/sensor/pose\n   x, y, θ absolutos',
        '#ffebee', fs=10.5)
    box(ax, (0.56, 0.30), (0.19, 0.40), 'ekf_filter_node\n(robot_localization)\n\nFiltro de Kalman\nEstendido 2D', '#e3f2fd',
        weight='bold')
    box(ax, (0.82, 0.30), (0.18, 0.40), '/odometry/filtered\n\nx, y, θ\nvx, ω\n+ covariância', '#bbdefb', weight='bold')
    arrow(ax, (0.20, 0.77), (0.27, 0.77)); arrow(ax, (0.37, 0.62), (0.37, 0.50))
    arrow(ax, (0.47, 0.31), (0.56, 0.44)); arrow(ax, (0.75, 0.50), (0.82, 0.50))
    ax.text(0.00, 0.50, 'Predição: modelo cinemático\n+ velocidades da odometria e do giroscópio\n\n'
            'Correção: posição absoluta (x, y, θ)\nque impede a deriva da odometria', fontsize=11, color=MUTED,
            va='top', linespacing=1.4)
    caption(fig, scenes['arq'])
    w.frame(fig, 24 * FPS)
    plt.close(fig)


EKF_SNIPPET = '''ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true         # robô planar: x, y, yaw
    publish_tf: false        # o TF odom->base_link é do diff_drive
    world_frame: odom

    odom0: /odom/noisy       # só as velocidades (vx, vyaw)
    odom0_config: [F,F,F, F,F,F, T,F,F, F,F,T, F,F,F]
    imu0:  /imu/data_noisy   # giroscópio (vyaw)
    imu0_config:  [F,F,F, F,F,F, F,F,F, F,F,T, F,F,F]
    pose0: /sensor/pose      # posição absoluta (x, y, yaw)
    pose0_config: [T,T,F, F,F,T, F,F,F, F,F,F, F,F,F]

    process_noise_covariance: [...]   # Q - confiança no modelo'''


def scene_config(w, scenes):
    fig = base_fig()
    header(fig, 'Exercício 1.1', 'Configuração do filtro (config/ekf.yaml)')
    code_box(fig, [0.03, 0.17, 0.60, 0.70], EKF_SNIPPET, fs=11)
    fig.text(0.66, 0.80, 'Por que assim?', fontsize=17, weight='bold', color=FG)
    pts = ['• A pose da odometria é a integral de uma\n  velocidade com ruído: ela deriva sem limite.\n  Por isso só as velocidades entram.',
           '• O giroscópio reforça a velocidade angular.',
           '• O sensor de posição absoluta (câmera / GPS)\n  tem ruído branco, mas não deriva: ele ancora\n  a estimativa.',
           '• Cada sensor chega com a sua covariância R;\n  a matriz Q define quanto confiar no modelo.']
    fig.text(0.66, 0.74, '\n\n'.join(pts), fontsize=12, color=FG, va='top', linespacing=1.35)
    caption(fig, scenes['cfg'])
    w.frame(fig, 20 * FPS)
    plt.close(fig)


def scene_terminal(w, scenes):
    cap = os.path.join(RES, 'captura')
    rd = lambda n: open(os.path.join(cap, n)).read().rstrip()
    import re
    raw_echo = rd('echo.txt')
    num = lambda blk, key: re.search(rf'{blk}[\s\S]*?\b{key}: (\S+)', raw_echo).group(1)
    echo = ['header.frame_id: odom   child_frame_id: base_link',
            f'pose.pose.position:     x: {float(num("position", "x")):+.4f}   y: {float(num("position", "y")):+.4f}',
            f'pose.pose.orientation:  z: {float(num("orientation", "z")):+.4f}   w: {float(num("orientation", "w")):+.4f}',
            f'twist.twist.linear:     x: {float(num("linear", "x")):+.4f}',
            f'twist.twist.angular:    z: {float(num("angular", "z")):+.4f}',
            'pose.covariance / twist.covariance: [36 valores cada]']
    blocks = [
        ('$ ros2 launch my_robot_estimation estimation.launch.py', rd('noise.txt').split(']: ')[-1][:100].rsplit(',', 1)[0] + ' ...'),
        ('$ ros2 node list | grep -E "ekf|noisy|ground"', '\n'.join(l for l in rd('nodes.txt').splitlines() if any(k in l for k in ('ekf', 'noisy', 'ground')))),
        ('$ ros2 topic hz /odometry/filtered', rd('hz.txt').splitlines()[0]),
        ('$ ros2 topic echo /odometry/filtered --once', '\n'.join(echo)),
    ]
    for k in range(1, len(blocks) + 1):
        fig = base_fig()
        header(fig, 'Exercício 1.1', '/odometry/filtered está publicando')
        txt = ''
        for cmd, out in blocks[:k]:
            txt += f'{cmd}\n{out}\n\n'
        code_box(fig, [0.03, 0.17, 0.94, 0.72], txt.rstrip(), fs=10.5)
        caption(fig, scenes['term'])
        w.frame(fig, int(4.5 * FPS) if k < len(blocks) else 7 * FPS)
        plt.close(fig)


def robot_poly(x, y, yaw, L=0.40, Wd=0.26):
    # base_link fica 0.1 m atrás do eixo; o chassi (0.4 x 0.2) é centrado nele
    c, s = np.cos(yaw), np.sin(yaw)
    pts = np.array([[L / 2, Wd / 2], [L / 2, -Wd / 2], [-L / 2, -Wd / 2], [-L / 2, Wd / 2]])
    R = np.array([[c, -s], [s, c]])
    return (pts @ R.T) + np.array([x, y])


def scene_anim(w, label, tag, title, cap_text, speed, noise_label, show_raw=True, hold=4):
    d = load(label)
    t = d['t'] - d['t'][0]
    # erros contra o ground truth
    err = {s: np.hypot(d[f'{s}_x'] - d['truth_x'], d[f'{s}_y'] - d['truth_y']) for s in ('raw', 'meas', 'ekf')}
    err['track'] = np.hypot(d['truth_x'] - d['des_x'], d['truth_y'] - d['des_y'])

    fig = base_fig()
    header(fig, tag, title)
    ax = fig.add_axes([0.05, 0.23, 0.41, 0.64])
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.7, 1.7); ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('Plano XY', fontsize=12)
    ax.plot(d['des_x'], d['des_y'], '--', color=C['des'], lw=1.2, label=NAME['des'])
    ln = {}
    for s, lw, al in (('raw', 0.9, 0.6), ('meas', 0.9, 0.7), ('truth', 2.4, 1), ('ekf', 1.6, 1)):
        if s == 'raw' and not show_raw:
            continue
        (ln[s],) = ax.plot([], [], color=C[s], lw=lw, alpha=al, label=NAME[s])
    poly = Polygon(robot_poly(0, 0, 0), closed=True, fc=C['truth'], ec='k', alpha=.9, zorder=5)
    ax.add_patch(poly)
    ax.legend(fontsize=7.6, loc='upper left', framealpha=.92)

    bx = fig.add_axes([0.55, 0.23, 0.42, 0.27])
    bx.set_xlim(0, t[-1]); bx.set_ylim(0, 1.0 if noise_label.startswith('5') else 0.6)
    bx.set_xlabel('t [s]'); bx.set_ylabel('erro [m]'); bx.grid(alpha=.3)
    bx.set_title('Erro de posição contra o ground truth', fontsize=12)
    el = {}
    for s, col, lab, lw in (('meas', C['meas'], 'sensor de posição', 0.9), ('ekf', C['ekf'], 'Kalman', 1.6),
                            ('track', C['des'], 'real × desejada (controle)', 1.0)):
        (el[s],) = bx.plot([], [], color=col, lw=lw, label=lab,
                           ls='--' if s == 'track' else '-')
    bx.legend(fontsize=8, loc='upper right', ncol=3)

    cx = fig.add_axes([0.54, 0.55, 0.43, 0.32]); cx.axis('off')
    txt = cx.text(0.0, 1.0, '', fontsize=12.5, va='top', family='DejaVu Sans Mono', linespacing=1.55, color=FG)
    caption(fig, cap_text)

    step = speed / FPS
    n = int(t[-1] / step) + 1
    for k in range(n + hold * FPS):
        i = min(np.searchsorted(t, min(k * step, t[-1])), len(t) - 1)
        for s in ln:
            ln[s].set_data(d[f'{s}_x'][:i + 1], d[f'{s}_y'][:i + 1])
        poly.set_xy(robot_poly(d['truth_x'][i], d['truth_y'][i], d['truth_yaw'][i]))
        for s in el:
            el[s].set_data(t[:i + 1], err[s][:i + 1])
        rm = lambda s: np.sqrt(np.mean(err[s][:i + 1] ** 2))
        lines = [f'ruído dos sensores  ×{noise_label}', f't = {t[i]:5.1f} s', '',
                 f'RMSE sensor de posição : {rm("meas"):.3f} m',
                 f'RMSE filtro de Kalman  : {rm("ekf"):.3f} m']
        if show_raw:
            lines.insert(4, f'RMSE odometria bruta   : {rm("raw"):.3f} m')
        lines.append(f'real × desejada        : {rm("track"):.3f} m')
        txt.set_text('\n'.join(lines))
        w.frame(fig)
    plt.close(fig)


def scene_control(w, scenes):
    fig = base_fig()
    header(fig, 'Exercício 2.1', 'Trajetória em 8 com o estado estimado')
    ax = fig.add_axes([0.02, 0.50, 0.96, 0.38]); ax.set_xlim(-0.03, 1.03); ax.set_ylim(0, 1); ax.axis('off')
    box(ax, (0.00, 0.20), (0.17, 0.60), 'figure8_tracker\n\nx_d(t) = A sen(Ωt)\ny_d(t) = B sen(Ωt) cos(Ωt)', '#fff8e1', fs=10.5)
    box(ax, (0.26, 0.20), (0.18, 0.60), 'cmd_vel\n(v, ω)', '#eeeeee')
    box(ax, (0.53, 0.20), (0.15, 0.60), 'Gazebo\n(robô real)', '#e8f5e9')
    box(ax, (0.77, 0.20), (0.22, 0.60), 'noisy_sensors\n+ ekf_filter_node\n\n/odometry/filtered', '#e3f2fd', weight='bold')
    arrow(ax, (0.17, 0.5), (0.26, 0.5)); arrow(ax, (0.44, 0.5), (0.53, 0.5)); arrow(ax, (0.68, 0.5), (0.77, 0.5))
    arrow(ax, (0.88, 0.20), (0.88, 0.03), color='#1f77b4'); arrow(ax, (0.88, 0.03), (0.085, 0.03), color='#1f77b4', style='-')
    arrow(ax, (0.085, 0.03), (0.085, 0.20), color='#1f77b4')
    ax.text(0.48, 0.07, 'realimentação: o controle lê o estado ESTIMADO', color='#1f77b4', fontsize=11, ha='center',
            weight='bold')
    code_box(fig, [0.03, 0.17, 0.46, 0.27],
             '# o tracker da prática 03, só trocando o tópico de entrada\n'
             'Node(package="my_robot_trajectory",\n'
             '     executable="figure8_tracker",\n'
             '     remappings=[("odom", "/odometry/filtered")])', fs=10.5)
    fig.text(0.52, 0.43, 'O que precisou ser ajustado', fontsize=14, weight='bold', color=FG, va='top')
    fig.text(0.52, 0.395,
             '• Ground truth do Gazebo (libgazebo_ros_p3d) só para os gráficos.\n'
             '• Calibração: o chassi girava ~17 % menos que a odometria\n'
             '  dizia -> wheel_separation_multiplier = 1.2.\n'
             '• Ω = 0.25 rad/s: com Ω = 0.5 o robô não segue nem com\n'
             '  estado perfeito (limite da planta, não do filtro).\n'
             '• /clock a 100 Hz, senão o EKF só publica a 10 Hz.',
             fontsize=11, color=FG, va='top', linespacing=1.45)
    caption(fig, scenes['ctrl'])
    w.frame(fig, 24 * FPS)
    plt.close(fig)


def scene_compare(w, scenes):
    s = summary()
    fig = base_fig()
    header(fig, 'Exercício 2.1', 'Comparação: o que o controle enxerga × o que acontece')
    a1 = fig.add_axes([0.06, 0.26, 0.40, 0.56])
    labs = ['ruído ×1', 'ruído ×5']
    x = np.arange(2); wd = 0.26
    a1.bar(x - wd, [s['e1_kalman_ruido1']['rmse_meas_m'], s['e2_kalman_ruido5']['rmse_meas_m']], wd, color=C['meas'], label='sensor de posição')
    a1.bar(x, [s['e1_kalman_ruido1']['rmse_ekf_m'], s['e2_kalman_ruido5']['rmse_ekf_m']], wd, color=C['ekf'], label='filtro de Kalman')
    a1.bar(x + wd, [s['e3_bruto_ruido1']['rmse_raw_m'], s['e4_bruto_ruido5']['rmse_raw_m']], wd, color=C['raw'], label='odometria bruta*')
    for xi, v in zip(np.concatenate([x - wd, x, x + wd]),
                     [s['e1_kalman_ruido1']['rmse_meas_m'], s['e2_kalman_ruido5']['rmse_meas_m'],
                      s['e1_kalman_ruido1']['rmse_ekf_m'], s['e2_kalman_ruido5']['rmse_ekf_m'],
                      s['e3_bruto_ruido1']['rmse_raw_m'], s['e4_bruto_ruido5']['rmse_raw_m']]):
        a1.text(xi, v + 0.012, f'{v:.2f}', ha='center', fontsize=9)
    a1.set_xticks(x); a1.set_xticklabels(labs); a1.set_ylabel('RMSE da estimativa [m]'); a1.grid(axis='y', alpha=.3)
    a1.set_title('Erro da ESTIMATIVA (contra o ground truth)', fontsize=12); a1.legend(fontsize=9)
    fig.text(0.06, 0.17, '* odometria bruta medida nas rodadas em que o robô foi controlado por ela', fontsize=8.5, color=MUTED)

    a2 = fig.add_axes([0.56, 0.26, 0.40, 0.56])
    vals = [s['e1_kalman_ruido1']['track_rmse_m'], s['e3_bruto_ruido1']['track_rmse_m'],
            s['e2_kalman_ruido5']['track_rmse_m'], s['e4_bruto_ruido5']['track_rmse_m']]
    cols = [C['ekf'], C['raw'], C['ekf'], C['raw']]
    xs = [0, 1, 2.6, 3.6]
    a2.bar(xs, vals, 0.8, color=cols)
    for xi, v in zip(xs, vals):
        a2.text(xi, v + 0.012, f'{v:.2f} m', ha='center', fontsize=10, weight='bold')
    a2.set_xticks(xs); a2.set_xticklabels(['Kalman', 'bruto', 'Kalman', 'bruto'])
    a2.text(0.5, -0.13, 'ruído ×1', ha='center', transform=a2.get_xaxis_transform(), fontsize=11, color=MUTED)
    a2.text(3.1, -0.13, 'ruído ×5', ha='center', transform=a2.get_xaxis_transform(), fontsize=11, color=MUTED)
    a2.set_ylabel('erro REAL de seguimento [m]'); a2.grid(axis='y', alpha=.3)
    a2.set_title('Robô × trajetória desejada, controlando com…', fontsize=12)
    caption(fig, scenes['cmp'])
    w.frame(fig, 26 * FPS)
    plt.close(fig)


def scene_end(w, scenes):
    fig = base_fig()
    header(fig, 'Conclusão', 'O que ficou')
    pts = ['• O filtro de Kalman (robot_localization) funde odometria, giroscópio e posição absoluta e publica a pose\n'
           '   (x, y, θ) em /odometry/filtered a 50 Hz.',
           '• Com ruído ×1 e ×5 a estimativa foi melhor que qualquer sensor sozinho.',
           '• Controlando com o estado estimado, o robô seguiu o 8 com 0.07 m (ruído ×1) e 0.11 m (ruído ×5) de erro real.',
           '• Controlando com a odometria bruta, o controle "achava" que estava a 0.01–0.04 m, mas o robô real ficou\n'
           '   a 0.32–0.58 m da trajetória: sem sensor absoluto, o erro não é observável.',
           '• Lição prática: o controle é tão bom quanto o estado que ele recebe.']
    fig.text(0.05, 0.80, '\n\n'.join(pts), fontsize=14.5, color=FG, va='top', linespacing=1.4)
    caption(fig, scenes['fim'])
    w.frame(fig, 16 * FPS)
    plt.close(fig)


# ----------------------------------------------------------------------------- texto da narracao
SCENES = {
    'titulo': 'Nesta prática eu uso o filtro de Kalman do pacote robot_localization para estimar a pose do robô, e depois '
              'refaço o seguimento da trajetória em 8, agora controlando com o estado estimado e com os sensores bem mais ruidosos.',
    'arq': 'Exercício 1.1. O Gazebo fornece a odometria das rodas, o giroscópio da IMU e o ground truth. Um nó acrescenta ruído '
           'gaussiano controlado por um parâmetro, o noise_scale. O ekf_filter_node recebe as velocidades da odometria e do giroscópio '
           'e uma medida de posição absoluta, e publica a pose estimada, x, y e theta, em /odometry/filtered.',
    'cfg': 'No ekf.yaml o filtro roda em modo 2D a 50 hertz. A odometria entra só com as velocidades, porque a pose dela é uma integral '
           'ruidosa e deriva. O giroscópio reforça a velocidade angular e o sensor de posição absoluta ancora x, y e yaw. '
           'A matriz Q diz quanto confiar no modelo, e cada sensor traz a sua covariância R.',
    'term': 'Aqui está a saída real dos comandos. O ekf_filter_node está no ar junto com os sensores ruidosos, o tópico '
            '/odometry/filtered publica a cerca de 50 hertz, e a mensagem traz posição, orientação e velocidades estimadas.',
    'anim1': 'Esta é a simulação com ruído padrão. Em verde está o estado real; em laranja, o sensor de posição ruidoso; '
             'em vermelho, a odometria bruta, que deriva para longe; e em azul, a estimativa do filtro, que fica colada no real. '
             'O RMSE do Kalman é menor que o do melhor sensor sozinho.',
    'ctrl': 'Exercício 2.1. Refiz o experimento do 8 trocando apenas o tópico que o controlador lê: em vez da odometria, ele usa '
            '/odometry/filtered. Para o robô realmente seguir a trajetória foi preciso calibrar a separação das rodas e usar '
            'ômega igual a 0.25, porque com ômega 0.5 o robô não segue o 8 nem com o estado perfeito.',
    'anim2': 'Agora aumentei o ruído cinco vezes: o sensor de posição passa a ter desvio de 25 centímetros e a nuvem laranja '
             'espalha tudo. O filtro ainda reduz o erro de 0.36 para 0.12 metros, e o robô, controlado pela estimativa, '
             'continua desenhando o 8, com erro real de 11 centímetros.',
    'cmp': 'Comparando com o controle usando a odometria bruta: o controle acha que está a poucos centímetros da trajetória, '
           'mas o robô real fica a 32 centímetros com ruído padrão e 58 centímetros com ruído cinco vezes maior. '
           'Com o Kalman, o erro real fica em 7 e 11 centímetros.',
    'fim': 'Resumindo: o filtro de Kalman fundiu os sensores, publicou a pose em /odometry/filtered, e controlar com o estado '
           'estimado mantém o robô na trajetória mesmo com muito ruído. O controle é tão bom quanto o estado que ele recebe.',
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(SRC, 'video', 'pratica_05_1.mp4'))
    a = ap.parse_args()
    w = Writer(a.out)
    marks = []

    def mark(name):
        marks.append((name, w.frames / FPS))

    mark('titulo'); scene_title(w, SCENES)
    mark('arq'); scene_arch(w, SCENES)
    mark('cfg'); scene_config(w, SCENES)
    mark('term'); scene_terminal(w, SCENES)
    mark('anim1')
    scene_anim(w, 'e1_kalman_ruido1', 'Exercício 1.1', 'Estimativa x sensores (ruído ×1)', SCENES['anim1'], 2.0, '1')
    mark('ctrl'); scene_control(w, SCENES)
    mark('anim2')
    scene_anim(w, 'e2_kalman_ruido5', 'Exercício 2.1', 'Controle com o Kalman, ruído ×5', SCENES['anim2'], 2.0, '5')
    mark('cmp'); scene_compare(w, SCENES)
    mark('fim'); scene_end(w, SCENES)
    w.close()

    with open(os.path.join(os.path.dirname(a.out), 'roteiro_video.md'), 'w') as f:
        f.write('# Roteiro de narração — Prática 05.1\n\nTexto de cada cena, com o instante em que ela começa no vídeo '
                f'`{os.path.basename(a.out)}` (duração total {w.frames / FPS:.0f} s).\n\n')
        for name, t0 in marks:
            f.write(f'## [{int(t0 // 60):02d}:{int(t0 % 60):02d}] {name}\n\n{SCENES[name]}\n\n')
    print('video:', a.out, f'{w.frames / FPS:.0f} s')


if __name__ == '__main__':
    main()
