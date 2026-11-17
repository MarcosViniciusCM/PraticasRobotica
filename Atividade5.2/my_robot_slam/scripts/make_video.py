#!/usr/bin/env python3
"""Gera os clipes de video da pratica 05.2 a partir dos resultados reais.

    python3 make_video.py [--clips 0 1 2 3 4] [--out video/clips]

Nao e gravacao de tela: as animacoes sao desenhadas com os CSV gravados pelo slam_logger
(results/*/slam.csv) e com os mapas salvos (results/*/map.pgm); os trechos de codigo sao lidos
dos proprios arquivos do pacote. Cada cena tem uma legenda com o texto da narracao; o mesmo
texto sai em video/narracao.md para ser lido em voz alta (ou colado como legenda na edicao).

Clipes (1 por topico do roteiro de gravacao):
  0 abertura   1 ex1_1 ruido no Lidar   2 ex2_1 slam_toolbox
  3 ex2_2 comparacao   4 ex3_1 robo sequestrado
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
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle, Circle

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.environ.get('PKG_SRC') or os.path.dirname(HERE)
RES = os.path.join(PKG, 'results')

W, H, DPI, FPS = 1280, 720, 100, 25
BG, FG, MUTED = '#fafafa', '#1b1f24', '#5b6570'
C = {'truth': '#2ca02c', 'odom': '#d62728', 'slam': '#1f77b4'}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'axes.facecolor': 'white'})


# ----------------------------------------------------------------------------- dados
def load_csv(label):
    d = np.genfromtxt(os.path.join(RES, label, 'slam.csv'), delimiter=',', names=True)
    return {k: d[k] for k in d.dtype.names}


def read_map(label):
    pgm = os.path.join(RES, label, 'map.pgm')
    with open(pgm, 'rb') as f:
        f.readline()
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    meta = {}
    for line in open(pgm.replace('.pgm', '.yaml')):
        k, _, v = line.partition(':')
        meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    ox, oy = [float(s) for s in meta['origin'].strip('[]').split(',')[:2]]
    return img, [ox, ox + w * res, oy, oy + h * res]


def metrics():
    with open(os.path.join(RES, 'resumo.csv')) as f:
        return {r['rotulo']: {k: float(v) for k, v in r.items() if k != 'rotulo'} for r in csv.DictReader(f)}


def src_lines(rel, a, b):
    with open(os.path.join(PKG, rel)) as f:
        return ''.join(f.readlines()[a - 1:b]).rstrip('\n')


def err(d, s):
    return np.hypot(d[f'{s}_x'] - d['truth_x'], d[f'{s}_y'] - d['truth_y'])


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
    return plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)


def header(fig, tag, title):
    fig.text(0.035, 0.945, tag, fontsize=12, color='white', weight='bold', va='center',
             bbox=dict(boxstyle='round,pad=0.4', fc='#1f77b4', ec='none'))
    fig.text(0.035 + 0.012 + 0.0095 * len(tag), 0.945, title, fontsize=20, weight='bold', color=FG, va='center')


def caption(fig, text):
    fig.patches.append(FancyBboxPatch((0.0, 0.0), 1.0, 0.135, transform=fig.transFigure,
                                      boxstyle='square,pad=0', fc='#14181d', ec='none', zorder=5))
    fig.text(0.03, 0.067, '\n'.join(textwrap.wrap(text, 118)), fontsize=13.5, color='white', va='center',
             zorder=6, linespacing=1.35)


def code_box(fig, rect, text, fs=11.5):
    ax = fig.add_axes(rect)
    ax.set_facecolor('#0f1720')
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.text(0.03, 0.96, text, family='DejaVu Sans Mono', fontsize=fs, color='#e6edf3', va='top', ha='left',
            linespacing=1.45, transform=ax.transAxes)
    return ax


def box(ax, xy, wh, text, fc, ec='#444', fs=11, tc=FG, weight='normal'):
    x, y = xy
    ww, hh = wh
    ax.add_patch(FancyBboxPatch((x, y), ww, hh, boxstyle='round,pad=0.01,rounding_size=0.02', fc=fc, ec=ec, lw=1.4))
    ax.text(x + ww / 2, y + hh / 2, text, ha='center', va='center', fontsize=fs, color=tc, weight=weight,
            linespacing=1.35)


def arrow(ax, a, b, color='#444', lw=1.6, style='-|>'):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=14, color=color, lw=lw))


def draw_room(ax, lw=1.0, color='#888'):
    ax.add_patch(Rectangle((-4, -3), 8, 6, fill=False, ec=color, lw=lw + 0.5))
    ax.add_patch(Rectangle((-2.0, 0.7), 1.0, 0.6, fill=False, ec=color, lw=lw))
    ax.add_patch(Rectangle((1.1, -1.4), 0.8, 0.8, fill=False, ec=color, lw=lw))
    ax.add_patch(Circle((2.0, 1.1), 0.4, fill=False, ec=color, lw=lw))


def show_map(ax, label, alpha=1.0):
    img, ext = read_map(label)
    ax.imshow(img, cmap='gray', extent=ext, origin='upper', vmin=0, vmax=255, alpha=alpha, zorder=0)


def robot_poly(x, y, yaw, L=0.40, Wd=0.26):
    c, s = np.cos(yaw), np.sin(yaw)
    pts = np.array([[L / 2, Wd / 2], [L / 2, -Wd / 2], [-L / 2, -Wd / 2], [-L / 2, Wd / 2]])
    return pts @ np.array([[c, -s], [s, c]]).T + np.array([x, y])


# ----------------------------------------------------------------------------- texto da narracao
N = {
    'ab': 'Prática 05.2, SLAM. Montei o pacote my_robot_slam: o robô da prática anterior com um Lidar 2D, uma sala simples no '
          'Gazebo, e o slam_toolbox. Vou mostrar o ruído no Lidar, a configuração do SLAM, a comparação com e sem ruído, e o '
          'robô sequestrado.',
    'e11a': 'O ruído é do próprio sensor do Gazebo: cada distância medida ganha um valor gaussiano de média zero. O desvio padrão é '
            'o argumento lidar_noise_std; zero é sensor perfeito, 0.03 são 3 centímetros. O Lidar tem 360 feixes, 10 hertz e '
            'alcance de 8 metros, e fica a 15 centímetros de altura, acima do topo das rodas.',
    'e11b': 'Mesma volta, mesmo mapa, só muda o ruído. Sem ruído, a parede vira uma linha fina de células ocupadas. Com 3 '
            'centímetros de desvio, a mesma parede fica espalhada: de 780 para 2030 células ocupadas.',
    'e21a': 'Configurei o slam_toolbox em modo online assíncrono. Ele precisa do tópico do scan, do frame do robô e do TF de odom '
            'para base_link. O resto são os parâmetros de scan matching e de fechamento de laço, com o padrão do pacote; só '
            'reduzi a distância mínima entre scans de 0.5 para 0.15 metro.',
    'e21b': 'A árvore de TF: o slam_toolbox publica map para odom; o meu nó odom_tf_node publica odom para base_link, com a pose '
            'real do Gazebo ou com a odometria ruidosa; o robot_state_publisher publica o Lidar.',
    'e21c': 'Resultado com tudo ground truth: o robô percorre a sala e o SLAM constrói o mapa. A trajetória estimada, em azul, '
            'cobre a real, em verde: erro de posição de 1,3 centímetro.',
    'e22a': 'Três situações, mesmo percurso de duas voltas: tudo ground truth; ruído no Lidar com odometria perfeita; e ruído no '
            'Lidar e na odometria. O script run_slam_experiment salva mapa, CSV e métricas contra o ground truth.',
    'e22b': 'Ruído no Lidar e na odometria. A linha vermelha é só a odometria: ela deriva até 4,8 metros, o caminho nem lembra a '
            'sala. O SLAM, em azul, fica em 6 centímetros, com picos de até 28. Quem remove a deriva é o scan matching contra '
            'o mapa e o fechamento de laço na segunda volta.',
    'e22c': 'Os três mapas lado a lado: com ruído no Lidar as paredes engrossam, mas o mapa continua fechado, mesmo com a '
            'odometria ruidosa.',
    'e22d': 'Resumindo: o ruído no Lidar piora pouco, de 1 para 4 centímetros; a odometria ruidosa sozinha seria desastrosa, '
            '2,2 metros de RMSE, mas com o SLAM o erro fica em 6 centímetros.',
    'e31a': 'Com o mapa já construído, o slam_toolbox em modo localização, com ruído no Lidar e na odometria. Os passos: pausar o '
            'Gazebo, mover o robô em modo de translação para o outro lado do mapa, e retomar a simulação.',
    'e31b': 'O robô de verdade, em verde, foi levado de lá para cá. A estimativa do SLAM, em azul, continua na origem. O erro vai '
            'de 1 centímetro para 3,6 metros e não corrige: um minuto depois são 4,1 metros.',
    'e31c': 'Dando uma pose nova pelo 2D Pose Estimate do RViz, que publica em initialpose, a estimativa pula para o lugar certo: '
            'o erro cai para 4 centímetros.',
    'e31d': 'A localização não se recupera sozinha. O slam_toolbox só compara o scan com o mapa perto da pose que ele já acredita '
            'estar, e a odometria não percebeu o salto. Alargar a busca ou deixar o robô andando não resolveu. É preciso uma pose '
            'inicial nova, ou um localizador global como o AMCL.',
}


# ----------------------------------------------------------------------------- clipe 0
def clip0(w):
    fig = base_fig()
    fig.text(0.5, 0.72, 'Prática 05.2 — SLAM', fontsize=36, weight='bold', ha='center', color=FG)
    fig.text(0.5, 0.63, 'slam_toolbox · Lidar 2D com ruído · robô sequestrado', fontsize=19, ha='center', color=MUTED)
    code_box(fig, [0.22, 0.20, 0.56, 0.35],
             'my_robot_slam/\n'
             '├── urdf/    my_robot_slam.urdf.xacro   Lidar + ruído\n'
             '├── worlds/  room.world                 sala 8 x 6 m\n'
             '├── config/  slam_mapping.yaml\n'
             '│            slam_localization.yaml\n'
             '├── launch/  slam.launch.py\n'
             '├── scripts/ odom_tf_node.py  ...\n'
             '└── results/ s1 s2 s3 (mapas)  k1..k4 (sequestro)', fs=11.5)
    fig.text(0.5, 0.57, 'Robótica · Prof. Davi Santos', fontsize=14, ha='center', color=MUTED)
    caption(fig, N['ab'])
    w.frame(fig, 18 * FPS)
    plt.close(fig)


# ----------------------------------------------------------------------------- clipe 1
def clip1(w, m):
    # cena a: codigo
    fig = base_fig()
    header(fig, 'Exercício 1.1', 'Ruído nas medições do Lidar 2D')
    x = 'urdf/my_robot_slam.urdf.xacro'
    snippet = ('# linha 15\n' + src_lines(x, 15, 15).strip() + '\n\n'
               '# linha 38 - Lidar acima das rodas\n' + src_lines(x, 38, 38).strip() + '\n\n'
               '# linhas 62-66 - ruído gaussiano\n' + '\n'.join(l[2:] for l in src_lines(x, 62, 66).split('\n')))
    code_box(fig, [0.03, 0.17, 0.60, 0.70], snippet, fs=11.5)
    fig.text(0.66, 0.80, 'Sensor "ray" do Gazebo', fontsize=17, weight='bold', color=FG)
    fig.text(0.66, 0.74,
             '• 360 feixes, 10 Hz, 0.12 a 8 m\n\n'
             '• cada distância medida recebe\n  + N(0, σ²), σ = lidar_noise_std\n\n'
             '• σ = 0.0  sensor perfeito\n  σ = 0.03  3 cm\n\n'
             '• no launch:\n  lidar_noise:=0.03\n\n'
             '• Lidar a 0.15 m de altura:\n  mais baixo, o feixe batia nas rodas\n  e o SLAM via pontos fixos no robô',
             fontsize=12, color=FG, va='top', linespacing=1.35)
    caption(fig, N['e11a'])
    w.frame(fig, 23 * FPS)
    plt.close(fig)

    # cena b: mesma parede nos dois mapas
    fig = base_fig()
    header(fig, 'Exercício 1.1', 'O mesmo mapa, sem e com ruído no Lidar')
    for k, (lab, title) in enumerate((('s1_sem_ruido', 'σ = 0 (sem ruído)'), ('s2_ruido_lidar', 'σ = 3 cm'))):
        ax = fig.add_axes([0.05 + 0.47 * k, 0.20, 0.43, 0.62])
        show_map(ax, lab)
        draw_room(ax, color='#2ca02c')
        ax.set_xlim(-4.3, -0.4); ax.set_ylim(0.2, 3.3); ax.set_aspect('equal')
        ax.set_title(f'{title}  —  {int(m[lab]["map_cells"])} células ocupadas no mapa todo', fontsize=11.5)
        if k == 0:
            ax.set_ylabel('y [m]')
    fig.text(0.5, 0.155, 'verde = geometria real da sala · preto = células ocupadas · detalhe do canto noroeste',
             fontsize=10, color=MUTED, ha='center')
    caption(fig, N['e11b'])
    w.frame(fig, 15 * FPS)
    plt.close(fig)


# ----------------------------------------------------------------------------- clipe 2
def clip2(w, m):
    fig = base_fig()
    header(fig, 'Exercício 2.1', 'slam_toolbox — config/slam_mapping.yaml')
    y = 'config/slam_mapping.yaml'
    sn = '\n'.join(l for l in src_lines(y, 11, 56).split('\n')
                   if any(k in l for k in ('odom_frame', 'map_frame', 'base_frame', 'scan_topic', 'mode:',
                                           'max_laser_range', 'minimum_travel', 'map_update', 'do_loop_closing')))
    sn = 'slam_toolbox:\n  ros__parameters:\n' + '\n'.join(l.split('#')[0].rstrip() + ('  #' + l.split('#')[1] if '#' in l else '')
                                                           for l in sn.split('\n'))
    code_box(fig, [0.03, 0.17, 0.66, 0.70], sn, fs=10.4)
    fig.text(0.71, 0.84, 'O que mudou do padrão', fontsize=15, weight='bold', color=FG)
    fig.text(0.71, 0.79,
             '• base_frame: base_link\n  (padrão base_footprint)\n\n'
             '• scan_topic: /scan/out\n\n'
             '• alcance 0.12 a 8 m,\n  igual ao do sensor\n\n'
             '• scans a cada 0.15 m / rad\n  (padrão 0.5): mapa melhor\n\n'
             '• fechamento de laço ligado:\n  corrige a deriva ao revisitar\n  um lugar',
             fontsize=11.5, color=FG, va='top', linespacing=1.35)
    caption(fig, N['e21a'])
    w.frame(fig, 22 * FPS)
    plt.close(fig)

    fig = base_fig()
    header(fig, 'Exercício 2.1', 'Árvore de TF e fluxo de dados')
    ax = fig.add_axes([0.02, 0.18, 0.96, 0.70]); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    box(ax, (0.02, 0.62), (0.22, 0.28), 'Gazebo\nLidar 2D  /scan/out\nrodas  /odom\nground truth', '#e8f5e9')
    box(ax, (0.32, 0.62), (0.26, 0.28), 'odom_tf_node.py\nodom:=truth  pose real\nodom:=noisy  dead reckoning\n(+ ruído, + viés)', '#fff3e0')
    box(ax, (0.66, 0.40), (0.30, 0.34), 'slam_toolbox\n(async, mapping)\n\nscan matching\n+ fechamento de laço', '#e3f2fd', weight='bold')
    box(ax, (0.66, 0.06), (0.30, 0.20), '/map  (OccupancyGrid)\nTF  map → odom', '#bbdefb', weight='bold')
    arrow(ax, (0.24, 0.76), (0.32, 0.76)); arrow(ax, (0.58, 0.72), (0.66, 0.62))
    arrow(ax, (0.24, 0.66), (0.66, 0.52)); arrow(ax, (0.81, 0.40), (0.81, 0.26))
    ax.text(0.02, 0.45, 'TF:  map ──▶ odom ──▶ base_link ──▶ lidar_link', fontsize=14, family='DejaVu Sans Mono', color=FG)
    ax.text(0.02, 0.37, 'slam_toolbox   odom_tf_node   robot_state_publisher', fontsize=11.5, family='DejaVu Sans Mono', color=MUTED)
    ax.text(0.02, 0.22, 'Percurso: waypoint_driver.py anda 2 voltas pelo perímetro da sala,\nsempre igual, para as três situações serem comparáveis.',
            fontsize=12, color=FG, va='top', linespacing=1.5)
    caption(fig, N['e21b'])
    w.frame(fig, 14 * FPS)
    plt.close(fig)

    # cena c: mapa + trajetoria animada (s1)
    d = load_csv('s1_sem_ruido')
    t = d['t']
    fig = base_fig()
    header(fig, 'Exercício 2.1', 'SLAM com tudo ground truth')
    ax = fig.add_axes([0.05, 0.20, 0.50, 0.68])
    show_map(ax, 's1_sem_ruido', alpha=0.9); draw_room(ax, color='#2ca02c')
    ax.set_xlim(-4.4, 4.4); ax.set_ylim(-3.4, 3.4); ax.set_aspect('equal'); ax.set_title('mapa final (fundo) e trajetórias', fontsize=11.5)
    (lt,) = ax.plot([], [], color=C['truth'], lw=2.2, label='real (Gazebo)')
    (ls,) = ax.plot([], [], '--', color=C['slam'], lw=1.4, label='SLAM (map→base_link)')
    poly = Polygon(robot_poly(0, 0, 0), closed=True, fc=C['truth'], ec='k', zorder=5); ax.add_patch(poly)
    ax.legend(fontsize=9, loc='lower right')
    bx = fig.add_axes([0.63, 0.25, 0.34, 0.30]); bx.set_xlim(0, t[-1]); bx.set_ylim(0, 0.08)
    bx.set_xlabel('t [s]'); bx.set_ylabel('erro [m]'); bx.grid(alpha=.3); bx.set_title('erro do SLAM vs ground truth', fontsize=11)
    (le,) = bx.plot([], [], color=C['slam'])
    tx = fig.text(0.63, 0.84, '', fontsize=12.5, family='DejaVu Sans Mono', va='top', color=FG, linespacing=1.6)
    caption(fig, N['e21c'])
    e = err(d, 'slam')
    step = t[-1] / (11 * FPS)
    for k in range(int(t[-1] / step) + 1 + 2 * FPS):
        i = min(np.searchsorted(t, min(k * step, t[-1])), len(t) - 1)
        lt.set_data(d['truth_x'][:i + 1], d['truth_y'][:i + 1]); ls.set_data(d['slam_x'][:i + 1], d['slam_y'][:i + 1])
        poly.set_xy(robot_poly(d['truth_x'][i], d['truth_y'][i], d['truth_yaw'][i]))
        le.set_data(t[:i + 1], e[:i + 1])
        ee = e[:i + 1][~np.isnan(e[:i + 1])]
        tx.set_text(f't = {t[i]:5.1f} s\nerro atual : {e[i] if not np.isnan(e[i]) else 0:.3f} m\n'
                    f'RMSE       : {np.sqrt(np.mean(ee ** 2)) if len(ee) else 0:.3f} m')
        w.frame(fig)
    plt.close(fig)


# ----------------------------------------------------------------------------- clipe 3
def clip3(w, m):
    fig = base_fig()
    header(fig, 'Exercício 2.2', 'Três situações, o mesmo percurso')
    rows = [('s1_sem_ruido', 'tudo ground truth', 'perfeito', 'ground truth'),
            ('s2_ruido_lidar', 'ruído no Lidar', 'σ = 3 cm', 'ground truth'),
            ('s3_ruido_odom_lidar', 'ruído no Lidar e na odometria', 'σ = 3 cm', 'ruidosa (dead reckoning)')]
    txt = 'rótulo                 Lidar      odometria\n' + '-' * 54 + '\n'
    for lab, _, l, o in rows:
        txt += f'{lab:<22} {l:<10} {o}\n'
    txt += ('\n$ ros2 run my_robot_slam run_slam_experiment.sh \\\n'
            '      s3_ruido_odom_lidar 0.03 noisy\n\n'
            '  -> sobe Gazebo + SLAM, anda 2 voltas (~175 s)\n'
            '  -> salva mapa (map.pgm), grafo de poses, slam.csv\n'
            '  -> compara com o ground truth: metricas.txt, resultado.png')
    code_box(fig, [0.05, 0.19, 0.90, 0.68], txt, fs=13)
    caption(fig, N['e22a'])
    w.frame(fig, 15 * FPS)
    plt.close(fig)

    # animacao s3
    d = load_csv('s3_ruido_odom_lidar')
    t = d['t']
    ok = ~np.isnan(d['slam_x']) & ~np.isnan(d['odom_x'])
    eo, es = err(d, 'odom'), err(d, 'slam')
    fig = base_fig()
    header(fig, 'Exercício 2.2', 'Ruído no Lidar e na odometria')
    ax = fig.add_axes([0.05, 0.20, 0.42, 0.68])
    show_map(ax, 's3_ruido_odom_lidar', alpha=0.85); draw_room(ax, color='#2ca02c')
    ax.set_xlim(-6, 6); ax.set_ylim(-4.6, 4.6); ax.set_aspect('equal'); ax.set_title('mapa final (fundo) e trajetórias', fontsize=11.5)
    (lt,) = ax.plot([], [], color=C['truth'], lw=2.2, label='real (Gazebo)')
    (lo,) = ax.plot([], [], ':', color=C['odom'], lw=1.5, label='odometria sozinha')
    (ls,) = ax.plot([], [], '--', color=C['slam'], lw=1.4, label='SLAM')
    poly = Polygon(robot_poly(0, 0, 0), closed=True, fc=C['truth'], ec='k', zorder=5); ax.add_patch(poly)
    ax.legend(fontsize=8.5, loc='lower right')
    bx = fig.add_axes([0.57, 0.22, 0.40, 0.33]); bx.set_xlim(0, t[-1]); bx.set_ylim(0, 5.2)
    bx.set_xlabel('t [s]'); bx.set_ylabel('erro de posição [m]'); bx.grid(alpha=.3); bx.set_title('erro contra o ground truth', fontsize=11)
    (beo,) = bx.plot([], [], color=C['odom'], label='odometria sozinha'); (bes,) = bx.plot([], [], color=C['slam'], label='SLAM')
    bx.legend(fontsize=9, loc='upper left')
    tx = fig.text(0.57, 0.86, '', fontsize=12.5, family='DejaVu Sans Mono', va='top', color=FG, linespacing=1.6)
    caption(fig, N['e22b'])
    step = t[-1] / (20 * FPS)
    for k in range(int(t[-1] / step) + 1 + 3 * FPS):
        i = min(np.searchsorted(t, min(k * step, t[-1])), len(t) - 1)
        lt.set_data(d['truth_x'][:i + 1], d['truth_y'][:i + 1])
        lo.set_data(d['odom_x'][:i + 1], d['odom_y'][:i + 1]); ls.set_data(d['slam_x'][:i + 1], d['slam_y'][:i + 1])
        poly.set_xy(robot_poly(d['truth_x'][i], d['truth_y'][i], d['truth_yaw'][i]))
        beo.set_data(t[:i + 1][ok[:i + 1]], eo[:i + 1][ok[:i + 1]]); bes.set_data(t[:i + 1][ok[:i + 1]], es[:i + 1][ok[:i + 1]])
        sel = ok[:i + 1]
        r = lambda a: np.sqrt(np.mean(a[:i + 1][sel] ** 2)) if sel.any() else 0.0
        tx.set_text(f't = {t[i]:5.1f} s\nodometria : {eo[i] if ok[i] else 0:5.2f} m   (RMSE {r(eo):.2f})\n'
                    f'SLAM      : {es[i] if ok[i] else 0:5.2f} m   (RMSE {r(es):.3f})')
        w.frame(fig)
    plt.close(fig)

    # tres mapas
    fig = base_fig()
    header(fig, 'Exercício 2.2', 'Os três mapas')
    for k, (lab, title) in enumerate((('s1_sem_ruido', 'tudo ground truth'), ('s2_ruido_lidar', 'ruído no Lidar'),
                                      ('s3_ruido_odom_lidar', 'ruído no Lidar + odometria'))):
        ax = fig.add_axes([0.03 + 0.325 * k, 0.30, 0.31, 0.55])
        show_map(ax, lab); draw_room(ax, color='#2ca02c')
        ax.set_xlim(-4.4, 4.4); ax.set_ylim(-3.4, 3.4); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(title, fontsize=12)
        ax.set_xlabel(f'erro de pose {100 * m[lab]["slam_rmse"]:.1f} cm · mapa {100 * m[lab]["map_err_mean"]:.1f} cm', fontsize=10.5)
    fig.text(0.5, 0.20, 'verde = geometria real · preto = mapa do SLAM', ha='center', fontsize=10.5, color=MUTED)
    caption(fig, N['e22c'])
    w.frame(fig, 10 * FPS)
    plt.close(fig)

    # tabela / barras
    fig = base_fig()
    header(fig, 'Exercício 2.2', 'Erro de posição contra o ground truth')
    ax = fig.add_axes([0.07, 0.26, 0.55, 0.58])
    labs = ['s1\nsem ruído', 's2\nLidar', 's3\nLidar + odom.']
    ks = ['s1_sem_ruido', 's2_ruido_lidar', 's3_ruido_odom_lidar']
    x = np.arange(3); wd = 0.36
    vo = [max(m[k]['odom_rmse'], 1e-3) for k in ks]; vs = [m[k]['slam_rmse'] for k in ks]
    ax.bar(x - wd / 2, vo, wd, color=C['odom'], label='odometria sozinha'); ax.bar(x + wd / 2, vs, wd, color=C['slam'], label='SLAM')
    for xi, v in zip(x - wd / 2, [m[k]['odom_rmse'] for k in ks]):
        ax.text(xi, max(v, 1e-3) * 1.15, f'{v:.3f} m', ha='center', fontsize=9.5)
    for xi, v in zip(x + wd / 2, vs):
        ax.text(xi, v * 1.15, f'{v:.3f} m', ha='center', fontsize=9.5, weight='bold')
    ax.set_yscale('log'); ax.set_ylim(5e-4, 10); ax.set_xticks(x); ax.set_xticklabels(labs); ax.set_ylabel('RMSE [m] (escala log)')
    ax.grid(axis='y', alpha=.3); ax.legend(fontsize=10, loc='upper left')
    code_box(fig, [0.66, 0.26, 0.31, 0.58],
             'RMSE    odom.    SLAM\n' + '-' * 27 + '\n' +
             ''.join(f's{i + 1}   {m[k]["odom_rmse"]:7.3f}  {m[k]["slam_rmse"]:7.3f}\n' for i, k in enumerate(ks)) +
             '\nmáx. s3  4.807    0.277\nyaw  s3  --       2.84°\n\nmapa (erro médio)\n' +
             ''.join(f's{i + 1}  {100 * m[k]["map_err_mean"]:4.1f} cm\n' for i, k in enumerate(ks)), fs=11.5)
    caption(fig, N['e22d'])
    w.frame(fig, 13 * FPS)
    plt.close(fig)


# ----------------------------------------------------------------------------- clipe 4
def kidnap_anim(w, label, cap, dur, mark_init=None, tag='Exercício 3.1', title='Robô sequestrado'):
    d = load_csv(label)
    t = d['t']
    e = err(d, 'slam')
    jump = np.hypot(np.diff(d['truth_x']), np.diff(d['truth_y']))
    kidx = int(np.argmax(jump)) + 1
    fig = base_fig()
    header(fig, tag, title)
    ax = fig.add_axes([0.05, 0.20, 0.45, 0.68])
    draw_room(ax, color='#888'); ax.set_xlim(-4.5, 4.5); ax.set_ylim(-3.5, 3.5); ax.set_aspect('equal'); ax.grid(alpha=.25)
    ax.set_title('posição real × estimada (RViz)', fontsize=11.5)
    (lt,) = ax.plot([], [], '.', color=C['truth'], ms=3); (ls,) = ax.plot([], [], '.', color=C['slam'], ms=3)
    (mt,) = ax.plot([], [], 'o', color=C['truth'], ms=13, mec='k', label='robô real (Gazebo)')
    (ms,) = ax.plot([], [], 's', color=C['slam'], ms=11, mec='k', label='estimativa do SLAM')
    ax.legend(fontsize=9, loc='lower left')
    bx = fig.add_axes([0.58, 0.25, 0.39, 0.33]); bx.set_xlim(0, t[-1]); bx.set_ylim(0, 4.6)
    bx.set_xlabel('t [s de simulação]'); bx.set_ylabel('erro de posição [m]'); bx.grid(alpha=.3)
    bx.axhline(0.30, color='gray', ls=':'); (le,) = bx.plot([], [], color=C['slam'])
    bx.axvline(t[kidx], color='r', ls='--', lw=1)
    if mark_init is not None:
        bx.axvline(t[mark_init], color='purple', ls='--', lw=1)
    tx = fig.text(0.58, 0.86, '', fontsize=12.5, family='DejaVu Sans Mono', va='top', color=FG, linespacing=1.6)
    caption(fig, cap)
    step = (t[-1] - t[0]) / ((dur - 3) * FPS)
    for k in range(int((t[-1] - t[0]) / step) + 1 + 3 * FPS):
        i = min(np.searchsorted(t, min(t[0] + k * step, t[-1])), len(t) - 1)
        lt.set_data(d['truth_x'][:i + 1], d['truth_y'][:i + 1]); ls.set_data(d['slam_x'][:i + 1], d['slam_y'][:i + 1])
        mt.set_data([d['truth_x'][i]], [d['truth_y'][i]]); ms.set_data([d['slam_x'][i]], [d['slam_y'][i]])
        le.set_data(t[:i + 1], e[:i + 1])
        status = 'localizado' if i < kidx else ('SEQUESTRADO' if e[i] > 0.3 else 'recuperou')
        tx.set_text(f't = {t[i]:5.1f} s   {status}\nerro de posição: {e[i]:.2f} m')
        w.frame(fig)
    plt.close(fig)


def clip4(w, m):
    fig = base_fig()
    header(fig, 'Exercício 3.1', 'Robô sequestrado com o mapa já construído')
    ax = fig.add_axes([0.02, 0.50, 0.96, 0.38]); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    box(ax, (0.00, 0.15), (0.22, 0.7), '1. Pausar o Gazebo\n\nbotão Pause\ngz world -p 1', '#fff8e1')
    box(ax, (0.26, 0.15), (0.24, 0.7), '2. Mover o robô\n\ntecla T (translação) e\narrastar para o outro lado\ngz model -m my_robot -x 3 -y -2', '#ffebee', fs=10.5)
    box(ax, (0.54, 0.15), (0.20, 0.7), '3. Retomar\n\nbotão Play\ngz world -p 0', '#e8f5e9')
    box(ax, (0.78, 0.15), (0.22, 0.7), '4. Observar\n\nA estimativa do SLAM\nse recupera?', '#e3f2fd', weight='bold')
    for a, b in ((0.22, 0.26), (0.50, 0.54), (0.74, 0.78)):
        arrow(ax, (a, 0.5), (b, 0.5))
    code_box(fig, [0.03, 0.17, 0.94, 0.28],
             '$ ros2 launch my_robot_slam slam.launch.py localization:=true \\\n'
             '      map_file:=.../results/s3_ruido_odom_lidar/posegraph \\\n'
             '      lidar_noise:=0.03 odom:=noisy drive:=false\n'
             '# slam_localization.yaml:  mode: localization', fs=11.5)
    caption(fig, N['e31a'])
    w.frame(fig, 16 * FPS)
    plt.close(fig)

    kidnap_anim(w, 'k1_sequestro', N['e31b'], 16)

    d2 = load_csv('k2_sequestro_initialpose')
    e2 = err(d2, 'slam')
    kid = int(np.argmax(np.hypot(np.diff(d2['truth_x']), np.diff(d2['truth_y'])))) + 1
    init = kid + int(np.argmax(e2[kid:] < 0.3)) - 0
    kidnap_anim(w, 'k2_sequestro_initialpose', N['e31c'], 11, mark_init=max(init - 12, kid + 1),
                title='Com /initialpose (2D Pose Estimate)')

    fig = base_fig()
    header(fig, 'Exercício 3.1', 'Quatro tentativas')
    txt = 'rodada                      o que muda                          erro 60 s depois   recuperou?\n' + '-' * 100 + '\n'
    cases = [('k1_sequestro', 'robô parado depois do sequestro'),
             ('k2_sequestro_initialpose', 'pose real em /initialpose'),
             ('k3_sequestro_busca_larga', 'janela +-2 m e busca de laço 12 m'),
             ('k4_sequestro_em_movimento', 'robô continua o percurso')]
    for lab, what in cases:
        dd = load_csv(lab)
        ee = err(dd, 'slam')
        sim = 'SIM' if ee[-1] < 0.3 else 'não'
        txt += f'{lab:<27} {what:<35} {ee[-1]:>6.2f} m          {sim}\n'
    txt += ('\nPor quê: em modo localização o slam_toolbox só casa o scan com os nós do mapa\n'
            'PERTO da pose que ele já estima. A odometria (rodas) não viu o salto, então a\n'
            'estimativa fica na origem e nunca se aproxima da pose real.\n\n'
            'Saída: uma pose inicial nova (2D Pose Estimate -> /initialpose) ou um\n'
            'localizador global, como o AMCL do Nav2 (partículas por todo o mapa).')
    code_box(fig, [0.03, 0.17, 0.94, 0.70], txt, fs=11.4)
    caption(fig, N['e31d'])
    w.frame(fig, 22 * FPS)
    plt.close(fig)


# ----------------------------------------------------------------------------- main
CLIPS = {0: ('00_abertura', lambda w, m: clip0(w), ['ab']),
         1: ('01_ex1_1_ruido_lidar', clip1, ['e11a', 'e11b']),
         2: ('02_ex2_1_slam_toolbox', clip2, ['e21a', 'e21b', 'e21c']),
         3: ('03_ex2_2_comparacao', clip3, ['e22a', 'e22b', 'e22c', 'e22d']),
         4: ('04_ex3_1_robo_sequestrado', clip4, ['e31a', 'e31b', 'e31c', 'e31d'])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', type=int, nargs='*', default=sorted(CLIPS))
    ap.add_argument('--out', default=os.path.join(PKG, 'video', 'clips'))
    ap.add_argument('--narracao', action='store_true', help='so escreve video/narracao.md')
    a = ap.parse_args()
    if not a.narracao:
        m = metrics()
        for c in a.clips:
            name, fn, _ = CLIPS[c]
            w = Writer(os.path.join(a.out, name + '.mp4'))
            fn(w, m)
            w.close()
            print(f'{name}.mp4  {w.frames / FPS:.0f} s', flush=True)
    else:
        with open(os.path.join(os.path.dirname(a.out), 'narracao.md'), 'w') as f:
            f.write('# Narração dos clipes — Prática 05.2\n\nTexto de cada cena (mesmo da legenda do vídeo), para ler em voz alta.\n\n')
            for c in sorted(CLIPS):
                name, _, keys = CLIPS[c]
                f.write(f'## {name}\n\n')
                for k in keys:
                    f.write(f'{N[k]}\n\n')


if __name__ == '__main__':
    main()
