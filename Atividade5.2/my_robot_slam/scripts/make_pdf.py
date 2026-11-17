#!/usr/bin/env python3
"""Gera video/legendas.pdf: narracao de cada cena com o tempo em que ela entra no
pratica_05_2_completo.mp4, para leitura (ou para gravar a voz).

Os textos sao os de make_video.py (N). Na cena 1:32-1:45 (SLAM ao vivo no RViz) o texto e o da
versao para o clipe gravado ao vivo.
"""

import os
import sys
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_video import N, PKG  # noqa: E402

LIVE_E21C = ('Aqui o SLAM rodando ao vivo, com tudo ground truth. Os pontos vermelhos são o scan do Lidar; '
             'o mapa vai sendo construído conforme o robô percorre a sala, e as paredes e os três obstáculos aparecem. '
             'Na rodada completa, o erro de posição do SLAM ficou em 1,3 centímetro.')

# (clipe, [(chave, duracao em s, descricao da imagem)])
CLIPS = [
    ('Abertura', [('ab', 18, 'Título e árvore do pacote')]),
    ('Exercício 1.1 — Ruído no Lidar', [('e11a', 23, 'Código do ruído no xacro'),
                                        ('e11b', 15, 'Mesmo mapa, sem e com ruído (ou seu clipe do RViz)')]),
    ('Exercício 2.1 — slam_toolbox', [('e21a', 22, 'Trechos do slam_mapping.yaml'),
                                      ('e21b', 14, 'Árvore de TF e fluxo de dados'),
                                      ('e21c', 13, 'SLAM com tudo ground truth (seu clipe ao vivo do RViz)')]),
    ('Exercício 2.2 — Comparação', [('e22a', 15, 'As três situações e o comando'),
                                    ('e22b', 23, 'Ruído no Lidar e na odometria (animação)'),
                                    ('e22c', 10, 'Os três mapas'),
                                    ('e22d', 13, 'Gráfico de erros')]),
    ('Exercício 3.1 — Robô sequestrado', [('e31a', 16, 'Os três passos'),
                                          ('e31b', 16, 'Sequestro (ou seu clipe 5b)'),
                                          ('e31c', 11, 'Recuperação com /initialpose (ou seu clipe 5c)'),
                                          ('e31d', 22, 'Tabela das quatro tentativas')]),
]


def mmss(s):
    return f'{int(s) // 60}:{int(s) % 60:02d}'


def main():
    pages, fig, y, t0 = [], None, 0, 0

    def new_page(first=False):
        nonlocal fig, y
        fig = plt.figure(figsize=(8.27, 11.69))
        fig.patch.set_facecolor('white')
        y = 0.955
        if first:
            fig.text(0.08, y, 'Prática 05.2 — SLAM', fontsize=22, weight='bold', va='top')
            fig.text(0.08, y - 0.034, 'Narração e legendas do vídeo (3 min 51 s)', fontsize=13, color='#555', va='top')
            fig.text(0.08, y - 0.062, 'Os tempos são do vídeo completo. Onde há clipe gravado ao vivo, o texto vale para ele.',
                     fontsize=9.5, color='#555', va='top')
            y -= 0.10
        pages.append(fig)

    new_page(True)
    for clip, scenes in CLIPS:
        need = 0.05 + sum(0.035 + 0.0175 * len(textwrap.wrap(N[k], 88)) for k, _, _ in scenes[:1])
        if y - need < 0.05:
            new_page()
        fig.text(0.08, y, clip, fontsize=14, weight='bold', color='#1f77b4', va='top')
        y -= 0.034
        for key, dur, what in scenes:
            text = LIVE_E21C if key == 'e21c' else N[key]
            lines = textwrap.wrap(text, 88)
            h = 0.030 + 0.0175 * len(lines)
            if y - h < 0.04:
                new_page()
            fig.text(0.08, y, f'{mmss(t0)} – {mmss(t0 + dur)}', fontsize=10.5, weight='bold', va='top',
                     family='DejaVu Sans Mono')
            fig.text(0.24, y, what, fontsize=9.5, color='#777', va='top', style='italic')
            y -= 0.022
            fig.text(0.08, y, '\n'.join(lines), fontsize=11, va='top', linespacing=1.45)
            y -= 0.0175 * len(lines) + 0.020
            t0 += dur
        y -= 0.012

    out = os.path.join(PKG, 'video', 'legendas.pdf')
    with PdfPages(out) as pdf:
        for f in pages:
            pdf.savefig(f)
            plt.close(f)
    print(out, f'{len(pages)} páginas, fim em {mmss(t0)}')


if __name__ == '__main__':
    main()
