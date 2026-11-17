# Pratica 05.2 - SLAM

Pacote `my_robot_slam`: SLAM 2D com o `slam_toolbox` no Gazebo, com ruido no Lidar e na odometria, e o
teste do robo sequestrado. Reaproveita o robo e os sensores da pratica 05.1 (`my_robot_estimation`).

| exercicio | o que pede | onde esta |
|---|---|---|
| 1.1 | ruido nas medicoes do Lidar 2D | `urdf/my_robot_slam.urdf.xacro` (argumento `lidar_noise_std`) |
| 2.1 | configurar o `slam_toolbox` em um cenario simples | `config/slam_mapping.yaml`, `worlds/room.world`, `launch/slam.launch.py` |
| 2.2 | comparar: sem ruido / ruido no Lidar / ruido no Lidar e na odometria | `scripts/run_slam_experiment.sh`, `results/` |
| 3.1 | robo sequestrado com o mapa pronto | `config/slam_localization.yaml`, `scripts/run_kidnap.sh` |

## Passo 0 - Instalar e compilar

```bash
sudo apt install ros-humble-slam-toolbox                 # no seu PC, a forma normal
cd ~/ros2_ws
colcon build --packages-select my_robot_slam
source install/setup.bash
```

> Neste ambiente nao havia `sudo`: o `slam_toolbox` e as bibliotecas que ele pede (ceres, glog,
> suitesparse) foram extraidos dos `.deb` em `~/ros2_ws/.local_root`. Uso: `source ~/ros2_ws/local_env.sh`
> depois do `setup.bash` do ROS. Com o `apt install` isso nao e necessario.

## Como funciona

```
Gazebo ─ /scan/out (Lidar 2D, 10 Hz, 360 feixes, 0.12-8 m, ruido gaussiano)  ──────────┐
       ─ /odom (rodas) ─▶ odom_tf_node ─ TF odom->base_link (verdade OU ruidosa) ──────▶ slam_toolbox ─▶ /map
       ─ /ground_truth/odom ─ so para metricas, e para a odometria "perfeita"             └▶ TF map->odom
```

* **Lidar com ruido (1.1)**: o sensor `ray` do Gazebo soma `N(0, sigma^2)` a cada distancia. `sigma` e o
  argumento xacro `lidar_noise_std` (`0.0` = perfeito); no launch: `lidar_noise:=0.03`.
* **Odometria (`odom_tf_node.py`)**: `odom:=truth` publica o TF `odom->base_link` com a pose real do Gazebo
  (sem erro). `odom:=noisy` integra as velocidades das rodas com ruido branco, +3 % de escala e vies de
  giroscopio - dead reckoning puro, sem fonte absoluta, e deriva sem limite. `odom_noise:=` multiplica o ruido.
* **TF**: `map -> odom` (slam_toolbox) -> `base_link` (odom_tf_node) -> `lidar_link` (robot_state_publisher).
  Quem corrige a deriva da odometria e o SLAM, ao casar cada scan com o mapa (scan matching) e fechar lacos.
* **Percurso**: `waypoint_driver.py` anda sempre a mesma volta pelo perimetro da sala (2 voltas), para as tres
  situacoes serem comparaveis. Usa o ground truth so para dirigir, nunca entra no SLAM.

## Exercicios 1.1 e 2.1 - rodar

```bash
ros2 launch my_robot_slam slam.launch.py rviz:=true gui:=false                              # tudo ground truth
ros2 launch my_robot_slam slam.launch.py rviz:=true gui:=false lidar_noise:=0.03            # ruido no Lidar
ros2 launch my_robot_slam slam.launch.py rviz:=true gui:=false lidar_noise:=0.03 odom:=noisy   # + odometria
ros2 launch my_robot_slam slam.launch.py rviz:=true drive:=false teleop:=true               # dirigir no teclado
ros2 topic hz /scan/out          # ~10 Hz
ros2 run tf2_ros tf2_echo map base_link
```

Salvar o mapa (imagem + grafo de poses para a localizacao):

```bash
ros2 run my_robot_slam save_map.py ~/mapa
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/$USER/mapa_grafo'}"
```

### Parametros do `slam_toolbox` (`config/slam_mapping.yaml`)

So o que mudou do padrao (o resto esta no arquivo com os valores do pacote):

| parametro | valor | por que |
|---|---|---|
| `base_frame` | `base_link` | o padrao e `base_footprint`; este robo so tem `base_link` |
| `scan_topic` | `/scan/out` | topico do plugin do Lidar |
| `max_laser_range` / `min_laser_range` | 8.0 / 0.12 | iguais ao alcance do sensor |
| `minimum_travel_distance` / `_heading` | 0.15 m / 0.15 rad | padrao 0.5: mais scans entram no grafo e o mapa fica melhor |
| `minimum_time_interval` | 0.2 s | padrao 0.5 s |
| `map_update_interval` | 1.0 s | padrao 5 s: o mapa acompanha o robo no RViz |
| `do_loop_closing` | true | corrige a deriva quando o robo revisita um lugar |

> **Armadilha do Lidar**: o primeiro Lidar estava a 0.08 m de altura, abaixo do topo das rodas (0.10 m). O
> feixe batia nas rodas, o SLAM via pontos fixos no robo e achava que ele nunca saia do lugar (erro de 1 m e
> mapa torto mesmo com odometria perfeita). Com o sensor a 0.15 m o erro caiu para centimetros.

## Exercicio 2.2 - comparacao

```bash
ros2 run my_robot_slam run_slam_experiment.sh <rotulo> <lidar_noise_std> <truth|noisy> [odom_noise_scale] [voltas]
```

Headless; grava `results/<rotulo>/` (mapa, grafo de poses, CSV, `resultado.png`, `metricas.txt`) e uma linha em
`results/resumo.csv`. Metricas contra o ground truth: erro de posicao do SLAM (TF `map->base_link`) e da
odometria sozinha (TF `odom->base_link`), e erro do mapa (distancia de cada celula ocupada a superficie real
da sala; cobertura = fracao da superficie real que aparece no mapa).

Todas com o mesmo percurso (2 voltas, ~175 s de simulacao), Lidar `sigma = 3 cm`, RMSE **contra o ground truth**:

| rodada | Lidar | odometria | erro de posicao: odometria sozinha | **erro de posicao: SLAM** | yaw SLAM | erro medio do mapa |
|---|---|---|---|---|---|---|
| `s1_sem_ruido` | perfeito | ground truth | 0.002 m | **0.013 m** (max 0.058) | 0.29 graus | 3.0 cm |
| `s2_ruido_lidar` | 3 cm | ground truth | 0.000 m | **0.040 m** (max 0.102) | 0.75 graus | 4.5 cm |
| `s3_ruido_odom_lidar` | 3 cm | ruidosa (`noise_scale` 1) | **2.178 m** (max 4.807) | **0.061 m** (max 0.277) | 2.84 graus | 4.2 cm |

(`results/resumo.csv`; figuras em `results/<rotulo>/resultado.png`.)

- **Sem ruido**: o mapa coincide com a geometria real (erro de celula ~3 cm, que e o tamanho da celula de 5 cm).
  O erro de 1.3 cm do SLAM e o proprio scan matching.
- **Ruido no Lidar**: o erro de pose triplica (1.3 -> 4.0 cm) e as paredes ficam mais grossas no mapa (mais
  celulas ocupadas: 780 -> 2030), porque cada feixe cai um pouco antes ou depois da parede. O SLAM continua
  fechando o mapa sem deriva, porque a odometria e perfeita.
- **Ruido no Lidar e na odometria**: sozinha, a odometria deriva ate **4.8 m** (RMSE 2.2 m) e o percurso
  integrado nem lembra a sala. Com o SLAM a pose estimada fica em 6 cm (com picos de ate 28 cm, que aparecem
  no grafico de erro ao longo do percurso) e o mapa tem a mesma qualidade da rodada anterior (4.2 cm). Quem remove a deriva e
  o scan matching contra o mapa, mais o fechamento de laco na 2a volta.
- A "cobertura" do mapa (coluna `map_cover_10cm` do CSV) so mede a fracao da superficie real que aparece no
  mapa; em `s1` deu 88 % (nas outras, 100 %), provavelmente porque a parede leste ficou cortada na borda da grade
  do mapa (na figura ela aparece so em verde) e nao por erro de localizacao - nao investiguei mais a fundo. Por
  isso a cobertura nao esta na tabela.

## Exercicio 3.1 - robo sequestrado

```bash
ros2 run my_robot_slam run_kidnap.sh <rotulo> <results/mapa_pronto> <lidar_noise_std> [odom_noise_scale] [sim|nao]
```

Com o mapa de uma rodada anterior (`posegraph.*`), sobe o `slam_toolbox` em modo **localizacao**, com Lidar e
odometria ruidosos, e repete o roteiro da pratica pela linha de comando:

1. pausar o Gazebo - `gz world -p 1` (botao de pausa);
2. mover o robo - `gz model -m my_robot -x 3 -y -2 -Y 3.14` (modo de translacao);
3. retomar - `gz world -p 0`.

Na interface do Gazebo e o mesmo: botao de pausa, tecla `T` (translacao), arrastar o robo para o outro lado,
botao de play. O ultimo argumento (`sim`) publica a pose real em `/initialpose` depois do sequestro (o
"2D Pose Estimate" do RViz).

Mapa usado: o da rodada `s3` (Lidar e odometria ruidosos). Sequestro de (0, 0) para (3.0, -2.0), virado para o
outro lado (theta = pi), cerca de 15 s depois de ligar a localizacao. Erro de posicao do SLAM contra o ground truth:

| rodada | o que muda | erro logo apos | erro 60 s depois | recuperou? |
|---|---|---|---|---|
| `k1_sequestro` | robo parado depois do sequestro | 3.64 m | 4.12 m | **nao** |
| `k2_sequestro_initialpose` | publica a pose real em `/initialpose` 20 s depois | 3.64 m | **0.04 m** | **sim**, 1.5 s depois do `/initialpose` |
| `k3_sequestro_busca_larga` | janela de casamento +-2 m e busca de laco de 12 m (`slam_localization_busca_larga.yaml`) | 3.65 m | 5.31 m | nao (piorou) |
| `k4_sequestro_em_movimento` | o robo continua o percurso durante o sequestro | 2.99 m | 2.95 m | nao |

**A localizacao do robo nao se recupera sozinha.** O `slam_toolbox` em modo localizacao so casa o scan novo com os
nos do mapa **perto da pose que ele ja estima**. Como a odometria nao viu o salto (as rodas nao giraram), a estimativa
continua na origem e o scan, que agora enxerga outra parte da sala, e casado (ou descartado) la, sem nunca se
aproximar da pose real. Alargar a busca (`k3`) nao resolve e ainda deixa o casamento aceitar encaixes errados;
andar (`k4`) tambem nao. Recuperar exige uma pose inicial nova - o "2D Pose Estimate" do RViz publica em
`/initialpose` (`k2`) - ou um localizador global, como o AMCL do Nav2 (filtro de particulas espalhadas por todo o
mapa), que o `slam_toolbox` nao tem.

## Encerrar

`Ctrl+C` no launch; se sobrar processo: `~/ros2_ws/killsim.sh`.

## Video

`video/clips/` tem 5 clipes (um por topico do roteiro), `video/pratica_05_2_completo.mp4` e a narracao em
`video/narracao.md` e `video/legendas.pdf` (narracao com os tempos de cada cena). Nao e gravacao de tela: as animacoes sao desenhadas com os CSV e mapas reais de `results/`
e os trechos de codigo sao lidos dos proprios arquivos. Para regerar: `python3 scripts/make_video.py`.
