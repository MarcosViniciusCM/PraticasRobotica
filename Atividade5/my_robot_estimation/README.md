# Pratica 05.1 - Estimacao de estados

Pacote `my_robot_estimation`: filtro de Kalman (`robot_localization`) para estimar a
pose do robo e seguimento da trajetoria em 8 usando o estado estimado.

| exercicio | o que pede | onde esta |
|---|---|---|
| 1.1 | EKF do `robot_localization` estimando (x, y, theta) em `/odometry/filtered` | `config/ekf.yaml`, `launch/estimation.launch.py` |
| 2.1 | 8 com o estado estimado no controle; aumentar o ruido; PlotJuggler com real, bruto, estimado e desejado | `launch/figure8_estimated.launch.py`, `config/plotjuggler_estimation.xml`, `scripts/run_experiment.sh` |

## Passo 0 - Instalar e compilar

```bash
sudo apt install ros-humble-robot-localization       # no seu PC, a forma normal
cd ~/ros2_ws
colcon build --packages-select my_robot_control my_robot_estimation
source install/setup.bash
```

> Neste ambiente nao havia `sudo`; o `robot_localization` (e o `ffmpeg`) foram extraidos
> dos `.deb` em `~/ros2_ws/.local_root`. Para usar assim: `source ~/ros2_ws/local_env.sh`
> depois do `setup.bash` do ROS. Com o `apt install` isso nao e necessario.

## Como funciona

```
Gazebo ──/odom (rodas), /imu/out (giroscopio), /ground_truth/odom──▶ noisy_sensors.py
                                                                         │ x noise_scale
     /odom/noisy (vx, w)   /imu/data_noisy (w)   /sensor/pose (x, y, yaw absolutos)
                         └──────────────┬──────────────┘
                                 ekf_filter_node  ──▶  /odometry/filtered
```

| topico | origem | uso |
|---|---|---|
| `/ground_truth/odom` | plugin `libgazebo_ros_p3d` | **so para graficos/metricas**, nunca entra no filtro nem no controle |
| `/odom/noisy` | odometria das rodas + ruido | EKF usa so `vx` e `vyaw` |
| `/imu/data_noisy` | giroscopio + ruido + vies | EKF usa `vyaw` |
| `/sensor/pose` | ground truth + ruido branco, 10 Hz | EKF usa `x, y, yaw` (sensor absoluto: camera, GPS/RTK) |
| `/odometry/filtered` | `ekf_filter_node` | estado estimado, 50 Hz |

Por que o sensor absoluto: a pose da odometria e a integral de velocidades com ruido e
vies, entao deriva sem limite (RMSE > 1 m em 50 s). Sem alguma medida absoluta nenhum
filtro conserta isso - ele so suaviza.

`noise_scale` multiplica o desvio padrao de todos os sensores. Valores com `noise_scale = 1`:
odometria `v` 0.05 m/s (+3 % de vies) e `w` 0.05 rad/s; giroscopio 0.02 rad/s (+0.01 de vies);
posicao 0.05 m; yaw 0.05 rad. As covariancias publicadas acompanham o ruido, e o EKF as usa.

## Exercicio 1.1 - Kalman

```bash
ros2 launch my_robot_estimation estimation.launch.py                   # ruido padrao
ros2 launch my_robot_estimation estimation.launch.py noise_scale:=5.0
ros2 topic hz /odometry/filtered                                       # ~50 Hz
ros2 topic echo /odometry/filtered --once
```

Para ver o filtro com o robo se mexendo, rode o exercicio 2.1.

## Exercicio 2.1 - 8 com o estado estimado

```bash
ros2 launch my_robot_estimation figure8_estimated.launch.py                      # controle com o Kalman
ros2 launch my_robot_estimation figure8_estimated.launch.py noise_scale:=5.0     # ruido 5x
ros2 launch my_robot_estimation figure8_estimated.launch.py state_topic:=/odom/noisy   # controle com o dado bruto
ros2 launch my_robot_estimation figure8_estimated.launch.py plotjuggler:=true
```

E o mesmo `figure8_tracker` da pratica 03; so muda o remapeamento `odom -> /odometry/filtered`.
Cada rodada grava `/tmp/estado.csv` (`log_file:=`), com real, bruto, medido, estimado e desejado.

### PlotJuggler

`config/plotjuggler_estimation.xml` (abra o PlotJuggler **depois** do launch; nao clique em
Start - ver README da pratica 03). Aba *Trajetoria*: plano XY e x(t), y(t) com

| cor | sinal | topico |
|---|---|---|
| verde | estado real | `/ground_truth/odom` |
| vermelho | bruto: odometria | `/odom/noisy` |
| laranja | bruto: posicao | `/sensor/pose` |
| azul | estimado (Kalman) | `/odometry/filtered` |
| preto | trajetoria desejada | `/figure8/desired_pose` |

> Este layout foi escrito a partir do layout da pratica 03 e o XML foi validado, mas **nao foi
> aberto no PlotJuggler** (o ambiente de desenvolvimento nao tem tela). Se alguma curva vier
> vazia, e so arrasta-la da lista da esquerda. Os mesmos graficos, feitos a partir do CSV,
> estao em `results/*/trajetoria_erro.png` e `series.png`.

### Experimentos e resultados

```bash
ros2 run my_robot_estimation run_experiment.sh <rotulo> <noise_scale> [state_topic] [laps] [Omega] [d] [kp]
```

Headless; grava `results/<rotulo>/` (CSV, PNGs, metricas) e uma linha em `results/resumo.csv`.
Todos com `Omega = 0.25`, 2 voltas. RMSE **contra o ground truth**:

| rodada | controle usa | ruido | RMSE sensor pos. | RMSE Kalman | RMSE odom. bruta | **erro real de seguimento** | erro que o controle "enxerga" |
|---|---|---|---|---|---|---|---|
| e1 | Kalman | x1 | 0.076 m | **0.046 m** | 1.18 m | **0.071 m** | 0.075 m |
| e2 | Kalman | x5 | 0.363 m | **0.122 m** | 1.91 m | **0.111 m** | 0.114 m |
| e3 | odom. bruta | x1 | 0.075 m | 0.046 m | 0.32 m | 0.323 m | 0.014 m |
| e4 | odom. bruta | x5 | 0.360 m | 0.126 m | 0.58 m | 0.578 m | 0.043 m |

- O Kalman e mais preciso que qualquer sensor isolado, e a vantagem cresce com o ruido
  (x1: 0.076 -> 0.046; x5: 0.363 -> 0.122).
- Controlando com o Kalman, o robo segue o 8 com 7 cm (x1) e 11 cm (x5) de erro real.
- Controlando com a odometria bruta, o controle acha que esta a 1-4 cm da trajetoria, mas o robo
  real esta a 32-58 cm: o erro de odometria e invisivel para quem so olha para ela.
- O erro de seguimento nao e menor que o da pratica 03 (0.054 m) porque aquele numero era medido
  na odometria, que "mentia" - ver abaixo.

## O que foi preciso ajustar (e por que)

Ao comparar a odometria com o ground truth pela primeira vez, o robo da pratica 02/03 **nao
seguia o 8 de verdade**, mesmo com o estado perfeito no controle:

1. **Derrapagem lateral.** `mu1 = 0.1` nas rodas (pratica 02) faz o chassi derrapar. O
   `my_robot.urdf.xacro` ganhou o argumento `wheel_mu1` (padrao 0.1, entao a pratica 02 nao
   muda); o xacro deste pacote usa 1.0.
2. **Separacao das rodas.** O chassi gira ~17 % menos do que a odometria diz. Calibrado com
   `wheel_separation_multiplier: 1.2` em `config/diff_drive_controller.yaml` (copia do da pratica 03).
3. **`Omega = 0.25 rad/s`.** Com `Omega = 0.5` (1.1 m/s, 2.8 rad/s nas pontas) o robo nao
   segue o 8 (0.56 m de erro) mesmo controlado pelo ground truth - e limite da planta, nao do
   filtro. Com `0.25` o controle com estado perfeito da 0.073 m.
4. **`/clock` a 100 Hz** (`config/gazebo_params.yaml`). O padrao e 10 Hz; com `use_sim_time` o EKF so
   publica a cada tick do `/clock`, e o controle (disparado por `/odometry/filtered`) cairia a 10 Hz.
5. **Sem `:=` nem `: ` em comentarios do URDF.** O `gazebo_ros2_control` repassa o URDF como
   parametro de linha de comando, que e lido como YAML. Por isso os plugins usam `<namespace>` em
   vez de `<remapping>`.

Mudancas na pratica 02 (todas com padrao igual ao anterior): `bringup.launch.py` ganhou
`xacro_file` e `gazebo_params_file`; `my_robot.urdf.xacro` ganhou `wheel_mu1`.

## Video

`video/pratica_05_1.mp4` (+ `video/roteiro_video.md` com o texto de cada cena). **Nao e gravacao de
tela**: as animacoes sao desenhadas a partir dos CSV reais (`results/*/estado.csv`) e o terminal
mostrado e a saida real dos comandos (`results/captura/`). Para regerar:

```bash
python3 scripts/make_video.py
```

Para gravar a tela de verdade (Gazebo + PlotJuggler), rode o launch com `gui:=true plotjuggler:=true`
e use o roteiro como guia.

## Encerrar

`Ctrl+C` no launch; se sobrar processo: `pkill -f gzserver; pkill -f gzclient`.
