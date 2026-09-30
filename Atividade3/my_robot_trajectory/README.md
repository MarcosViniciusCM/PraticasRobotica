# Pratica 03 - Controle de trajetoria

Pacote `my_robot_trajectory`: controle de pose (continuo e em tres manobras),
seguimento de missao por waypoints e seguimento de trajetoria em 8, sobre o robo
diferencial da pratica 02 (`my_robot_control`).

| exercicio | o que pede | onde esta |
|---|---|---|
| 1.1 | no de controle de pose continuo, PID de biblioteca, ganhos em YAML | `src/pose_controller.cpp`, `config/pose_controller.yaml` |
| 1.1 bonus | controle de pose com 3 manobras | `src/three_step_controller.cpp`, `config/three_step_controller.yaml` |
| 1.2 | seguimento de >= 3 waypoints carregados de YAML | `src/mission_follower.cpp`, `config/mission.yaml` |
| 1.2 bonus | comparar continuo x 3 manobras na missao | `scripts/compare_mission.sh`, `results/mission_compare.csv` |
| 2.1 | trajetoria em 8, parametros em YAML, malha aberta / realimentado, ganho `Kff` | `src/figure8_tracker.cpp`, `config/figure8.yaml` |
| 2.1 bonus | maior `Omega` que o robo ainda segue | `scripts/figure8_experiments.sh omega`, `results/figure8_omega.csv` |
| 2.2 | PlotJuggler com trajetoria desejada x real | `config/plotjuggler_figure8.xml` |
| 2.2 bonus | RMSE malha aberta x `Kff=0` x `Kff=1` | `scripts/figure8_experiments.sh rmse`, `results/figure8_rmse.csv` |

**Biblioteca de PID:** `control_toolbox::Pid` (pacote `ros-humble-control-toolbox`,
o mesmo usado pelos controladores do `ros2_control`). Cada PID le do YAML
`kp`, `ki`, `kd`, `i_max`/`i_min` (limites do integrador) e `antiwindup`.

---

# Passo a passo

## Passo 0 - Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_control my_robot_trajectory
source install/setup.bash
```

Todos os launches abaixo sobem a simulacao da pratica 02 sozinhos (Gazebo +
robo + `diff_drive_controller`), **sem** o teleop. Leva ~30 s ate o robo se
mexer. Argumentos comuns:

| argumento | padrao | efeito |
|---|---|---|
| `gui` | `true` | `false` roda o Gazebo sem janela (mais leve) |
| `sim` | `true` | `false` nao sobe o Gazebo - use se ele ja estiver aberto |

Para deixar o Gazebo aberto e so reiniciar os nos entre testes:

```bash
ros2 launch my_robot_trajectory sim.launch.py                       # terminal 1
ros2 launch my_robot_trajectory mission.launch.py sim:=false        # terminal 2, repetir a vontade
```

> Com `sim:=false`, a odometria continua de onde o robo parou. O 8 comeca na
> posicao atual do robo, mas os waypoints da missao sao absolutos no `odom`.

## Passo 1 - Exercicio 1.1: controle de pose

```bash
ros2 launch my_robot_trajectory pose_control.launch.py                        # continuo
ros2 launch my_robot_trajectory pose_control.launch.py controller:=three_step # 3 manobras
```

Quando aparecer `Aguardando alvo em /goal_pose`, envie uma pose em outro
terminal (x = 2, y = 1, yaw = 90 graus):

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0}, orientation: {z: 0.7071, w: 0.7071}}}"
```

O quaternion de um yaw `a` e `{z: sin(a/2), w: cos(a/2)}`. Tambem funciona o
botao **2D Goal Pose** do RViz (que publica no mesmo `/goal_pose`, com fixed
frame `odom`).

No terminal do launch aparece `Alvo atingido (erro pos=..., yaw=...)` e em
`/goal_reached` e publicado `true`.

## Passo 2 - Exercicio 1.2: missao

```bash
ros2 launch my_robot_trajectory mission.launch.py                        # continuo
ros2 launch my_robot_trajectory mission.launch.py controller:=three_step # 3 manobras
```

A missao padrao (`config/mission.yaml`) e um quadrado de 2 m percorrido no
sentido anti-horario, com a orientacao final de cada canto apontando para o
proximo:

```yaml
waypoints: ["p1", "p2", "p3", "p4"]
p1: [2.0, 0.0, 90.0]     # x [m], y [m], yaw [graus]
p2: [2.0, 2.0, 180.0]
p3: [0.0, 2.0, -90.0]
p4: [0.0, 0.0, 0.0]
```

Para outra missao, copie o arquivo e passe `mission_file:=/caminho/missao.yaml`.
Ao final o `mission_follower` imprime uma tabela com tempo e erro final de
cada waypoint, tempo total e distancia percorrida.

Visualizacao (opcional):

```bash
ros2 run plotjuggler plotjuggler \
  -l $(ros2 pkg prefix my_robot_trajectory)/share/my_robot_trajectory/config/plotjuggler_mission.xml
```

### Bonus - continuo x 3 manobras

```bash
ros2 run my_robot_trajectory compare_mission.sh
```

Roda a mesma missao com os dois controladores, headless, reiniciando o Gazebo
entre as rodadas, e imprime a tabela comparativa (CSV salvo em `~/`).

## Passo 3 - Exercicio 2.1: trajetoria em 8

```bash
ros2 launch my_robot_trajectory figure8.launch.py                   # realimentado + feedforward (Kff=1)
ros2 launch my_robot_trajectory figure8.launch.py mode:=open_loop   # malha aberta
ros2 launch my_robot_trajectory figure8.launch.py kff:=0.0          # realimentado sem feedforward
ros2 launch my_robot_trajectory figure8.launch.py Omega:=0.6        # outra frequencia
ros2 launch my_robot_trajectory figure8.launch.py plotjuggler:=true # ja abre o PlotJuggler
```

`mode`, `kff`, `Omega` e `laps` passados na linha de comando sobrescrevem o
YAML; sem eles vale `config/figure8.yaml`
(`A = 1`, `B = 2`, `Omega = 0.5`, 2 voltas).

Sequencia do no: espera a odometria -> 2 s -> gira no lugar ate a direcao
inicial do 8 -> percorre `laps` voltas -> para e imprime o RMSE.

## Passo 4 - Exercicio 2.2: PlotJuggler

Com o 8 rodando (ou use `plotjuggler:=true` no passo anterior):

```bash
ros2 run plotjuggler plotjuggler \
  -l $(ros2 pkg prefix my_robot_trajectory)/share/my_robot_trajectory/config/plotjuggler_figure8.xml
```

Como na pratica 02, **o streaming precisa ser ligado na interface**:

1. painel **Streaming** -> dropdown **`ROS2 Topic Subscriber`**;
2. **Buffer** em `60` s (duas voltas com `Omega = 0.5` levam ~25 s);
3. **Start** -> marque `/odom`, `/figure8/desired_pose`, `/figure8/error` e
   `/figure8/rmse` -> **OK**.

| aba | grafico | azul | vermelho |
|---|---|---|---|
| Trajetoria | plano XY | 8 desejado (`/figure8/desired_pose`) | trajetoria real (`/odom`) |
| Trajetoria | x(t), y(t) | desejado | real |
| Erro | erro de posicao | `ex` (azul), `ey` (verde) | |
| Erro | erro de orientacao | `e_theta` [rad] | |
| Erro | RMSE acumulado | | posicao [m] (vermelho), orientacao [rad] (laranja) |

O 8 completo tambem sai em `/figure8/desired_path` (`nav_msgs/Path`), para o RViz.

### Bonus - RMSE e maior Omega

```bash
ros2 run my_robot_trajectory figure8_experiments.sh rmse                  # malha aberta, Kff=0, Kff=1
ros2 run my_robot_trajectory figure8_experiments.sh omega 0.5 0.6 0.7 0.8 # varredura de Omega
```

Cada rodada reinicia o Gazebo (headless) e grava uma linha em CSV
(`OUT=arquivo.csv` para escolher onde). Resultados na secao seguinte.

---

# Resultados

Medidos em Gazebo Classic com este pacote (CSV completos em `results/`).

## 2.2 bonus - RMSE do 8 (A = 1, B = 2, Omega = 0.5, 2 voltas)

| configuracao | RMSE posicao | RMSE orientacao | erro max. |
|---|---|---|---|
| malha aberta | 0.318 m | 0.042 rad (2.4 deg) | 0.366 m |
| realimentado, `Kff = 0` | 0.381 m | 0.607 rad (34.8 deg) | 0.523 m |
| realimentado, `Kff = 1` | **0.054 m** | **0.027 rad (1.6 deg)** | 0.329 m |

- **Malha aberta:** o robo parte do repouso e a referencia ja comeca a ~1.1 m/s.
  Pela rampa de aceleracao, ele fica ~0.3 m atras e nunca recupera, porque nada
  mede o erro. A forma do 8 sai certa (erro de orientacao baixo), mas atrasada
  e deslocada. Em malha aberta, qualquer derrapagem ou erro inicial se acumula.
- **Realimentado com `Kff = 0`:** so o termo P. Para gerar velocidade, o P
  precisa de erro: com `v ~ 1 m/s` e `kp = 2`, o robo anda ~0.4-0.5 m atras da
  referencia o tempo todo e "corta" as curvas (erro de orientacao grande).
  Esse e o erro de regime de um controle so proporcional seguindo uma
  referencia em rampa.
- **Realimentado com `Kff = 1`:** o feedforward fornece a velocidade nominal e
  o P so corrige o desvio. O erro maximo (0.33 m) acontece na **partida**; em
  regime o erro fica em ~1 cm, e o RMSE e dominado por esse transitorio
  inicial.

## 2.1 bonus - maior Omega

Realimentado com `Kff = 1`, criterio "segue" = RMSE de posicao < 0.10 m:

| Omega [rad/s] | v de pico da referencia | RMSE posicao | RMSE orientacao |
|---|---|---|---|
| 0.5 | 1.12 m/s | 0.054 m | 0.027 rad |
| **0.6** | 1.34 m/s | **0.085 m** | 0.076 rad |
| 0.7 | 1.57 m/s | 0.182 m | 0.203 rad |
| 0.8 | 1.79 m/s | 0.360 m | 0.272 rad |
| 1.0 | 2.24 m/s | 0.935 m | 0.748 rad |

**Maior Omega que o robo ainda segue: ~0.6 rad/s.** A partir de ~0.67 rad/s a
velocidade pedida passa do limite do robo (1.5 m/s em
`config/diff_drive_controller.yaml`). A velocidade angular tambem satura
(4 rad/s) nas pontas do 8, onde o raio de curvatura e de ~12 cm. O comando
satura, o robo fica para tras e corta as curvas.

## 1.2 bonus - missao: continuo x 3 manobras

Quadrado de 2 m, 4 waypoints, mesmas tolerancias (5 cm / 0.03 rad) e mesmos
limites de velocidade (0.6 m/s, 1.5 rad/s):

| controlador | tempo total | distancia percorrida | erro pos. medio | erro yaw medio |
|---|---|---|---|---|
| continuo | 35.7 s | 8.32 m | 0.055 m | 1.65 deg |
| 3 manobras | **29.1 s** | **7.86 m** | 0.052 m | 1.57 deg |

- **3 manobras:** caminho quase igual ao minimo (8 m de lados), mas o robo
  para em cada canto para girar. Movimento previsivel e facil de ajustar: cada
  manobra tem um unico PID e um unico objetivo.
- **Continuo:** trajetoria suave em curva, sem parar para girar, mas percorre
  mais (as curvas de aproximacao impostas pelo termo `beta` para chegar com a
  orientacao certa). Como `v = k_rho * rho` decai exponencialmente, os
  ultimos centimetros sao lentos, o que explica o tempo maior. Aumentar
  `pid_rho.kp` reduz o tempo, ao custo de curvas mais abertas e mais saturacao.
- O erro final fica perto da tolerancia de 5 cm nos dois: o controlador para
  assim que entra nela. Para mais precisao, reduza `position_tolerance`.

---

# Referencia

## Estrutura do pacote

```
my_robot_trajectory/
├── include/my_robot_trajectory/common.hpp  # angulos, PID a partir de parametros, saturacao
├── src/pose_controller.cpp                 # ex. 1.1 - controle de pose continuo
├── src/three_step_controller.cpp           # ex. 1.1 bonus - rotacao, translacao, rotacao
├── src/mission_follower.cpp                # ex. 1.2 - missao por waypoints
├── src/figure8_tracker.cpp                 # ex. 2.1 - trajetoria em 8
├── config/pose_controller.yaml             # ganhos do controle continuo
├── config/three_step_controller.yaml       # ganhos das 3 manobras
├── config/mission.yaml                     # waypoints
├── config/figure8.yaml                     # parametros do 8 e do controlador
├── config/diff_drive_controller.yaml       # copia do da pratica 02 com limites maiores
├── config/plotjuggler_figure8.xml          # layout do ex. 2.2
├── config/plotjuggler_mission.xml          # layout da missao
├── launch/sim.launch.py                    # simulacao (bringup da pratica 02 sem teleop)
├── launch/pose_control.launch.py           # ex. 1.1
├── launch/mission.launch.py                # ex. 1.2
├── launch/figure8.launch.py                # ex. 2.1 / 2.2
├── scripts/figure8_experiments.sh          # bonus 2.1 e 2.2
├── scripts/compare_mission.sh              # bonus 1.2
└── results/*.csv                           # resultados medidos
```

## Topicos

| topico | tipo | quem publica | quem le |
|---|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | `mission_follower`, RViz, voce | controladores de pose |
| `/goal_reached` | `std_msgs/Bool` (latched) | controladores de pose | `mission_follower` |
| `/cmd_vel` | `geometry_msgs/Twist` | controladores, `figure8_tracker` | `diff_drive_bridge` |
| `/odom` | `nav_msgs/Odometry` | `diff_drive_bridge` | todos |
| `/figure8/desired_pose` | `geometry_msgs/PoseStamped` | `figure8_tracker` | PlotJuggler |
| `/figure8/desired_path` | `nav_msgs/Path` (latched) | `figure8_tracker` | RViz |
| `/figure8/error` | `geometry_msgs/Vector3Stamped` | `figure8_tracker` | x, y = erro de posicao; z = erro de orientacao |
| `/figure8/rmse` | `geometry_msgs/Vector3Stamped` | `figure8_tracker` | x = RMSE pos.; y = RMSE orient.; z = t |

## Controle de pose continuo (1.1)

Lei polar do Siegwart (*Introduction to Autonomous Mobile Robots*, cap. 3):

```
rho   = sqrt(dx^2 + dy^2)                  distancia ao alvo
alpha = atan2(dy, dx) - theta              alvo em relacao a frente do robo
beta  = theta_goal - atan2(dy, dx)         orientacao final em relacao a reta ate o alvo

v = PID_rho(rho) * max(cos(alpha), 0)
w = PID_alpha(alpha) + PID_beta(beta)
```

- No caso P puro, a malha e estavel com `k_rho > 0`, `k_beta < 0` e
  `k_alpha > k_rho`. Os valores do YAML (`0.6`, `-0.4`, `2.0`) respeitam isso.
- O fator `max(cos(alpha), 0)` nao esta no livro: ele zera o avanco enquanto o
  alvo esta atras do robo, que entao gira primeiro em vez de se afastar numa
  curva larga.
- Dentro de `position_tolerance`, o robo so corrige a orientacao final com
  `PID_yaw`, que tem integral para zerar o erro residual. Para evitar
  chaveamento, ele so volta a transladar se sair de 3x a tolerancia.
- A saturacao reduz `v` e `w` na mesma proporcao, preservando a curvatura.

## Controle de pose em 3 manobras (1.1 bonus)

| manobra | comando | termina quando |
|---|---|---|
| 1. rotacao | `w = PID_angular(atan2(dy,dx) - theta)`, `v = 0` | `\|erro\| < yaw_tolerance` |
| 2. translacao | `v = PID_linear(d)`, `w = PID_heading(phi - theta)` | `\|d\| < position_tolerance` |
| 3. rotacao | `w = PID_angular(theta_goal - theta)`, `v = 0` | `\|erro\| < yaw_tolerance` |

`phi` e a direcao da reta ate o alvo, congelada no inicio da translacao. `d` e
a distancia ate o alvo projetada nessa reta, com sinal: se o robo passar do
ponto, `d < 0` e ele da re. Usar `atan2(dy, dx)` direto na translacao nao
funciona, porque perto do alvo esse angulo muda bruscamente e faz o robo girar.

## Trajetoria em 8 (2.1)

Lemniscata de Gerono, com origem na posicao inicial do robo:

```
x_d(t) = A sin(W t)
y_d(t) = B sin(W t) cos(W t) = (B/2) sin(2 W t)       periodo T = 2 pi / W
```

Com `A = 1` e `B = 2`, o 8 ocupa um quadrado de 2 m x 2 m (x e y em [-1, 1]).

> **Confira com os slides.** Esta foi a parametrizacao adotada. Se a da aula
> for outra (por exemplo `y = B sin(2 W t)`), basta mudar a funcao
> `reference()` em `src/figure8_tracker.cpp`: ela devolve posicao, velocidade
> e aceleracao, e o resto do no usa so esse retorno.

Referencias derivadas analiticamente:

```
theta_d = atan2(dy_d, dx_d)
v_d     = sqrt(dx_d^2 + dy_d^2)
w_d     = (dx_d ddy_d - dy_d ddx_d) / (dx_d^2 + dy_d^2)
```

**Malha aberta** (`mode: open_loop`): `v = v_d`, `w = w_d`.

**Realimentado** (`mode: feedback`): linearizacao por realimentacao de um
ponto `P` a `d` metros a frente do eixo das rodas. A cinematica desse ponto e
inversivel, entao ele pode seguir qualquer velocidade `u`:

```
u = Kff * dP_d/dt + [PID_x(Px_d - Px), PID_y(Py_d - Py)]

v =  cos(theta) u_x + sin(theta) u_y
w = (-sin(theta) u_x + cos(theta) u_y) / d
```

- `P_d = (x_d + d cos theta_d, y_d + d sin theta_d)` e onde o ponto `P` estaria
  se o robo estivesse exatamente sobre o 8. Assim, erro zero em `P` implica o
  **centro** do robo sobre a trajetoria, e o RMSE e medido no centro do robo.
- `Kff` pondera o feedforward: `0` = so realimentacao, `1` = feedforward
  completo.
- `d` pequeno demais faz o termo `1/d` saturar `w` (com `d = 0.1` o RMSE era
  0.24 m, com `d = 0.3` caiu para 0.054 m). `d` grande demais deixa o robo
  "cortar" as curvas fechadas das pontas.
- `align_start: true` gira o robo no lugar ate `theta_d(0)` (~63 graus) antes
  de comecar. Assim a malha aberta parte da mesma condicao inicial que o
  realimentado e a comparacao de RMSE e justa.

## Detalhe - laco de controle disparado pela odometria

Os nos de controle **nao** usam timer: cada mensagem de `/odom` (50 Hz,
`publish_rate` do `diff_drive_controller`) dispara um passo de controle, e o
carimbo da odometria e o relogio do PID. Um timer em tempo simulado so dispara
quando o `/clock` avanca, e o Gazebo publica o `/clock` numa taxa baixa. Na
primeira versao o controle rodava a ~20 Hz, com amostras velhas, o que ajudava
a piorar o seguimento do 8.

## Mudanca na pratica 02

O `bringup.launch.py` do `my_robot_control` ganhou o argumento
`controllers_file`. O padrao continua sendo o YAML da propria pratica 02, entao
o comportamento dela nao muda. Este pacote passa o seu
`config/diff_drive_controller.yaml`, igual ao original exceto pelos limites:

| limite | pratica 02 | pratica 03 |
|---|---|---|
| `linear.x.max_velocity` | 1.0 m/s | 1.5 m/s |
| `linear.x.max_acceleration` | 1.0 m/s^2 | 2.0 m/s^2 |
| `angular.z.max_velocity` | 2.0 rad/s | 4.0 rad/s |
| `angular.z.max_acceleration` | 2.0 rad/s^2 | 6.0 rad/s^2 |

O 8 com `Omega = 0.5` pede ate 1.12 m/s e 2.8 rad/s, acima dos limites
originais.

## Encerrar

`Ctrl+C` no terminal do launch. Se sobrar uma janela do Gazebo:

```bash
pkill -f gzclient; pkill -f gzserver
```

O `process has died [gzserver ... exit code -6]` que aparece ao encerrar e um
crash conhecido do Gazebo Classic ao receber SIGINT, inofensivo.
