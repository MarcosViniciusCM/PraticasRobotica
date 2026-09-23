# Pratica 02 - Controle de movimento I

Pacote `my_robot_control`: modelo do robo diferencial em URDF, controle de
velocidade das rodas com `gazebo_ros2_control` + `diff_drive_controller`, e
teleoperacao pelo teclado.

| exercicio | o que pede | onde esta |
|---|---|---|
| 1.1 | modelo URDF + launch do Gazebo Classic | `urdf/my_robot.urdf.xacro`, `launch/urdf_gazebo.launch.py` |
| 1.2 | `diff_drive_controller` em `/cmd_vel` + ajuste dos parametros | `config/diff_drive_controller.yaml`, blocos `<gazebo>` do URDF |
| 2.1 | teleop pelo teclado + launch unico do sistema | `launch/bringup.launch.py` |

---

# Passo a passo

## Passo 0 - Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_control
source install/setup.bash
```

O `source` vale **por terminal**. Cada terminal novo precisa repetir
`source ~/ros2_ws/install/setup.bash`.

## Passo 1 - Exercicio 1.1: o modelo URDF no Gazebo

```bash
ros2 launch my_robot_control urdf_gazebo.launch.py
```

Sobe o Gazebo Classic e carrega **so** o modelo, sem a pilha de controle.

O que conferir na tela:

- o **chassi azul** aparece junto com as duas rodas e a roda boba
  (se aparecerem so as tres pecas pretas, veja *Mesh do chassi invisivel* no
  fim deste arquivo);
- o robo pousa no chao e fica parado, sem tremer nem afundar;
- em outro terminal, `ros2 topic echo /robot_description --once` mostra o URDF.

Encerre com `Ctrl+C` no terminal.

## Passo 2 - Exercicio 1.2: controladores

```bash
ros2 launch my_robot_control bringup.launch.py
```

Este e o launch unico que o exercicio 2.1 pede: ele sobe **tudo** - Gazebo, o
robo, o `controller_manager`, os controladores, a ponte de topicos e o teleop.

Espere ~30 s (no WSL pode passar disso) e confira, em **outro terminal**:

```bash
source ~/ros2_ws/install/setup.bash

# os dois controladores tem que aparecer como "active"
ros2 control list_controllers

# a odometria tem que estar publicando por volta de 50 Hz
ros2 topic hz /odom
```

Saida esperada do `list_controllers`:

```
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
diff_drive_controller   diff_drive_controller/DiffDriveController      active
```

Para comandar sem usar o teclado:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Passo 3 - Exercicio 1.2: PlotJuggler

Com o sistema do Passo 2 rodando, abra em outro terminal:

```bash
sudo apt install ros-humble-plotjuggler-ros    # so na primeira vez

source ~/ros2_ws/install/setup.bash
ros2 run plotjuggler plotjuggler \
  -l $(ros2 pkg prefix my_robot_control)/share/my_robot_control/config/plotjuggler_velocidades.xml
```

O `-l` carrega o layout que ja vem no pacote, com os quatro graficos montados -
nao precisa arrastar nenhuma serie.

**Agora e preciso ligar o streaming na interface.** As flags de linha de comando
(`--start_streamer`, `--buffer_size`) **nao funcionam** nesta versao: o
PlotJuggler restaura as preferencias salvas e ignora os argumentos. Entao:

1. No painel **Streaming** (canto superior esquerdo), troque o dropdown para
   **`ROS2 Topic Subscriber`**. Ele costuma vir em `Foxglove ROS2 Bridge`, que e
   outro plugin e nao le topicos ROS 2 diretamente.
2. Aumente o **Buffer** de `5` para `60` segundos.
3. Clique **Start**.
4. Na janela que abrir, marque **`/cmd_vel`** e **`/odom`** e confirme em **OK**.

Como saber que funcionou, sem depender do grafico:

- o botao **Start** vira **Stop**;
- e no terminal:

```bash
ros2 topic info /odom      # "Subscription count" tem que ser >= 1
```

Se o botao continuar escrito *Start*, o streaming **nao** subiu e o grafico vai
ficar congelado por mais que voce dirija o robo.

O layout poe, em cada grafico, o comando junto com a resposta medida:

| grafico | comando (azul) | resposta medida (vermelho) |
|---|---|---|
| Velocidade linear (m/s) | `/cmd_vel/linear/x` | `/odom/twist/twist/linear/x` |
| Velocidade angular (rad/s) | `/cmd_vel/angular/z` | `/odom/twist/twist/angular/z` |

> `/cmd_vel` so tem mensagens enquanto alguem esta comandando. Com o robo
> parado a curva azul fica sem pontos novos - isso e normal, nao e falha.

### Sequencia sugerida para o grafico

Dirigir pelo teclado gera um grafico baguncado. Para uma figura limpa, rode
estes degraus em outro terminal e grave a tela do PlotJuggler:

```bash
source ~/ros2_ws/install/setup.bash
P="ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist"

timeout 8 $P "{linear: {x: 0.5}, angular: {z: 0.0}}"    # degrau linear
timeout 4 $P "{linear: {x: 0.0}, angular: {z: 0.0}}"    # parado
timeout 8 $P "{linear: {x: 1.0}, angular: {z: 0.0}}"    # limite de velocidade
timeout 4 $P "{linear: {x: 0.0}, angular: {z: 0.0}}"
timeout 8 $P "{linear: {x: 0.0}, angular: {z: 1.0}}"    # degrau angular
timeout 4 $P "{linear: {x: 0.0}, angular: {z: 0.0}}"
timeout 8 $P "{linear: {x: 0.3}, angular: {z: 0.8}}"    # arco
timeout 4 $P "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

O que comentar no video ao mostrar esse grafico:

- a resposta (vermelha) **segue** o comando (azul) com uma **rampa**, nao um
  salto - a inclinacao e a `max_acceleration` do YAML;
- em regime permanente as duas curvas se encostam, ou seja, **erro praticamente
  nulo**;
- no degrau de `1.0 m/s` da para mostrar a **saturacao**: pedindo mais do que
  `linear.x.max_velocity` o controlador limita.

## Passo 4 - Exercicio 2.1: teleop pelo teclado

O `bringup.launch.py` ja abriu uma janela **xterm** com o titulo
*"teleop (use este terminal)"*.

**Clique dentro dessa janela** (o teclado precisa estar com o foco nela) e use:

```
   u    i    o          u = arco frente/esquerda
   j    k    l          i = frente        o = arco frente/direita
   m    ,    .          j = gira esquerda k = PARA   l = gira direita
                        , = re
```

| tecla | efeito |
|---|---|
| `q` / `z` | aumenta / diminui **ambas** as velocidades em 10% |
| `w` / `x` | so a **linear** em 10% |
| `e` / `c` | so a **angular** em 10% |
| `Ctrl+C` | encerra o teleop |

Comeca em **0,5 m/s** e **1,0 rad/s**.

Dois detalhes:

- `k` nao e especial: **qualquer** tecla nao mapeada para o robo;
- o robo **continua andando** ate voce mandar parar - nao e preciso segurar a
  tecla;
- as **maiusculas** (`U I O J K L`) sao modo holonomico (andar de lado) e
  `t` / `b` sobem/descem em Z. O robo e diferencial, entao o
  `diff_drive_controller` ignora `linear.y` e `linear.z` - so `linear.x` e
  `angular.z` fazem efeito.

Aviso inofensivo que aparece no terminal:
`xterm: cannot load font ...` - e so a fonte padrao do xterm faltando no WSL.

## Passo 5 - Encerrar

`Ctrl+C` no terminal onde voce rodou o `ros2 launch` derruba tudo.

Confira depois, porque o `gzclient` as vezes fica orfao e deixa uma janela
fantasma do Gazebo aberta:

```bash
pgrep -af "gzserver|gzclient"     # tem que nao retornar nada
pkill -f gzclient; pkill -f gzserver   # se precisar forcar
```

Se o Gazebo passar a falhar com `Address already in use`, ou se algum no travar
na descoberta, limpe o lixo do DDS deixado por processos mortos a forca:

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

---

# Referencia

## Estrutura do pacote

```
my_robot_control/
├── urdf/my_robot.urdf.xacro                 # modelo URDF + ros2_control (ex. 1.1 e 1.2)
├── config/diff_drive_controller.yaml        # controller_manager + diff_drive_controller (ex. 1.2)
├── config/plotjuggler_velocidades.xml       # layout do PlotJuggler
├── launch/urdf_gazebo.launch.py             # Gazebo Classic + modelo URDF (ex. 1.1)
├── launch/bringup.launch.py                 # sistema completo (ex. 2.1)
├── scripts/diff_drive_bridge.py             # expoe /cmd_vel e /odom
└── meshes/chassis_box.stl
```

Argumentos dos launches:

| argumento | launch | padrao | efeito |
|---|---|---|---|
| `use_ros2_control` | `urdf_gazebo` | `false` | carrega o modelo com a interface `ros2_control` |
| `gui` | `bringup` | `true` | `false` nao abre o gzclient (simulacao headless) |
| `teleop` | `bringup` | `true` | `false` nao abre o xterm do teclado |

## Por que existe o `diff_drive_bridge`

O `diff_drive_controller` usa topicos relativos ao proprio no
(`/diff_drive_controller/cmd_vel_unstamped` e `/diff_drive_controller/odom`).
Nesta versao do `ros2_control` (2.54) as regras de remapping do
`controller_manager` - inclusive as da tag `<ros><remapping>` do plugin
`gazebo_ros2_control` - **nao sao repassadas aos controladores**, entao nao da
para renomear esses topicos so por configuracao.

O no `diff_drive_bridge` faz essa ponte e expoe `/cmd_vel` e `/odom`, que sao os
nomes pedidos no roteiro e os padroes do ecossistema ROS. Com isso, qualquer
fonte de `Twist` (teleop, joystick, rqt, nav2) funciona sem configuracao extra.

## Parametros que afetam o comportamento

| sintoma | parametro | arquivo |
|---|---|---|
| gira menos do que o comandado | `fdir1` + `mu1` / `mu2` das rodas | `urdf/my_robot.urdf.xacro` |
| roda patina, robo nao anda | `mu2` das rodas | `urdf/my_robot.urdf.xacro` |
| robo treme ou afunda no chao | `kp`, `kd`, `minDepth` | `urdf/my_robot.urdf.xacro` |
| robo tomba ao acelerar | massas e `<inertial>` | `urdf/my_robot.urdf.xacro` |
| anda mais/menos do que o comandado | `wheel_radius` | `config/diff_drive_controller.yaml` |
| gira mais/menos do que o comandado | `wheel_separation` | `config/diff_drive_controller.yaml` |
| partida brusca | `max_acceleration` | `config/diff_drive_controller.yaml` |
| odometria com ruido | `velocity_rolling_window_size` | `config/diff_drive_controller.yaml` |

`wheel_radius` e `wheel_separation` do YAML **precisam bater** com o URDF
(`wheel_radius = 0.1`, `wheel_separation = 2 * wheel_offset_y = 0.26`), senao a
conversao (v, w) -> velocidade das rodas e a odometria ficam com erro de escala.

## Atrito das rodas - o detalhe que faz o giro funcionar

O ponto que mais afetou a qualidade do movimento foi a **direcao** do atrito das
rodas, nao o valor dele.

No Gazebo Classic, `mu1` e o coeficiente de atrito ao longo de `fdir1` e `mu2` o
coeficiente na direcao perpendicular. Sem declarar `fdir1`, o ODE escolhe uma
primeira direcao arbitraria, entao `mu1`/`mu2` nao correspondem a "longitudinal"
e "lateral" e a roda acaba com atrito alto tambem de lado.

Ao girar em torno do proprio eixo, as rodas de um robo diferencial precisam
arrastar lateralmente. Com atrito lateral alto esse arrasto vence o motor da
junta, as rodas giram mais devagar do que o comandado e a velocidade angular
medida fica bem abaixo da de referencia.

Apontando `fdir1` no eixo da roda (`0 1 0`), `mu1` vira o atrito lateral (baixo,
`0.1`) e `mu2` o de rolamento (alto, `1.5`, mantem a tracao). Medido com o
comando publicado em `/cmd_vel` e a resposta lida em `/odom`:

| ensaio | comando | antes (`mu1=1.5`, sem `fdir1`) | depois (`fdir1`, `mu1=0.1`) |
|---|---|---|---|
| girar no proprio eixo | `angular.z = 1.0 rad/s` | 0.847 rad/s (erro 15%) | **0.994 rad/s** (erro 0.6%) |
| andar para frente | `linear.x = 0.5 m/s` | 0.496 m/s | 0.496 m/s |
| arco | `0.3 m/s`, `0.5 rad/s` | 0.295 / 0.334 | **0.295 / 0.506** |

A velocidade das rodas em `/joint_states` confirma o diagnostico: no giro elas
marcavam +-1.10 rad/s no lugar dos +-1.30 rad/s comandados, e passaram a seguir
a referencia depois da correcao.

## Armadilha - dois-pontos nos comentarios do URDF

O plugin `gazebo_ros2_control` repassa o URDF inteiro ao `controller_manager`
como um *parameter override* (`--param robot_description:=<urdf>`), e o `rcl`
interpreta esse valor como YAML. Um `: ` (dois-pontos seguido de espaco) em
qualquer lugar do arquivo - **inclusive dentro de um comentario XML** - faz o
YAML ser lido como um mapeamento e o parse falha:

```
[ERROR] [gazebo_ros2_control]: parser error Couldn't parse parameter override
rule: '--param robot_description:=<?xml version="1.0" ?> ...
```

O sintoma e o `controller_manager` nunca subir, e os spawners ficarem parados em
`waiting for service /controller_manager/list_controllers`.

Por isso os comentarios deste URDF usam ` - ` no lugar de `: `. Ao editar o
modelo, vale conferir:

```bash
xacro urdf/my_robot.urdf.xacro | grep ": "   # nao deve retornar nada
```

## Armadilha - mesh do chassi invisivel (so as rodas aparecem)

Sintoma: o robo aparece no Gazebo como tres formas pretas soltas (as duas rodas
e a roda boba) e o chassi nao e desenhado. A fisica funciona normalmente, porque
a `<collision>` do chassi e uma `<box>` primitiva - o que falta e so o
`<visual>`, que usa um mesh STL.

Sao **dois** problemas somados:

1. **O Gazebo Classic nao entende `package://`.** Ele resolve `model://`. O
   `spawn_entity.py` converte um no outro, mas so com a flag
   `-package_to_model`, que precisa estar nos `arguments` do no.

2. **O `gzclient` nao herda o `GAZEBO_MODEL_PATH`.** Quem desenha o mesh e o
   `gzclient`, que e um processo separado do `gzserver`. O `gzserver.launch.py`
   monta os caminhos internamente e os entrega **so ao gzserver**; o `gzclient`
   herda o ambiente do shell, onde `GAZEBO_MODEL_PATH` costuma estar vazio
   (o `setup.bash` do ROS 2 nao carrega o `/usr/share/gazebo/setup.sh`).
   Resultado: o servidor conhece o mesh, mas o cliente nao consegue abri-lo.

Por isso os launches usam `AppendEnvironmentVariable` **antes** de subir os
processos, valendo para os dois:

```python
AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(pkg_share))
AppendEnvironmentVariable('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')
AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')
```

As duas ultimas linhas tambem fazem sumir o aviso
`Unable to find shader lib. Your GAZEBO_RESOURCE_PATH is probably improperly set.`

### Como conferir sem depender do olho

**Ausencia de erro no log do `gzserver` nao prova nada aqui** - o servidor nem
tenta abrir o mesh, porque quem renderiza e o cliente. Duas checagens objetivas:

```bash
# 1) o modelo no servidor tem o visual com a URI certa?
gz model -m my_robot -i | grep -A3 "type: MESH"
#    deve mostrar filename: "model://my_robot_control/meshes/chassis_box.stl"

# 2) o gzclient enxerga o caminho?
tr '\0' '\n' < /proc/$(pgrep -f '^gzclient')/environ | grep GAZEBO_MODEL_PATH
#    deve conter .../install/my_robot_control/share
```
