# Roteiro de gravação — Prática 05.2 (SLAM) — vídeo de até 5 min

Tempo total planejado: **4 min 45 s**. Os tempos abaixo são do vídeo final, não da gravação: as simulações
duram mais do que isso, então grave cada clipe separado e **corte/acelere (2x a 4x)** na edição.

## Antes de gravar (uma vez)

Abra 3 coisas lado a lado: VS Code (código), terminal integrado do VS Code, e deixe o RViz e o Gazebo para abrir pelos launches.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/local_env.sh        # só se o slam_toolbox foi extraído em .local_root (ver README); com apt install, pule
cd ~/ros2_ws && colcon build --packages-select my_robot_slam
source install/setup.bash
~/ros2_ws/killsim.sh                 # garante que não sobrou nada de simulação anterior
```

Abra no VS Code, em abas, nesta ordem (você vai alternar entre elas):
1. `urdf/my_robot_slam.urdf.xacro`
2. `config/slam_mapping.yaml`
3. `scripts/odom_tf_node.py`
4. `config/slam_localization.yaml`

Deixe prontas para abrir (no explorador de arquivos ou no VS Code): `results/s1_sem_ruido/resultado.png`,
`results/s2_ruido_lidar/resultado.png`, `results/s3_ruido_odom_lidar/resultado.png`, `results/resumo.csv`,
`results/k1_sequestro/resultado.png`, `results/k2_sequestro_initialpose/resultado.png`.

> Eu não consegui ver as janelas do Gazebo e do RViz (o ambiente onde preparei isto não tem tela). O RViz
> abriu com o `config/slam.rviz` sem erro, mas **faça um ensaio rápido de cada comando antes de gravar** e, se
> alguma camada do RViz vier vazia, ligue pelo "Add" (Map em `/map`, LaserScan em `/scan/out`).

---

## Sequência do vídeo

| # | Tempo no vídeo | Tópico | O que aparece |
|---|---|---|---|
| 1 | 0:00 – 0:20 | Abertura | árvore do pacote |
| 2 | 0:20 – 1:00 | Ex. 1.1 Ruído no Lidar | xacro + RViz com e sem ruído |
| 3 | 1:00 – 2:00 | Ex. 2.1 slam_toolbox | yaml + launch + mapa sendo construído |
| 4 | 2:00 – 3:30 | Ex. 2.2 Comparação | 3 situações + tabela de métricas |
| 5 | 3:30 – 4:45 | Ex. 3.1 Robô sequestrado | Gazebo pausado, robô movido, resultado |

---

### Clipe 1 — Abertura (0:00 – 0:20)

**Gravar:** o VS Code com o explorador mostrando `src/Atividade5/my_robot_slam/` (pastas `urdf`, `worlds`, `config`, `launch`, `scripts`, `results`).

**Falar (~15 s):**
> "Prática 05.2, SLAM. Montei o pacote my_robot_slam: o robô da prática anterior com um Lidar 2D, uma sala simples no Gazebo, e o slam_toolbox. Vou mostrar o ruído no Lidar, a configuração do SLAM, a comparação com e sem ruído, e o robô sequestrado."

---

### Clipe 2 — Exercício 1.1: ruído no Lidar (0:20 – 1:00)

**2a. Código (0:20 – 0:35).** Aba `urdf/my_robot_slam.urdf.xacro`. Mostre:
- linha 15: `<xacro:arg name="lidar_noise_std" default="0.0" />`
- linhas 62–66: bloco `<noise>` com `<type>gaussian</type>` e `<stddev>$(arg lidar_noise_std)</stddev>`
- linha 38: `origin xyz="0 0 0.15"` (o Lidar fica acima das rodas)

**Falar:**
> "O ruído é do próprio sensor do Gazebo: cada distância medida ganha um valor gaussiano de média zero. O desvio padrão é o argumento lidar_noise_std — zero é sensor perfeito, 0.03 são 3 centímetros. O Lidar tem 360 feixes, 10 hertz, alcance até 8 metros. Ele fica a 15 centímetros de altura, acima do topo das rodas; mais baixo, o feixe batia nas rodas e o SLAM achava que o robô não saía do lugar."

**2b. Rodando (0:35 – 1:00).** Terminal. Grave duas vezes o mesmo comando, mudando só o ruído (cada uma ~10 s, depois de o RViz abrir e o robô começar a andar). O robô anda sozinho pelo percurso.

```bash
ros2 launch my_robot_slam slam.launch.py gui:=false rviz:=true lidar_noise:=0.0
# Ctrl+C, ~/ros2_ws/killsim.sh, e agora:
ros2 launch my_robot_slam slam.launch.py gui:=false rviz:=true lidar_noise:=0.03
```

No RViz foque nos **pontos vermelhos do Lidar** (LaserScan): sem ruído as paredes são linhas retas; com 3 cm elas ficam serrilhadas/"grossas". Se quiser mostrar o número: em outro terminal, `ros2 topic echo /scan/out --once | head -30`.

**Falar:**
> "Sem ruído, o scan desenha a parede como uma linha limpa. Com 3 centímetros de desvio, a mesma parede fica espalhada."

---

### Clipe 3 — Exercício 2.1: configurar o slam_toolbox (1:00 – 2:00)

**3a. Configuração (1:00 – 1:25).** Aba `config/slam_mapping.yaml`. Passe o cursor/destaque nestas linhas:
- linha 16 `base_frame: base_link` — "o padrão é base_footprint, este robô só tem base_link"
- linha 17 `scan_topic: /scan/out`
- linha 18 `mode: mapping`
- linha 25 `max_laser_range: 8.0`
- linhas 36–37 `minimum_travel_distance` / `minimum_travel_heading: 0.15` — "padrão é 0.5; assim entram mais scans no grafo"
- linha 55 `do_loop_closing: true` — "corrige a deriva quando o robô passa de novo pelo mesmo lugar"

**Falar:**
> "Configurei o slam_toolbox em modo online assíncrono. Ele precisa de três coisas: o tópico do scan, o frame do robô, e o TF odom para base_link. O resto são os parâmetros de scan matching e de fechamento de laço, com o padrão do pacote, só reduzi a distância mínima entre scans de 0.5 para 0.15 metro."

**3b. Quem publica o TF (1:25 – 1:35).** Aba `scripts/odom_tf_node.py`, linhas 52 (`if self.mode == 'truth'`) e 83–101 (`on_truth` e `on_wheels`). Ou, sem código, só diga:
> "A árvore de TF é: o slam_toolbox publica map para odom; o meu nó odom_tf_node publica odom para base_link, com a pose real do Gazebo ou com a odometria ruidosa; e o robot_state_publisher publica o Lidar."

**3c. Rodando (1:35 – 2:00).** Terminal (tudo ground truth, o caso mais limpo):

```bash
~/ros2_ws/killsim.sh
ros2 launch my_robot_slam slam.launch.py gui:=false rviz:=true
```

Grave ~25 s do RViz com o **mapa (cinza/preto) crescendo** e o robô andando. Em outro terminal, 3 s mostrando:
```bash
ros2 run tf2_ros tf2_echo map base_link
```

**Falar:**
> "Aqui, tudo ground truth. O mapa vai sendo construído conforme o robô percorre a sala; as paredes e os três obstáculos aparecem."

---

### Clipe 4 — Exercício 2.2: comparação (2:00 – 3:30)

Aqui você **não precisa rodar nada ao vivo**: os três experimentos já estão feitos e salvos (cada um leva ~3 min de simulação). Se quiser mostrar um deles rodando, grave só ~10 s do clipe 2b (com `odom:=noisy`) e use como B-roll.

**4a. Mostrar como os experimentos foram feitos (2:00 – 2:15).** Terminal:

```bash
cat src/Atividade5/my_robot_slam/scripts/run_slam_experiment.sh | head -8
```
e diga que o comando roda 2 voltas pelo mesmo percurso e salva mapa, CSV e métricas contra o ground truth:
```bash
ros2 run my_robot_slam run_slam_experiment.sh s3_ruido_odom_lidar 0.03 noisy
```
(não precisa executar; é o comando que gerou o `results/s3_...`.)

**4b. As três imagens (2:15 – 3:15).** Abra, uma a uma (~15–20 s cada), `resultado.png` de:

1. `results/s1_sem_ruido/` — "Sem ruído. A linha azul tracejada (SLAM) cobre a verde (real). Erro de posição do SLAM: **1,3 cm**. Mapa com erro médio de 3 cm."
2. `results/s2_ruido_lidar/` — "Ruído de 3 cm no Lidar, odometria perfeita. O erro do SLAM sobe para **4 cm** e as paredes ficam mais grossas no mapa, mas ele continua fechando o mapa sem deriva."
3. `results/s3_ruido_odom_lidar/` — "Ruído no Lidar e na odometria. A linha vermelha pontilhada é só a odometria: ela deriva até **4,8 metros**, o caminho nem lembra a sala. O SLAM, em azul, fica em **6 cm**, com picos de até 28 cm, e o mapa tem a mesma qualidade. Quem remove a deriva é o scan matching contra o mapa e o fechamento de laço na segunda volta."

**4c. Tabela (3:15 – 3:30).** `results/resumo.csv` (ou a tabela do README, seção "Exercício 2.2"): aponte as colunas `odom_rmse` (0,002 → 0,000 → **2,178 m**) e `slam_rmse` (0,013 → 0,040 → **0,061 m**).

**Falar (fecho do clipe):**
> "Resumindo: o ruído no Lidar piora pouco; o ruído na odometria sozinho seria desastroso, mas o SLAM compensa."

---

### Clipe 5 — Exercício 3.1: robô sequestrado (3:30 – 4:45)

Este clipe **precisa do Gazebo com interface** (`gui:=true`) e do RViz lado a lado. É o único que precisa ser gravado ao vivo, passo a passo como pede o roteiro. Use o mapa já construído na rodada `s3` (Lidar e odometria ruidosos).

**5a. Subir a localização (3:30 – 3:45).** Terminal:

```bash
~/ros2_ws/killsim.sh
ros2 launch my_robot_slam slam.launch.py gui:=true rviz:=true localization:=true \
  map_file:=$HOME/ros2_ws/src/Atividade5/my_robot_slam/results/s3_ruido_odom_lidar/posegraph \
  lidar_noise:=0.03 odom:=noisy drive:=false
```

Antes, mostre 5 s a aba `config/slam_localization.yaml`, linha 20: `mode: localization`.

**Falar:**
> "Agora o slam_toolbox carrega o mapa que eu construí, no modo localização, com ruído no Lidar e na odometria. O robô está parado na origem e o RViz mostra o robô sobre o mapa."

**5b. O sequestro, ao vivo (3:45 – 4:15).** Mostre na ordem, **com a janela do Gazebo e do RViz visíveis**:
1. Clique no botão de **Pause** do Gazebo (barra inferior, ao lado do relógio).
2. Aperte **T** (modo de translação) e **arraste o robô** pela seta até o outro lado da sala, por exemplo para perto de x = 3, y = −2.
3. Clique em **Play**.
4. Olhe o RViz por ~10–15 s: o robô estimado continua no lugar antigo, no meio do mapa, enquanto no Gazebo o robô está do outro lado.

Alternativa pela linha de comando, caso a interface falhe (é o que o script de teste usa):
```bash
gz world -p 1
gz model -m my_robot -x 3.0 -y -2.0 -z 0.1 -Y 3.14
gz world -p 0
```

**Falar:**
> "Pauso o Gazebo, seleciono o robô em modo de translação e levo ele para o outro lado do mapa. Retomo a simulação. O robô de verdade está aqui, mas a estimativa do SLAM continua lá na origem, e não corrige."

**5c. Resultado e recuperação (4:15 – 4:45).** Abra `results/k1_sequestro/resultado.png` (~10 s) e `results/k2_sequestro_initialpose/resultado.png` (~10 s). Se der tempo, mostre ao vivo a recuperação: na barra do RViz, **"2D Pose Estimate"**, clique e arraste no ponto onde o robô realmente está (veja pelo Gazebo); a estimativa pula para o lugar certo.

**Falar (~25 s, fecho do vídeo):**
> "Resultado: o erro vai de 1 centímetro para 3,6 metros no instante do sequestro e fica lá — 4,1 metros um minuto depois. A localização **não se recupera sozinha**. O slam_toolbox em modo localização só compara o scan com o mapa perto da pose que ele já acredita estar, e como a odometria não percebeu o salto, ele nunca chega perto da pose real. Testei alargar a janela de busca e deixar o robô andando: não resolve. O que resolve é dar uma pose inicial nova — o 2D Pose Estimate do RViz, que publica em initialpose: o erro cai para 4 centímetros. Para recuperar sozinho seria preciso um localizador global, como o AMCL, que espalha partículas pelo mapa inteiro."

---

## Resumo do que gravar (ordem de gravação sugerida — pela facilidade, não pela do vídeo final)

1. **Clipe 2b** (2 execuções do launch, `lidar_noise:=0.0` e `0.03`, ~10 s cada).
2. **Clipe 3c** (launch tudo ground truth, ~25 s com mapa crescendo + `tf2_echo`).
3. **Clipe 5a/5b** (localização + sequestro ao vivo; é o mais trabalhoso — faça 2 tentativas).
4. Telas estáticas: clipes 1, 2a, 3a/3b, 4 (imagens e tabela), 5c. São só abrir arquivos e falar.

Na edição, siga a **ordem da tabela "Sequência do vídeo"**. Se passar de 5:00, corte primeiro o clipe 3b (TF) e reduza o 4a.

## Se algo der errado na gravação

| problema | o que fazer |
|---|---|
| sobrou simulação antiga (robô duplicado, TF brigando) | `~/ros2_ws/killsim.sh` e relance |
| RViz sem mapa | Fixed Frame = `map`; Add → Map → topic `/map`, Durability = Transient Local |
| Gazebo trava ao abrir (WSL) | rode com `gui:=false` e mostre só o RViz; no clipe 5 use os comandos `gz world`/`gz model` |
| o robô não anda | o percurso começa 8 s depois de ligar; espere |
