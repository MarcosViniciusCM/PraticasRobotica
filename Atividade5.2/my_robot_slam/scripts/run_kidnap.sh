#!/usr/bin/env bash
# Pratica 05.2 - exercicio 3.1: robo sequestrado, headless.
#
#   [DRIVE=true: o robo segue o percurso durante o teste] [SLAM_PARAMS=slam_localization_busca_larga.yaml] run_kidnap.sh <rotulo> <pasta_do_mapa> <lidar_noise_std> [odom_noise_scale] [initialpose: sim|nao]
#
# <pasta_do_mapa> e o results/<rotulo>/ de uma rodada de mapeamento (tem posegraph.*).
# Passos (os mesmos da interface do Gazebo - botao de pausa e modo de translacao):
#   1. localizacao com o mapa pronto, robo parado na origem, ~25 s de simulacao;
#   2. pausa a simulacao                  (gz world -p 1)
#   3. teleporta o robo para o outro lado (gz model -m my_robot -x .. -y .. -Y ..)
#   4. retoma a simulacao                 (gz world -p 0) e acompanha ~60 s;
#   5. opcional: se initialpose=sim, publica a pose real em /initialpose (o "2D Pose Estimate" do RViz).
set -u
LABEL=${1:?rotulo}; MAPDIR=${2:?pasta do mapa}; LIDAR=${3:?lidar_noise_std}; ONOISE=${4:-1.0}; INIT=${5:-nao}
KX=${KX:-3.0}; KY=${KY:--2.0}; KYAW=${KYAW:-3.14159}
HERE="$(ros2 pkg prefix my_robot_slam)/lib/my_robot_slam"
RES="${RES:-$HOME/ros2_ws/src/Atividade5/my_robot_slam/results}"
OUT="$RES/$LABEL"; mkdir -p "$OUT"
LOG="$OUT/launch.log"
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1

echo ">> $LABEL  mapa=$MAPDIR  Lidar=$LIDAR m  odometria ruidosa x$ONOISE  sequestro para ($KX, $KY)"
setsid ros2 launch my_robot_slam slam.launch.py gui:=false localization:=true map_file:="$MAPDIR/posegraph" \
  ${SLAM_PARAMS:+slam_params:=$SLAM_PARAMS} lidar_noise:="$LIDAR" odom:=noisy odom_noise:="$ONOISE" drive:="${DRIVE:-false}" log_file:="$OUT/slam.csv" \
  > "$LOG" 2>&1 < /dev/null &
until grep -q "Registering sensor" "$LOG" 2> /dev/null; do sleep 2; done
# espera 25 s de simulacao (o robo esta parado e o SLAM localizado)
sleep 30
echo "[kidnap] pausa e move o robo"
gz world -p 1
gz model -m my_robot -x "$KX" -y "$KY" -z 0.1 -Y "$KYAW"
sleep 2
echo "[kidnap] retoma"
gz world -p 0
if [ "$INIT" = "sim" ]; then
  sleep 20
  echo "[kidnap] /initialpose com a pose real"
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $KX, y: $KY}, orientation: {z: $(python3 -c "import math;print(math.sin($KYAW/2))"), w: $(python3 -c "import math;print(math.cos($KYAW/2))")}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" > /dev/null
  sleep 40
else
  sleep 60
fi
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1
python3 "$HERE/analyze_kidnap.py" "$OUT/slam.csv" --out "$OUT" --label "$LABEL" | tee "$OUT/metricas.txt"
