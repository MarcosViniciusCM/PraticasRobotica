#!/usr/bin/env bash
# Pratica 05.2 - roda UMA situacao de SLAM headless, salva mapa/metricas em results/<rotulo>/.
#
#   run_slam_experiment.sh <rotulo> <lidar_noise_std> <odom: truth|noisy> [odom_noise_scale] [laps]
#
# Saida: results/<rotulo>/{map.pgm,map.yaml,posegraph.*,slam.csv,metricas.txt,resultado.png,launch.log}
# e uma linha em results/resumo.csv
set -u
LABEL=${1:?rotulo}; LIDAR=${2:?lidar_noise_std}; ODOM=${3:?truth|noisy}; ONOISE=${4:-1.0}; LAPS=${5:-2}
HERE="$(ros2 pkg prefix my_robot_slam)/lib/my_robot_slam"
RES="${RES:-$HOME/ros2_ws/src/Atividade5/my_robot_slam/results}"
OUT="$RES/$LABEL"; mkdir -p "$OUT"
LOG="$OUT/launch.log"
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1

echo ">> $LABEL  lidar_noise=$LIDAR m  odometria=$ODOM (noise_scale=$ONOISE)"
setsid ros2 launch my_robot_slam slam.launch.py gui:=false lidar_noise:="$LIDAR" odom:="$ODOM" \
  odom_noise:="$ONOISE" laps:="$LAPS" log_file:="$OUT/slam.csv" > "$LOG" 2>&1 < /dev/null &
until grep -q "TRAJETORIA CONCLUIDA" "$LOG" 2> /dev/null; do sleep 3; done
sleep 8   # deixa o SLAM terminar de processar os ultimos scans
python3 "$HERE/save_map.py" "$OUT/map"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/posegraph'}" > "$OUT/serialize.txt" 2>&1
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1
python3 "$HERE/analyze.py" "$OUT/slam.csv" "$OUT/map.pgm" --out "$OUT" --label "$LABEL" \
  --append "$RES/resumo.csv" --title "$LABEL (Lidar std=$LIDAR m, odometria $ODOM x$ONOISE)" | tee "$OUT/metricas.txt"
