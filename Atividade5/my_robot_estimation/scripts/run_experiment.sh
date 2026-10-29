#!/usr/bin/env bash
# Pratica 05.1 - roda UM experimento do 8 headless, analisa e grava os resultados.
#
#   run_experiment.sh <rotulo> <noise_scale> [state_topic] [laps] [Omega] [d] [kp]
#
#   state_topic  /odometry/filtered (padrao, controle com o filtro de Kalman)
#                /odom/noisy        (controle com a odometria bruta ruidosa)
#                /ground_truth/odom (controle com o estado perfeito, referencia)
#
# Saida em results/<rotulo>/ (CSV bruto, graficos) e uma linha em results/resumo.csv
set -u
LABEL=${1:?rotulo}; NOISE=${2:?noise_scale}; STATE=${3:-/odometry/filtered}; LAPS=${4:-2}; OMEGA=${5:-0.25}; D=${6:-0.3}; KP=${7:-2.0}
HERE="$(ros2 pkg prefix my_robot_estimation)/lib/my_robot_estimation"
RES="${RES:-$HOME/ros2_ws/src/Atividade4/my_robot_estimation/results}"
OUT="$RES/$LABEL"; mkdir -p "$OUT"
LOG="$OUT/launch.log"
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1

echo ">> $LABEL  noise_scale=$NOISE  estado do controle=$STATE"
setsid ros2 launch my_robot_estimation figure8_estimated.launch.py gui:=false \
  noise_scale:="$NOISE" state_topic:="$STATE" laps:="$LAPS" Omega:="$OMEGA" d:="$D" kp:="$KP" log_file:="$OUT/estado.csv" \
  > "$LOG" 2>&1 < /dev/null &
until grep -q "TRAJETORIA CONCLUIDA" "$LOG" 2> /dev/null; do sleep 3; done
sleep 2
"$HOME/ros2_ws/killsim.sh" > /dev/null 2>&1
grep -E "RMSE|maximo" "$LOG" | sed 's/.*figure8_tracker\]: //' | tee "$OUT/controle.txt"
python3 "$HERE/analyze.py" "$OUT/estado.csv" --out "$OUT" --label "$LABEL" --append "$RES/resumo.csv" \
  --title "$LABEL (noise_scale=$NOISE, controle com $STATE)" | tee "$OUT/metricas.txt"
