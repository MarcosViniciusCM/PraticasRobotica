#!/usr/bin/env bash
# Pratica 03 - bonus do exercicio 1.2.
#
# Executa a mesma missao (config/mission.yaml) com o controle de pose continuo
# e com o de tres manobras, em simulacao headless, e compara tempo total,
# distancia percorrida e erro final medio.
#
#   compare_mission.sh
#
# Variaveis opcionais:  OUT=arquivo.csv  TIMEOUT=400
set -u

OUT=${OUT:-$HOME/mission_compare_$(date +%Y%m%d_%H%M%S).csv}
TIMEOUT=${TIMEOUT:-400}

cleanup_sim() {
  pkill -INT -f "gzserver" 2>/dev/null
  pkill -INT -f "gzclient" 2>/dev/null
  sleep 3
  pkill -9 -f "gzserver|gzclient" 2>/dev/null
  pkill -f "diff_drive_bridge|robot_state_publisher|pose_controller|three_step_controller|mission_follower" 2>/dev/null
  sleep 1
}

for controller in continuous three_step; do
  echo ">>> missao com controlador '$controller'"
  cleanup_sim
  timeout --signal=INT "$TIMEOUT" ros2 launch my_robot_trajectory mission.launch.py \
    gui:=false controller:="$controller" shutdown_when_done:=true results_file:="$OUT" \
    2>&1 | grep --line-buffered -E "mission_follower|ERROR" || true
done
cleanup_sim

echo
echo "==================== resultados ($OUT)"
column -s, -t < "$OUT"
