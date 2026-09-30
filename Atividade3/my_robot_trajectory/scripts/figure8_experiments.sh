#!/usr/bin/env bash
# Pratica 03 - bonus dos exercicios 2.1 e 2.2.
#
# Roda o 8 em simulacao headless (sem gzclient), uma vez por configuracao,
# reiniciando o Gazebo entre as rodadas para todas partirem da mesma condicao.
# Cada rodada acrescenta uma linha no CSV de resultados.
#
#   figure8_experiments.sh rmse                    # malha aberta, Kff=0, Kff=1
#   figure8_experiments.sh omega 0.5 0.75 1.0 1.25 # varre Omega (realimentado, Kff=1)
#
# Variaveis opcionais:  OUT=arquivo.csv  LAPS=2  TIMEOUT=300  THRESHOLD=0.10
set -u

MODE=${1:-rmse}
shift || true
OUT=${OUT:-$HOME/figure8_${MODE}_$(date +%Y%m%d_%H%M%S).csv}
LAPS=${LAPS:-2}
TIMEOUT=${TIMEOUT:-300}
THRESHOLD=${THRESHOLD:-0.10}   # RMSE de posicao (m) abaixo do qual "segue"

cleanup_sim() {
  pkill -INT -f "gzserver" 2>/dev/null
  pkill -INT -f "gzclient" 2>/dev/null
  sleep 3
  pkill -9 -f "gzserver|gzclient" 2>/dev/null
  # processos do sistema anterior que eventualmente ficaram orfaos
  pkill -f "diff_drive_bridge|robot_state_publisher|figure8_tracker" 2>/dev/null
  sleep 1
}

run() {  # label, argumentos extras do launch
  local label=$1; shift
  echo ">>> rodada '$label': $*"
  cleanup_sim
  timeout --signal=INT "$TIMEOUT" ros2 launch my_robot_trajectory figure8.launch.py \
    gui:=false shutdown_when_done:=true results_file:="$OUT" label:="$label" \
    laps:="$LAPS" "$@" 2>&1 | grep --line-buffered -E "figure8_tracker|ERROR" || true
  if ! grep -q "^$label," "$OUT" 2>/dev/null; then
    echo "!!! rodada '$label' nao gerou resultado (timeout de ${TIMEOUT}s?)"
  fi
}

case "$MODE" in
  rmse)
    run open_loop mode:=open_loop
    run feedback_kff0 mode:=feedback kff:=0.0
    run feedback_kff1 mode:=feedback kff:=1.0
    ;;
  omega)
    [ $# -gt 0 ] || set -- 0.5 0.75 1.0 1.25 1.5
    for w in "$@"; do
      run "omega_$w" mode:=feedback kff:=1.0 Omega:="$w"
    done
    ;;
  *)
    echo "uso: $0 rmse | omega [Omega ...]"; exit 1 ;;
esac
cleanup_sim

echo
echo "==================== resultados ($OUT)"
column -s, -t < "$OUT"
if [ "$MODE" = omega ]; then
  echo
  echo "Maior Omega com RMSE de posicao < ${THRESHOLD} m:"
  awk -F, -v th="$THRESHOLD" 'NR>1 && $6+0 < th && $4+0 > best {best=$4+0} END {print (best ? best " rad/s" : "nenhum")}' "$OUT"
fi
