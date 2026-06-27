#!/bin/bash

model="$1"
sitl_num="${2:-2}"
session="px4_multi"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src_path="$(cd "$SCRIPT_DIR/../.." && pwd)"
build_path="$src_path/build/px4_sitl_default"

stop_all()
{
	trap - INT TERM EXIT

	echo
	echo "[cleanup] Encerrando PX4 e Gazebo..."

	tmux kill-session -t "$session" 2>/dev/null || true

	pkill -TERM -x px4 2>/dev/null || true
	pkill -TERM -x gz 2>/dev/null || true
	pkill -TERM -x gzserver 2>/dev/null || true
	pkill -TERM -x gzclient 2>/dev/null || true
	pkill -TERM -f '[g]z sim' 2>/dev/null || true

	sleep 2

	pkill -KILL -x px4 2>/dev/null || true
	pkill -KILL -x gz 2>/dev/null || true
	pkill -KILL -x gzserver 2>/dev/null || true
	pkill -KILL -x gzclient 2>/dev/null || true
	pkill -KILL -f '[g]z sim' 2>/dev/null || true

	echo "[cleanup] Concluído."
}

if [ -z "$model" ]; then
	echo "Uso: $0 <modelo> [quantidade]"
	echo "Exemplo: $0 gz_x500_vision 2"
	exit 1
fi

command -v tmux >/dev/null || {
	echo "tmux não está instalado."
	exit 1
}

if [ ! -x "$build_path/bin/px4" ]; then
	echo "Execute primeiro: make px4_sitl"
	exit 1
fi

# Limpa execuções anteriores antes de começar.
stop_all

trap stop_all INT TERM EXIT

tmux new-session -d -s "$session" -c "$src_path" \
	"PX4_SIM_MODEL='$model' \
	 PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \
	 '$build_path/bin/px4' -i 0"

echo "Aguardando o Gazebo iniciar..."
sleep 5

n=1
while [ "$n" -lt "$sitl_num" ]; do
	x=$((n * 2))

	tmux split-window -t "$session" -c "$src_path" \
		"PX4_GZ_STANDALONE=1 \
		 PX4_SIM_MODEL='$model' \
		 PX4_GZ_MODEL_POSE='${x},0,0,0,0,0' \
		 '$build_path/bin/px4' -i '$n'"

	n=$((n + 1))
done

tmux select-layout -t "$session" tiled
tmux set-window-option -t "$session" synchronize-panes on

echo "Os comandos do terminal serão enviados para todos os PX4."
echo "Ctrl+C encerrará PX4, Gazebo e tmux."

tmux attach-session -t "$session"