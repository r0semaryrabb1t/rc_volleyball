#!/bin/bash

SESSION="strike"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-88}"

MOTOR_CONFIG="$WS_DIR/src/motor_control_ros2/config/motors.yaml"
STRIKE_CONFIG="$WS_DIR/src/motor_control_ros2/config/strike_node_params.yaml"
LOG_ROOT="$WS_DIR/log/strike_start"
RUN_LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"

echo "=========================================="
echo "  机械臂击球系统一键启动"
echo "=========================================="

tcd() {
  cd "$1" || { echo "工作区 $1 不存在"; exit 1; }
}

tcd "$WS_DIR"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 环境不存在: $ROS_SETUP"
  echo "可通过 ROS_SETUP=/path/to/setup.bash 指定"
  exit 1
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "未找到 $WS_DIR/install/setup.bash"
  echo "请先执行: colcon build --packages-select motor_control_ros2"
  exit 1
fi

for file in "$MOTOR_CONFIG" "$STRIKE_CONFIG"; do
  if [ ! -f "$file" ]; then
    echo "配置文件不存在: $file"
    exit 1
  fi
done

mkdir -p "$RUN_LOG_DIR"

echo ""
echo "设备状态:"
ls -l /dev/ttyUSB2 2>/dev/null || echo "  未找到 /dev/ttyUSB2"
ls -l /dev/ttyUSB3 2>/dev/null || echo "  未找到 /dev/ttyUSB3"
ls -l /dev/ttyUSB4 2>/dev/null || echo "  未找到 /dev/ttyUSB4"

ENV_VARS="unset CYCLONEDDS_URI && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE"
SOURCE_CMD="cd \"$WS_DIR\" && $ENV_VARS && source \"$ROS_SETUP\" && source \"$WS_DIR/install/setup.bash\""

send_node() {
  local pane="$1"
  local name="$2"
  local command="$3"
  local log_file="$RUN_LOG_DIR/$name.log"

  tmux send-keys -t "$pane" \
    "$SOURCE_CMD && set -o pipefail && echo [$name] log=$log_file && ( $command ) 2>&1 | tee -a \"$log_file\"; echo [$name] exited=\${PIPESTATUS[0]}" C-m
}

echo "清理旧进程..."
pkill -9 -f "unitree_motor_node|strike_node" 2>/dev/null

tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

echo ""
echo "启动 2 个节点..."

PANE_MAIN="$(tmux new-session -d -s "$SESSION" -n core -P -F "#{pane_id}" bash)"
PANE_STRIKE="$(tmux split-window -t "$PANE_MAIN" -h -P -F "#{pane_id}" bash)"

send_node "$PANE_MAIN" "unitree_motor_node" \
  "ros2 run motor_control_ros2 unitree_motor_node --ros-args -p config_file:=$MOTOR_CONFIG"
send_node "$PANE_STRIKE" "strike_node" \
  "ros2 run motor_control_ros2 strike_node"

tmux select-layout -t "$SESSION":core tiled

tmux select-window -t "$SESSION":core

echo ""
echo "tmux attach -t $SESSION"
echo "日志目录: $RUN_LOG_DIR"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "如果需要，可通过 ROS_SETUP 指定 ROS 环境，例如: ROS_SETUP=/opt/ros/humble/setup.bash ./src/ROS_test/launch/start_strike.sh"
