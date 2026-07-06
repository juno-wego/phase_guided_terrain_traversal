#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS_DIR="${SCRIPT_DIR}/../ros_ws"
DEFAULT_POLICY_FILE="${SCRIPT_DIR}/../policies/policy_go2_pgtt_level13_run0"

cleanup() {
  pkill -f 'ros2 launch wego_pgtt go2_stair_policy_mode.launch.py|wego_pgtt_policy_mode_server|deploy_real.py' || true
}

trap cleanup EXIT INT TERM

cd "${ROS_WS_DIR}"
source /opt/ros/humble/setup.bash

if [ -f /home/wego/dddmr_navigation/install/setup.bash ]; then
  source /home/wego/dddmr_navigation/install/setup.bash
fi

cleanup
sleep 1

colcon build --symlink-install --packages-select \
  wego_pgtt

source install/local_setup.bash

set -u

ros2 launch wego_pgtt go2_stair_policy_mode.launch.py \
  auto_start_go2_driver:=false \
  command_source:=hybrid \
  policy_file:="${DEFAULT_POLICY_FILE}" \
  debug_inputs:=true \
  debug_print_hz:=1.0 \
  "$@" &
POLICY_SERVER_PID=$!

sleep 2

echo "Policy server is ready."
echo "Start policy with:"
echo "  ros2 service call /toggle_pgtt_mode std_srvs/srv/Trigger \"{}\""

wait "${POLICY_SERVER_PID}"
