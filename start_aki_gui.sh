#!/usr/bin/env bash
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

# Start the RoboSoft AKI application on a development computer.
#
# Usage:
#   ./start_aki_gui.sh [physical|virtual] [CAN interface] [ROS launch arguments...]
#   ./start_aki_gui.sh virtual vcan0 enable_uvc_sequence:=false
#
# The default is physical operation with can0. The script sources the ROS 2
# and workspace installations, starts the Qt GUI, optionally starts the SICK
# TiM5xx driver and then launches the AKI control stack. Virtual mode creates
# vcan when needed and selects the simulator CAN identity, pseudo-terminal
# GNSS endpoint and localhost lidar address. The simulator itself is started
# separately with start_aki_simulator.sh.
#
# Environment overrides:
#   ROS_DISTRO              ROS 2 distribution name (default jazzy)
#   SICK_LIDAR_HOSTNAME     TiM5xx address (physical default 192.168.1.160;
#                           virtual default 127.0.0.1)
#   START_SICK_LIDAR        auto, true or false (auto currently enables it)
#   AKI_SIMULATION          true to use simulator identities in physical mode
#
# Additional arguments are forwarded to aki_nodes.launch.py. Unless explicitly
# overridden, TaskManager uses robosoft_core/tasks from this source workspace.
# SIGINT and SIGTERM stop the GUI and lidar child processes before exit.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
GUI_CONFIG="${WORKSPACE_DIR}/install/robosoft_core/share/robosoft_core/config/aki_gui_params.yaml"
SICK_LIDAR_HOSTNAME="${SICK_LIDAR_HOSTNAME:-}"
GUI_PID=""
SICK_PID=""
CLEANUP_STARTED="false"
INTERFACE_TYPE=""
CAN_INTERFACE=""
START_SICK_LIDAR="${START_SICK_LIDAR:-auto}"
AKI_SIMULATION="${AKI_SIMULATION:-false}"

if [[ -n "${1:-}" && "${1}" != "physical" && "${1}" != "virtual" && "${1}" != *":="* ]]; then
  echo "Usage: $0 [physical|virtual] [CAN interface] [ROS launch arguments...]" >&2
  exit 2
fi
if [[ "${1:-}" == "physical" || "${1:-}" == "virtual" ]]; then
  INTERFACE_TYPE="$1"
  shift
  if [[ -n "${1:-}" && "${1}" != *":="* ]]; then
    CAN_INTERFACE="$1"
    shift
  fi
fi

AKI_LAUNCH_ARGS=("$@")
SERIAL_DEVICE=""
LOCAL_TASK_DIRECTORY="${WORKSPACE_DIR}/src/robosoft_core/tasks"

for launch_argument in "$@"; do
  case "${launch_argument}" in
    can_interface:=*) CAN_INTERFACE="${launch_argument#can_interface:=}" ;;
    serial_device:=*) SERIAL_DEVICE="${launch_argument#serial_device:=}" ;;
  esac
done

if [[ -z "${CAN_INTERFACE}" ]]; then
  CAN_INTERFACE="$(
    printf '%s\n' "$@" | sed -n 's/^can_interface:=//p' | tail -n 1
  )"
fi
if [[ -z "${CAN_INTERFACE}" ]]; then
  CAN_INTERFACE="$([[ "${INTERFACE_TYPE}" == "virtual" ]] && echo vcan0 || echo can0)"
fi
if [[ -z "${INTERFACE_TYPE}" ]]; then
  INTERFACE_TYPE="$([[ "${CAN_INTERFACE}" == vcan* ]] && echo virtual || echo physical)"
fi
if [[ -z "${SICK_LIDAR_HOSTNAME}" ]]; then
  SICK_LIDAR_HOSTNAME="$([[ "${INTERFACE_TYPE}" == "virtual" ]] && echo 127.0.0.1 || echo 192.168.1.160)"
fi
if ! printf '%s\n' "$@" | grep -q '^can_interface:='; then
  AKI_LAUNCH_ARGS+=("can_interface:=${CAN_INTERFACE}")
fi
if ! printf '%s\n' "$@" | grep -q '^task_directory:='; then
  AKI_LAUNCH_ARGS+=("task_directory:=${LOCAL_TASK_DIRECTORY}")
fi

if [[ "${INTERFACE_TYPE}" == "virtual" ]]; then
  AKI_SIMULATION="true"
  if ! ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
    echo "Creating virtual CAN interface ${CAN_INTERFACE}..."
    sudo modprobe vcan
    sudo ip link add dev "${CAN_INTERFACE}" type vcan
    sudo ip link set up "${CAN_INTERFACE}"
  fi
fi
if ! ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
  echo "CAN interface not found: ${CAN_INTERFACE}" >&2
  exit 1
fi

if [[ "${START_SICK_LIDAR}" == "auto" ]]; then
  START_SICK_LIDAR="true"
fi

if [[ "${AKI_SIMULATION}" == "true" ]]; then
  if ! printf '%s\n' "$@" | grep -q '^tecu_server_name:='; then
    AKI_LAUNCH_ARGS+=("tecu_server_name:=01006007008600A0")
  fi
  if ! printf '%s\n' "$@" | grep -q '^tecu_server_address:='; then
    AKI_LAUNCH_ARGS+=("tecu_server_address:=240")
  fi
  if ! printf '%s\n' "$@" | grep -q '^serial_device:='; then
    SERIAL_DEVICE="/tmp/aki_gnss_robot"
    AKI_LAUNCH_ARGS+=("serial_device:=/tmp/aki_gnss_robot")
  fi
  if ! printf '%s\n' "$@" | grep -q '^enable_lidar:='; then
    AKI_LAUNCH_ARGS+=("enable_lidar:=${START_SICK_LIDAR}")
  fi
  if ! printf '%s\n' "$@" | grep -q '^disable_can_loopback:='; then
    AKI_LAUNCH_ARGS+=("disable_can_loopback:=false")
  fi
  if ! printf '%s\n' "$@" | grep -q '^remote_watchdog_timeout_ms:='; then
    AKI_LAUNCH_ARGS+=("remote_watchdog_timeout_ms:=1000")
  fi
fi

for required_file in "${ROS_SETUP}" "${WORKSPACE_SETUP}" "${GUI_CONFIG}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

# ROS setup scripts inspect optional unset variables. Enable nounset only
# after sourcing them.
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"
set -u

stop_process() {
  local process_id="$1"
  local watchdog_pid

  [[ -n "${process_id}" ]] || return
  if ! kill -0 "${process_id}" 2>/dev/null; then
    wait "${process_id}" 2>/dev/null || true
    return
  fi

  kill -INT "${process_id}" 2>/dev/null || true
  (
    sleep 3
    kill -TERM "${process_id}" 2>/dev/null || exit 0
    sleep 2
    kill -KILL "${process_id}" 2>/dev/null || true
  ) &
  watchdog_pid=$!
  wait "${process_id}" 2>/dev/null || true
  kill "${watchdog_pid}" 2>/dev/null || true
  wait "${watchdog_pid}" 2>/dev/null || true
}

cleanup() {
  [[ "${CLEANUP_STARTED}" == "false" ]] || return
  CLEANUP_STARTED="true"
  stop_process "${GUI_PID}"
  stop_process "${SICK_PID}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting RoboSoft AKI GUI..."
ros2 launch robosoft_core gui.launch.py \
  "config_file:=${GUI_CONFIG}" \
  "task_directory:=${LOCAL_TASK_DIRECTORY}" &
GUI_PID=$!

echo "Waiting for the GUI to subscribe to /rosout..."
for attempt in {1..60}; do
  if ! kill -0 "${GUI_PID}" 2>/dev/null; then
    echo "RoboSoft AKI GUI exited during startup." >&2
    exit 1
  fi
  if ros2 node info /robosoft_gui_node 2>/dev/null |
      grep -qE '^[[:space:]]+/rosout:[[:space:]]+rcl_interfaces/msg/Log'; then
    break
  fi
  if [[ "${attempt}" -eq 60 ]]; then
    echo "Timed out waiting for the GUI /rosout subscription." >&2
    exit 1
  fi
  sleep 0.5
done

if [[ "${START_SICK_LIDAR}" == "true" ]]; then
  if ! SICK_SHARE="$(ros2 pkg prefix --share sick_scan_xd 2>/dev/null)"; then
    echo "sick_scan_xd is not installed." >&2
    exit 1
  fi
  SICK_LAUNCH="${SICK_SHARE}/launch/sick_tim_5xx.launch"

  echo "Starting SICK TiM5xx at ${SICK_LIDAR_HOSTNAME}..."
  ros2 run sick_scan_xd sick_generic_caller \
    "${SICK_LAUNCH}" "hostname:=${SICK_LIDAR_HOSTNAME}" &
  SICK_PID=$!
  sleep 1
  if ! kill -0 "${SICK_PID}" 2>/dev/null; then
    echo "SICK TiM5xx driver exited during startup." >&2
    exit 1
  fi
else
  echo "Physical SICK lidar startup is disabled for simulation."
fi

echo "GUI is ready. Starting AKI nodes on ${INTERFACE_TYPE} interface ${CAN_INTERFACE}..."
ros2 launch robosoft_aki aki_nodes.launch.py "${AKI_LAUNCH_ARGS[@]}"
