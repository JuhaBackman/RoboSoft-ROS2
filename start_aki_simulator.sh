#!/usr/bin/env bash
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

# Start the hardware-boundary simulator used by the AKI example.
#
# Usage:
#   ./start_aki_simulator.sh [physical|virtual] [CAN interface] [GNSS device]
#   ./start_aki_simulator.sh virtual vcan0 /tmp/aki_gnss_sim
#
# The default is physical operation with can0 and /dev/ttyUSB0. Physical mode
# expects existing CAN and serial interfaces and leaves transport delays to the
# hardware. Virtual mode creates vcan when needed, creates and owns the paired
# /tmp/aki_gnss_sim and /tmp/aki_gnss_robot pseudo-terminals when the default
# GNSS endpoint is used, and enables the configured 100 ms simulated transport
# delays. The launch runs in simulator-only ROS domain 42.
#
# Environment overrides:
#   ROS_DISTRO              ROS 2 distribution name (default jazzy)
#   ENABLE_JOYSTICK         start joy_node (default true)
#   JOYSTICK_DEVICE_ID      Linux joystick index (default 0)
#   ENABLE_LIDAR_SIMULATOR  start TASK cloud and TiM5xx simulation (default true)
#
# SIGINT and SIGTERM stop the launch and socat child processes. Pseudo-terminal
# links created by this invocation are removed during cleanup.

set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENABLE_JOYSTICK="${ENABLE_JOYSTICK:-true}"
ENABLE_LIDAR_SIMULATOR="${ENABLE_LIDAR_SIMULATOR:-true}"
JOYSTICK_DEVICE_ID="${JOYSTICK_DEVICE_ID:-0}"
LAUNCH_PID=""
SOCAT_PID=""
SERIAL_PAIR_CREATED="false"
CLEANUP_STARTED="false"
INTERFACE_TYPE="${1:-physical}"
if [[ "${INTERFACE_TYPE}" != "physical" && "${INTERFACE_TYPE}" != "virtual" ]]; then
  echo "Usage: $0 [physical|virtual] [CAN interface] [GNSS serial device]" >&2
  exit 2
fi

if [[ "${INTERFACE_TYPE}" == "virtual" ]]; then
  CAN_INTERFACE="${2:-vcan0}"
  GNSS_DEVICE="${3:-/tmp/aki_gnss_sim}"
  DISABLE_CAN_LOOPBACK="false"
  COMMAND_DELAY_S="0.1"
  MEASUREMENT_DELAY_S="0.1"
  if ! ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
    echo "Creating virtual CAN interface ${CAN_INTERFACE}..."
    sudo modprobe vcan
    sudo ip link add dev "${CAN_INTERFACE}" type vcan
    sudo ip link set up "${CAN_INTERFACE}"
  fi
else
  CAN_INTERFACE="${2:-can0}"
  GNSS_DEVICE="${3:-/dev/ttyUSB0}"
  DISABLE_CAN_LOOPBACK="true"
  # Physical CAN and RS-232 already contribute their real transport delays.
  COMMAND_DELAY_S="0.0"
  MEASUREMENT_DELAY_S="0.0"
fi

if ! ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
  echo "CAN interface not found: ${CAN_INTERFACE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace has not been built: ${WORKSPACE_DIR}/install/setup.bash" >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
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
  stop_process "${LAUNCH_PID}"
  stop_process "${SOCAT_PID}"
  if [[ "${SERIAL_PAIR_CREATED}" == "true" ]]; then
    [[ -L /tmp/aki_gnss_sim ]] && unlink /tmp/aki_gnss_sim
    [[ -L /tmp/aki_gnss_robot ]] && unlink /tmp/aki_gnss_robot
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${INTERFACE_TYPE}" == "virtual" && -f /tmp/aki_gnss_pair.pid ]]; then
  # Remove a detached pair created by the previous startup implementation.
  # Validate the process command before signalling the stored PID so a reused
  # PID can never terminate an unrelated process.
  read -r PREVIOUS_SOCAT_PID </tmp/aki_gnss_pair.pid || PREVIOUS_SOCAT_PID=""
  if [[ "${PREVIOUS_SOCAT_PID}" =~ ^[0-9]+$ ]] &&
    kill -0 "${PREVIOUS_SOCAT_PID}" 2>/dev/null &&
    [[ -r "/proc/${PREVIOUS_SOCAT_PID}/cmdline" ]] &&
    tr '\0' ' ' <"/proc/${PREVIOUS_SOCAT_PID}/cmdline" |
      grep -q 'socat.*link=/tmp/aki_gnss_sim.*link=/tmp/aki_gnss_robot'; then
    kill -TERM "${PREVIOUS_SOCAT_PID}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${PREVIOUS_SOCAT_PID}" 2>/dev/null || break
      sleep 0.05
    done
  fi
  unlink /tmp/aki_gnss_pair.pid
  [[ -L /tmp/aki_gnss_sim ]] && unlink /tmp/aki_gnss_sim
  [[ -L /tmp/aki_gnss_robot ]] && unlink /tmp/aki_gnss_robot
fi

if [[ "${INTERFACE_TYPE}" == "virtual" && "${GNSS_DEVICE}" == "/tmp/aki_gnss_sim" ]] &&
  [[ ! -e /tmp/aki_gnss_sim || ! -e /tmp/aki_gnss_robot ]]; then
  if ! command -v socat >/dev/null 2>&1; then
    echo "socat is required for the virtual GNSS serial pair." >&2
    echo "Install it with: sudo apt install socat" >&2
    exit 1
  fi
  [[ -L /tmp/aki_gnss_sim ]] && unlink /tmp/aki_gnss_sim
  [[ -L /tmp/aki_gnss_robot ]] && unlink /tmp/aki_gnss_robot
  echo "Creating virtual GNSS serial pair..."
  socat \
    pty,raw,echo=0,link=/tmp/aki_gnss_sim \
    pty,raw,echo=0,link=/tmp/aki_gnss_robot &
  SOCAT_PID=$!
  SERIAL_PAIR_CREATED="true"
  for _ in {1..20}; do
    [[ -e /tmp/aki_gnss_sim && -e /tmp/aki_gnss_robot ]] && break
    sleep 0.1
  done
  if [[ ! -e /tmp/aki_gnss_sim || ! -e /tmp/aki_gnss_robot ]]; then
    echo "Failed to create the virtual GNSS serial pair." >&2
    exit 1
  fi
fi

echo "Starting AKI simulator on ${INTERFACE_TYPE} interface ${CAN_INTERFACE}..."
if [[ "${ENABLE_JOYSTICK}" == "true" ]]; then
  echo "Starting joystick ${JOYSTICK_DEVICE_ID} for physical remote simulation..."
  echo "Monitor it with: ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 topic echo /joy"
fi
ros2 launch robosoft_simulator aki_can_simulator.launch.py \
  "can_interface:=${CAN_INTERFACE}" \
  "disable_can_loopback:=${DISABLE_CAN_LOOPBACK}" \
  "domain_id:=42" \
  "enable_joystick:=${ENABLE_JOYSTICK}" \
  "enable_lidar_simulator:=${ENABLE_LIDAR_SIMULATOR}" \
  "joystick_device_id:=${JOYSTICK_DEVICE_ID}" \
  "gnss_serial_device:=${GNSS_DEVICE}" \
  "command_delay_s:=${COMMAND_DELAY_S}" \
  "measurement_delay_s:=${MEASUREMENT_DELAY_S}" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}"
