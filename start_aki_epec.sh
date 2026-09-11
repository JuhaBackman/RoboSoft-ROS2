#!/usr/bin/env bash
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

# Start RoboSoft AKI on the EPEC embedded target.
#
# Usage:
#   ./start_aki_epec.sh [ROS launch arguments...]
#   ./start_aki_epec.sh serial_device:=/dev/ttymxc2 enable_lidar:=false
#
# This is the embedded counterpart of start_aki_gui.sh. It supports physical
# hardware only and expects the host system to configure can0 before starting
# the application container. The script locates and sources the ROS 2 base,
# optional sick_scan_xd vendor overlay and ARMv7 RoboSoft installation. It
# seeds /opt/ros2_ws/tasks from the installed TASK files without replacing
# a newer runtime file, starts the GUI, optionally starts the SICK driver and
# finally launches the AKI control stack.
#
# Environment overrides:
#   ROBOSOFT_INSTALL_PREFIX     RoboSoft installation prefix
#   SICK_LIDAR_HOSTNAME         TiM5xx address (default 192.168.1.160)
#   START_SICK_LIDAR            true or false (default true)
#   GUI_STARTUP_DELAY_SECONDS   GUI initialization delay (default 2)
#
# Additional arguments are forwarded to aki_nodes.launch.py. This script
# always uses can0 and rejects any other can_interface value. SIGINT and
# SIGTERM are forwarded through the cleanup routine to child processes.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
CAN_INTERFACE="can0"
SICK_LIDAR_HOSTNAME="${SICK_LIDAR_HOSTNAME:-192.168.1.160}"
START_SICK_LIDAR="${START_SICK_LIDAR:-true}"
GUI_STARTUP_DELAY_SECONDS="${GUI_STARTUP_DELAY_SECONDS:-2}"
GUI_PID=""
SICK_PID=""
CLEANUP_STARTED="false"

if [[ -n "${ROBOSOFT_INSTALL_PREFIX:-}" ]]; then
  INSTALL_PREFIX="${ROBOSOFT_INSTALL_PREFIX}"
elif [[ -f "/opt/ros2_ws/install-armv7/setup.bash" ]]; then
  INSTALL_PREFIX="/opt/ros2_ws/install-armv7"
elif [[ -f "${WORKSPACE_DIR}/install-armv7/setup.bash" ]]; then
  INSTALL_PREFIX="${WORKSPACE_DIR}/install-armv7"
else
  INSTALL_PREFIX="${WORKSPACE_DIR}/install"
fi

WORKSPACE_SETUP="${INSTALL_PREFIX}/local_setup.bash"
GUI_CONFIG="${INSTALL_PREFIX}/robosoft_core/share/robosoft_core/config/aki_gui_params.yaml"
TASK_DIRECTORY="/opt/ros2_ws/tasks"
INSTALLED_TASK_DIRECTORY="${INSTALL_PREFIX}/robosoft_core/share/robosoft_core/tasks"
AKI_LAUNCH_ARGS=("$@")

for launch_argument in "$@"; do
  case "${launch_argument}" in
    can_interface:=can0) ;;
    can_interface:=*)
      echo "The EPEC startup script supports only can0." >&2
      exit 2
      ;;
  esac
done
if ! printf '%s\n' "$@" | grep -q '^can_interface:='; then
  AKI_LAUNCH_ARGS+=("can_interface:=${CAN_INTERFACE}")
fi
if ! printf '%s\n' "$@" | grep -q '^enable_lidar:='; then
  AKI_LAUNCH_ARGS+=("enable_lidar:=${START_SICK_LIDAR}")
fi

ROS_SETUP=""
for setup_candidate in \
  "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" \
  "/opt/ros2_${ROS_DISTRO_NAME}/setup.bash" \
  "/opt/ros2_${ROS_DISTRO_NAME}/install/setup.bash" \
  "/root/ros2_ws/install/setup.bash"; do
  if [[ -f "${setup_candidate}" ]]; then
    ROS_SETUP="${setup_candidate}"
    break
  fi
done

if [[ -z "${ROS_SETUP}" ]]; then
  echo "ROS 2 base setup.bash was not found." >&2
  exit 1
fi
for required_file in "${WORKSPACE_SETUP}" "${GUI_CONFIG}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

# ROS setup scripts inspect optional unset variables. Enable nounset only
# after sourcing them.
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# The EPEC runtime image provides the SICK driver as a separate vendor overlay.
if [[ -f "/opt/sick_scan_xd/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/sick_scan_xd/local_setup.bash"
fi
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"
set -u

# Keep mutable task files outside the ROS installation. Seed missing files and
# refresh older runtime copies from robosoft_core. A TASK edited on the robot
# after deployment has a newer timestamp and is therefore preserved.
mkdir -p "${TASK_DIRECTORY}"
if [[ -d "${INSTALLED_TASK_DIRECTORY}" ]]; then
  for installed_task in "${INSTALLED_TASK_DIRECTORY}"/*; do
    [[ -f "${installed_task}" ]] || continue
    runtime_task="${TASK_DIRECTORY}/$(basename -- "${installed_task}")"
    if [[ ! -e "${runtime_task}" || "${installed_task}" -nt "${runtime_task}" ]]; then
      echo "Updating runtime TASK: $(basename -- "${installed_task}")"
      cp "${installed_task}" "${runtime_task}"
    fi
  done
fi

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
  "task_directory:=${TASK_DIRECTORY}" &
GUI_PID=$!

# The GUI creates its /rosout subscription during GuiNode construction. Avoid
# querying the ROS graph with the CLI here: ros2 node info can block for a long
# time on an embedded target without a running ROS daemon. A short delay plus
# a process-health check is sufficient before the remaining nodes start.
echo "Waiting ${GUI_STARTUP_DELAY_SECONDS}s for the GUI to initialize..."
sleep "${GUI_STARTUP_DELAY_SECONDS}"
if ! kill -0 "${GUI_PID}" 2>/dev/null; then
  echo "RoboSoft AKI GUI exited during startup." >&2
  exit 1
fi

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
  echo "SICK lidar startup is disabled."
fi

echo "GUI is ready. Starting AKI nodes on EPEC ${CAN_INTERFACE}..."
ros2 launch robosoft_aki aki_nodes.launch.py "${AKI_LAUNCH_ARGS[@]}"
