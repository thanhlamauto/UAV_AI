#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_DESKTOP="${INSTALL_DESKTOP:-1}"
INSTALL_ROS_GZ="${INSTALL_ROS_GZ:-1}"

log() {
  printf '[ros2-setup] %s\n' "$*"
}

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

require_ubuntu() {
  if [[ ! -r /etc/os-release ]]; then
    log "Cannot find /etc/os-release"
    exit 1
  fi
  # shellcheck source=/dev/null
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" ]]; then
    log "This setup script expects Ubuntu; found ID=${ID:-unknown}"
    exit 1
  fi
  if [[ "${VERSION_ID:-}" != "22.04" && "${ROS_DISTRO}" == "humble" ]]; then
    log "ROS2 Humble is best supported on Ubuntu 22.04; found VERSION_ID=${VERSION_ID:-unknown}"
  fi
}

ensure_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    SUDO=()
  else
    if ! command -v sudo >/dev/null 2>&1; then
      log "sudo is required when not running as root"
      exit 1
    fi
    SUDO=(sudo)
  fi
}

add_ros_apt_repo_if_needed() {
  if [[ -f "/etc/apt/sources.list.d/ros2.list" ]]; then
    log "ROS2 apt source already exists"
    return
  fi

  log "Adding ROS2 apt source"
  "${SUDO[@]}" apt update
  "${SUDO[@]}" apt install -y software-properties-common curl gnupg lsb-release
  "${SUDO[@]}" add-apt-repository universe -y
  "${SUDO[@]}" install -m 0755 -d /etc/apt/keyrings
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | "${SUDO[@]}" tee /etc/apt/keyrings/ros-archive-keyring.gpg >/dev/null
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

install_packages() {
  local packages=(
    python3-colcon-common-extensions
    ffmpeg
    python3-numpy
    python3-venv
    python3-pip
  )

  if [[ "${INSTALL_DESKTOP}" == "1" ]]; then
    packages+=("ros-${ROS_DISTRO}-desktop")
  else
    packages+=("ros-${ROS_DISTRO}-ros-base")
  fi

  if [[ "${INSTALL_ROS_GZ}" == "1" ]]; then
    packages+=("ros-${ROS_DISTRO}-ros-gz")
  fi

  log "Installing packages: ${packages[*]}"
  "${SUDO[@]}" apt update
  "${SUDO[@]}" apt install -y "${packages[@]}"
}

verify_setup() {
  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    log "Missing /opt/ros/${ROS_DISTRO}/setup.bash after install"
    exit 1
  fi
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  command -v ros2 >/dev/null
  command -v colcon >/dev/null
  python3 - <<'PY'
import numpy
print("numpy", numpy.__version__)
PY
  log "ROS2 setup looks ready"
}

main() {
  require_ubuntu
  ensure_sudo
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    log "ROS2 ${ROS_DISTRO} already exists; checking support packages"
  else
    add_ros_apt_repo_if_needed
  fi
  install_packages
  verify_setup

  cat <<EOF

Next commands:
  source_setup /opt/ros/${ROS_DISTRO}/setup.bash
  scripts/check_ros2_server_preflight.sh
  scripts/verify_ros2_costmap_all_modes.sh astar

Optional PX4 bridge dependencies are intentionally not installed here.
Install/build px4_msgs and PX4-Autopilot separately before enabling PX4 Offboard.
EOF
}

main "$@"
