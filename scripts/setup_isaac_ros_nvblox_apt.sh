#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ISAAC_ROS_RELEASE="${ISAAC_ROS_RELEASE:-release-4.4}"
UBUNTU_SUITE="${UBUNTU_SUITE:-noble}"

if [[ "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "This script is written for Isaac ROS + ROS2 Jazzy. Got ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  SUDO=sudo
else
  SUDO=
fi

echo "Configuring NVIDIA Isaac ROS apt repository (${ISAAC_ROS_RELEASE}, ${UBUNTU_SUITE})"
$SUDO apt-get update
$SUDO apt-get install -y curl gnupg software-properties-common
$SUDO add-apt-repository -y universe

echo "Configuring NVIDIA VPI repository for libnvvpi4/vpi4-dev"
$SUDO apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
$SUDO add-apt-repository -y "deb https://repo.download.nvidia.com/jetson/x86_64/${UBUNTU_SUITE} r39.2 main"

keyring="/usr/share/keyrings/nvidia-isaac-ros.gpg"
list_file="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
repo_line="deb [signed-by=${keyring}] https://isaac.download.nvidia.com/isaac-ros/${ISAAC_ROS_RELEASE} ${UBUNTU_SUITE} main"

curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | $SUDO gpg --batch --yes --dearmor -o "${keyring}"
$SUDO touch "${list_file}"
grep -qxF "${repo_line}" "${list_file}" || echo "${repo_line}" | $SUDO tee -a "${list_file}" >/dev/null

$SUDO apt-get update

echo "Checking NVBlox package candidate"
apt-cache policy "ros-${ROS_DISTRO}-isaac-ros-nvblox" || true
apt-cache policy libnvvpi4 vpi4-dev nvsci || true

packages=(
  libnvvpi4 \
  vpi4-dev \
  "ros-${ROS_DISTRO}-isaac-ros-nvblox" \
  "ros-${ROS_DISTRO}-rviz2"
)

if [[ "${INSTALL_NAV2:-0}" == "1" ]]; then
  packages+=("ros-${ROS_DISTRO}-nav2-bringup")
fi

echo "Installing NVBlox packages"
$SUDO apt-get install -y "${packages[@]}"

echo "NVBlox apt installation complete"
