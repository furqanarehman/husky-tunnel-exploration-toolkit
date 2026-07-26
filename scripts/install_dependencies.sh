#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_EXPECTED="humble"
UBUNTU_CODENAME_EXPECTED="jammy"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

log() {
  printf '\n[husky-setup] %s\n' "$1"
}

if [[ "$(lsb_release -sc 2>/dev/null || true)" != "$UBUNTU_CODENAME_EXPECTED" ]]; then
  echo "This project is tested on Ubuntu 22.04 (jammy)." >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash" ]]; then
  echo "ROS 2 Humble is not installed." >&2
  echo "Install ROS 2 Humble Desktop first, then rerun this script." >&2
  echo "Official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
  exit 1
fi

log "Installing system tools"
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  curl \
  git \
  gnupg \
  lsb-release \
  wget \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev

log "Adding the Clearpath package repository"
wget -qO- https://packages.clearpathrobotics.com/public.key | sudo apt-key add -
echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/clearpath-latest.list >/dev/null

if [[ ! -f /etc/ros/rosdep/sources.list.d/50-clearpath.list ]]; then
  sudo wget -q \
    https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
    -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
fi

log "Adding the Gazebo Fortress package repository"
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
wget -qO- http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

log "Installing ROS, Gazebo, Clearpath, mapping and navigation packages"
sudo apt-get update
sudo apt-get install -y \
  ignition-fortress \
  ros-humble-clearpath-simulator \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2

log "Initialising rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

log "Initialising Git submodules"
git -C "$WORKSPACE_DIR" submodule update --init --recursive

log "Applying simulator compatibility files"
LIO_PATCH="$WORKSPACE_DIR/src/husky_tunnel_bringup/patches/lio_sam_gazebo_imu_orientation.patch"
if [[ -f "$LIO_PATCH" ]]; then
  if git -C "$WORKSPACE_DIR/src/lio_sam" apply --check "$LIO_PATCH" >/dev/null 2>&1; then
    git -C "$WORKSPACE_DIR/src/lio_sam" apply "$LIO_PATCH"
    echo "Applied the LIO-SAM Gazebo IMU patch."
  elif git -C "$WORKSPACE_DIR/src/lio_sam" apply --reverse --check "$LIO_PATCH" >/dev/null 2>&1; then
    echo "LIO-SAM Gazebo IMU patch is already applied."
  else
    echo "Warning: the LIO-SAM patch could not be checked cleanly." >&2
  fi
fi

SENSOR_OVERRIDE_DIR="$WORKSPACE_DIR/src/husky_tunnel_bringup/patches/clearpath_sensor_overrides"
SENSOR_INSTALL_DIR="/opt/ros/humble/share/clearpath_sensors_description/urdf"
if compgen -G "$SENSOR_OVERRIDE_DIR/*.xacro" >/dev/null; then
  sudo cp "$SENSOR_OVERRIDE_DIR"/*.xacro "$SENSOR_INSTALL_DIR"/
fi

log "Installing workspace package dependencies"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
rosdep install \
  --from-paths "$WORKSPACE_DIR/src" \
  --ignore-src \
  --rosdistro humble \
  -r \
  -y

log "Building the workspace"
cd "$WORKSPACE_DIR"
colcon build --symlink-install

cat <<EOF

Setup completed.

Launch the project with:
  source /opt/ros/humble/setup.bash
  source "$WORKSPACE_DIR/install/setup.bash"
  ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py
EOF
