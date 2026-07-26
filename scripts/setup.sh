#!/usr/bin/env bash
#
# setup.sh
#
# One-command setup for this self-contained repo. The simulation source
# code already has all 5 fixes applied directly (see patches/ for the
# individual diffs, and docs/PROGRESS_LOG.md for the full story behind
# each one) -- this script just installs dependencies and builds.
#
#   git clone https://github.com/furqanarehman/husky-tunnel-exploration-toolkit.git
#   cd husky-tunnel-exploration-toolkit
#   ./scripts/setup.sh
#
# What this does:
#   1. Fixes a known strict-mode bug in scripts/install_dependencies.sh
#      (set -euo pipefail conflicting with ROS 2's own setup.bash)
#   2. Runs the (now fixed) install script -- installs Gazebo Fortress,
#      the Clearpath simulator stack, and all ROS 2 dependencies via apt
#   3. Builds the workspace
#
# Safe to re-run.

set -eo pipefail  # deliberately NOT -u, matching the fix in TROUBLESHOOTING.md

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Husky Tunnel Exploration Toolkit — Setup ==="
echo ""
echo "Repo directory: $REPO_DIR"
echo ""

cd "$REPO_DIR"

if [ ! -f "src/husky_tunnel_bringup/package.xml" ]; then
    echo "ERROR: src/husky_tunnel_bringup/package.xml not found."
    echo "  This doesn't look like a complete clone of this repo."
    echo "  Try re-cloning, or check docs/TROUBLESHOOTING.md."
    exit 1
fi

# --- 1. Fix and run install_dependencies.sh -------------------------
if [ -f "scripts/install_dependencies.sh" ]; then
    echo "--- Fixing known strict-mode bug in install_dependencies.sh ---"
    if grep -q "set -euo pipefail" scripts/install_dependencies.sh; then
        sed -i 's/set -euo pipefail/set -eo pipefail/' scripts/install_dependencies.sh
        echo "Patched: set -euo pipefail -> set -eo pipefail"
    else
        echo "Already patched or uses different strict-mode flags -- skipping."
    fi

    chmod +x scripts/install_dependencies.sh

    echo ""
    echo "--- Running install_dependencies.sh ---"
    echo "(This installs Gazebo Fortress, the Clearpath simulator stack, and"
    echo " all ROS 2 dependencies via apt. It will prompt for your sudo"
    echo " password and may take several minutes.)"
    echo ""
    ./scripts/install_dependencies.sh
else
    echo "WARNING: scripts/install_dependencies.sh not found."
    echo "  You'll need to install Gazebo Fortress, the Clearpath simulator,"
    echo "  and ROS 2 dependencies manually. See docs/TROUBLESHOOTING.md."
fi

# --- 2. Build ---------------------------------------------------------
echo ""
echo "--- Building workspace ---"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select husky_tunnel_bringup

echo ""
echo "=== Setup complete ==="
echo ""
echo "To launch:"
echo "  export LIBGL_ALWAYS_SOFTWARE=1"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $REPO_DIR/install/setup.bash"
echo "  ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py"
echo ""
echo "If anything went wrong, check docs/TROUBLESHOOTING.md."
