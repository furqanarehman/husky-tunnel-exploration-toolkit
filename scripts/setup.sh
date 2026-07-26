#!/usr/bin/env bash
#
# setup.sh
#
# Robust setup for husky_ws (the feature/autonomous-exploration base project),
# applying every fix documented in docs/TROUBLESHOOTING.md and
# docs/PROGRESS_LOG.md automatically.
#
# Run this FROM the husky_ws directory (the base project you cloned), e.g.:
#
#   cd husky_ws
#   ../husky-exploration-toolkit/scripts/setup.sh
#
# What this does:
#   1. Sanity-checks you're running it from the right place
#   2. Fixes the base project's install_dependencies.sh strict-mode bug
#      (set -euo pipefail conflicting with ROS 2's own setup.bash)
#   3. Runs the (now fixed) install script
#   4. Applies all 5 patches from this repo's patches/ folder
#   5. Builds the workspace
#
# Safe to re-run -- patches are checked before applying, and the build step
# is idempotent.

set -eo pipefail  # deliberately NOT -u, matching the fix in TROUBLESHOOTING.md

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLKIT_DIR="$(dirname "$SCRIPT_DIR")"
PATCHES_DIR="$TOOLKIT_DIR/patches"

echo "=== Husky Tunnel Exploration Toolkit — Setup ==="
echo ""

# --- 1. Sanity check ---------------------------------------------------
if [ ! -f "src/husky_tunnel_bringup/package.xml" ]; then
    echo "ERROR: This doesn't look like the husky_ws base project directory."
    echo "  Expected to find: src/husky_tunnel_bringup/package.xml"
    echo "  Run this script FROM your husky_ws directory, e.g.:"
    echo "    cd husky_ws && ${BASH_SOURCE[0]}"
    exit 1
fi

if [ ! -d "$PATCHES_DIR" ]; then
    echo "ERROR: Could not find patches/ directory at $PATCHES_DIR"
    echo "  Make sure this toolkit repo is cloned alongside husky_ws, and"
    echo "  that you're running this script from its scripts/ subfolder."
    exit 1
fi

echo "Base project directory: $(pwd)"
echo "Toolkit directory:      $TOOLKIT_DIR"
echo ""

# --- 2. Fix and run the base project's install script -------------------
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
    echo "WARNING: scripts/install_dependencies.sh not found in the base"
    echo "project. Skipping automated dependency install -- you'll need to"
    echo "install Gazebo Fortress, the Clearpath simulator, and ROS 2"
    echo "dependencies manually. See docs/TROUBLESHOOTING.md for guidance."
fi

# --- 3. Apply patches ----------------------------------------------------
echo ""
echo "--- Applying patches from $PATCHES_DIR ---"

PATCH_ORDER=(
    "lio_sam_imu_dt_guard.patch"
    "husky_tunnel_bringup_nav_fixes.patch"
    "husky_tunnel_bringup_return_home_fix.patch"
    "husky_tunnel_bringup_mission_duration_cap.patch"
    "husky_tunnel_bringup_speed_increase.patch"
)

for patch_name in "${PATCH_ORDER[@]}"; do
    patch_path="$PATCHES_DIR/$patch_name"
    if [ ! -f "$patch_path" ]; then
        echo "  SKIP: $patch_name (not found in patches/)"
        continue
    fi

    if git apply --check "$patch_path" 2>/dev/null; then
        git apply "$patch_path"
        echo "  APPLIED: $patch_name"
    elif git apply --reverse --check "$patch_path" 2>/dev/null; then
        echo "  ALREADY APPLIED: $patch_name (skipping)"
    else
        echo "  WARNING: $patch_name doesn't apply cleanly (neither forward"
        echo "           nor already-applied). It may conflict with local"
        echo "           changes, or the base project may have changed since"
        echo "           this patch was written. Apply manually if needed:"
        echo "             git apply --check $patch_path"
    fi
done

# --- 4. Build --------------------------------------------------------
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
echo "  source install/setup.bash"
echo "  ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py"
echo ""
echo "If anything went wrong, check docs/TROUBLESHOOTING.md in this toolkit repo."
