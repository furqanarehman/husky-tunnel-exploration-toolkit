# Husky Tunnel Autonomous Exploration — Self-Contained Project

An autonomous tunnel-inspection robot simulation: a Clearpath Husky A200 explores a
tunnel network using LIO-SAM (3D LiDAR-inertial mapping) and SLAM Toolbox + Nav2 (2D
navigation and autonomous exploration), built for ME 640 (Autonomous Mobile Robotics),
University of Waterloo.

**This repository is fully self-contained.** All source code — including the base
simulation project and its dependencies — is included directly here, with five bug
fixes already applied. No separate clone of any other repository is needed.

## Where this came from

The base simulation (`src/husky_tunnel_bringup`, plus the `lio_sam` and
`frontier_exploration_ros2` packages) originates from
[husky-lio-sam-tunnel-inspection](https://github.com/mohamadalquraan99-arch/husky-lio-sam-tunnel-inspection)
by Mohamad Alquraan (`feature/autonomous-exploration` branch) — see
[`BASE_PROJECT_README.md`](BASE_PROJECT_README.md) for his original documentation. This
repository adds five verified bug fixes (already applied directly to the source — see
`patches/` for the individual diffs and `docs/PROGRESS_LOG.md` for the full investigation
behind each one), five evaluation/debugging tools, and complete setup/troubleshooting
documentation.

## What's in here

```
src/                  Simulation source code (fixes already applied)
worlds/                Gazebo world files
clearpath/              Robot platform configuration
patches/               The 5 diffs, for reference/transparency (already applied to src/)
tools/                  5 standalone evaluation/debugging tools
scripts/                Setup script + the base project's dependency installer
docs/                  Progress log and troubleshooting guide
```

## Quick start

### 1. Prerequisites
- Ubuntu 22.04 (native or WSL2)
- ROS 2 Humble Desktop
- Git

### 2. Clone and set up — one command
```bash
git clone https://github.com/furqanarehman/husky-tunnel-exploration-toolkit.git
cd husky-tunnel-exploration-toolkit
chmod +x scripts/setup.sh
./scripts/setup.sh
```
This installs all dependencies via Clearpath's official apt repository and builds the
workspace. See [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) if anything fails —
it's very likely already documented with a fix.

### 3. Launch
```bash
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py
```
The `LIBGL_ALWAYS_SOFTWARE=1` line matters — see
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md#gazebo-gui-crash) if you skip it and
Gazebo's GUI crashes.

### 4. Use the evaluation tools
```bash
python3 tools/nav_success_rate.py your_run_log.txt
```
See [`tools/README.md`](tools/README.md) for all five tools and their usage.

## What the five fixes address

| Fix | Problem it solves |
|---|---|
| LIO-SAM crash guard | `dt <= 0` crash in IMU preintegration, killing the 3D mapping pipeline seconds after launch |
| Navigation fixes | Hardcoded path bug + robot getting stuck in tight tunnel corners for extended periods |
| Return-home fix | Return-to-home logic never triggering, even after full exploration |
| Mission duration cap | Guarantees a bounded mission time and a reliable return-home-and-stop, even on very large maps |
| Speed increase | Modest, conservative speed increase to reduce total run time |

Full technical detail on each — root cause, investigation, and verification — is in
[`docs/PROGRESS_LOG.md`](docs/PROGRESS_LOG.md). This is worth reading in full for the
mission-duration cap in particular: it took three separate design iterations to get
right, and the investigation behind each attempt is documented in detail.

## Evaluation results

Computed with the tools in `tools/`, against real exploration runs:

- **Navigation success rate:** 97.1% (33/34 goals succeeded)
- **Trajectory accuracy (ATE):** LIO-SAM 9.4cm, SLAM Toolbox 6.0cm (both sub-decimeter over a 100m+ environment)
- **Trajectory accuracy (RPE):** LIO-SAM 1.6cm/s, SLAM Toolbox 2.5cm/s
- **Map completeness:** see `docs/PROGRESS_LOG.md` §5.3 for the full breakdown, including a
  genuinely interesting finding about how free-space and wall-occupancy confidence build
  up at different rates during exploration

## Status / what's not done yet

- Anomaly detection tool (`tools/anomaly_detector.py`) built and tuned, but not yet fully
  validated against a complete map with known real targets
- Camera-based anomaly detection — deferred, not started
- These fixes have not yet been proposed back to the original upstream repository
- Physical hardware validation — out of scope, simulation only
- See `docs/PROGRESS_LOG.md` §8 for the complete list

## Author

Furqan Abdul Rehman — ME 640, University of Waterloo.
Base simulation project by Mohamad Alquraan (see `BASE_PROJECT_README.md`).
