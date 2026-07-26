# Husky Tunnel Autonomous Exploration — Fixes, Tools & Evaluation

This repository contains bug fixes, evaluation tooling, and documented results for the
[husky-lio-sam-tunnel-inspection](https://github.com/mohamadalquraan99-arch/husky-lio-sam-tunnel-inspection)
project's `feature/autonomous-exploration` branch — a Clearpath Husky A200 simulation that
autonomously explores a tunnel network using LIO-SAM (3D LiDAR-inertial mapping) and
SLAM Toolbox + Nav2 (2D navigation), built for ME 640 (Autonomous Mobile Robotics),
University of Waterloo.

**This is not a copy of the base project.** It's a companion repository: five bug-fix
patches (all found, fixed, and verified during testing), five evaluation/analysis tools
built from scratch, and full documentation of the debugging process — meant to be applied
on top of a normal clone of the base repository.

## What's in here

```
patches/      5 patches fixing real bugs found during testing (see PATCHES.md)
tools/        5 standalone ROS 2 tools for evaluation and debugging
scripts/      A robust, fixed setup script
docs/         Full progress log and troubleshooting guide
```

## Why this exists

Getting the base project running and producing reliable results surfaced five real bugs
(a crash, a stuck-navigation loop, a broken return-home condition, a race condition in
mission completion, and a hardcoded path) and a long list of environment/setup pitfalls.
This repo exists so nobody else has to rediscover any of it.

## Quick start

### 1. Prerequisites
- Ubuntu 22.04 (native or WSL2)
- ROS 2 Humble Desktop
- Git

### 2. Clone the base project
```bash
git clone --branch feature/autonomous-exploration --single-branch \
  --recurse-submodules \
  https://github.com/mohamadalquraan99-arch/husky-lio-sam-tunnel-inspection.git \
  husky_ws
cd husky_ws
```

### 3. Clone this repo alongside it
```bash
cd ..
git clone <this-repo-url> husky-exploration-toolkit
```

### 4. Run the fixed setup script
```bash
cd husky_ws
chmod +x ../husky-exploration-toolkit/scripts/setup.sh
../husky-exploration-toolkit/scripts/setup.sh
```
This installs dependencies via Clearpath's official apt repository, applies all 5 patches,
and builds the workspace. See [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) if
anything here fails — it's very likely already documented with a fix.

### 5. Launch
```bash
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py
```
The `LIBGL_ALWAYS_SOFTWARE=1` line matters — see
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md#gazebo-gui-crash) if you skip it and
Gazebo's GUI crashes.

### 6. Use the evaluation tools
```bash
python3 ../husky-exploration-toolkit/tools/nav_success_rate.py your_run_log.txt
```
See [`tools/README.md`](tools/README.md) for all five tools and their usage.

## What the patches fix

| Patch | Fixes |
|---|---|
| `lio_sam_imu_dt_guard.patch` | LIO-SAM crash (`dt <= 0` in IMU preintegration) |
| `husky_tunnel_bringup_nav_fixes.patch` | Hardcoded path bug + robot getting stuck in tight corners |
| `husky_tunnel_bringup_return_home_fix.patch` | Return-to-home never triggering |
| `husky_tunnel_bringup_mission_duration_cap.patch` | Guaranteed bounded mission time + reliable return-home-and-stop |
| `husky_tunnel_bringup_speed_increase.patch` | Faster exploration (modest, conservative speed increase) |

Full technical detail on each — root cause, investigation, and how it was verified — is in
[`docs/PROGRESS_LOG.md`](docs/PROGRESS_LOG.md).

## Evaluation results

Computed with the tools in this repo, against real exploration runs:

- **Navigation success rate:** 97.1% (33/34 goals succeeded)
- **Trajectory accuracy (ATE):** LIO-SAM 9.4cm, SLAM Toolbox 6.0cm (both sub-decimeter over a 100m+ environment)
- **Trajectory accuracy (RPE):** LIO-SAM 1.6cm/s, SLAM Toolbox 2.5cm/s
- **Map completeness:** see `docs/PROGRESS_LOG.md` §5.3 for the full breakdown and a genuinely interesting finding about how free-space vs. wall confidence build up at different rates during exploration

## Status / what's not done yet

- Anomaly detection tool built but not yet fully validated against a complete map (paused mid-tuning)
- These patches have not yet been merged into the upstream base repository
- See `docs/PROGRESS_LOG.md` §8 for the full list

## Author

Furqan Abdul Rehman — ME 640, University of Waterloo. Built on top of work by Mohamad Alquraan.
