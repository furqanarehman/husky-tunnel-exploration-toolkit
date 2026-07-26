# Evaluation & Debugging Tools

Five standalone ROS 2 / Python tools built during testing of the
`feature/autonomous-exploration` branch. None require modifying the base project — each
works against topics, log files, or saved maps produced by a normal run.

## `anomaly_detector.py`
Detects potential anomalies (obstacles, unexpected objects) using connected-component
clustering on the SLAM Toolbox occupancy grid. Tunnel walls form long continuous blobs;
anything else compact and isolated is a candidate.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 anomaly_detector.py --ros-args -p use_sim_time:=true
```
Publishes confirmed detections as RViz markers on `~/anomaly_markers` and a JSON list on
`~/anomalies`. Tunable via ROS parameters — see the file's docstring.

**Status:** built and tuned against false positives (see `docs/PROGRESS_LOG.md` §4), but
not yet fully validated against a complete map with known real targets.

## `nav_success_rate.py`
Parses an exploration run's log file (no live run needed) and reports navigation goal
success/failure rate, broken down by goal type, plus time-to-goal statistics.

```bash
python3 nav_success_rate.py path/to/run_log.txt --csv output.csv
```

## `pose_recorder.py`
Records three trajectories live during a run: Gazebo ground truth, LIO-SAM's estimated
odometry, and SLAM Toolbox's estimated pose. Requires a one-time bridge command first —
see the file's docstring for the exact command and topic names to verify.

```bash
python3 pose_recorder.py --ros-args \
  -p use_sim_time:=true \
  -p output_dir:=./pose_logs \
  -r /tf:=/a200_0000/tf \
  -r /tf_static:=/a200_0000/tf_static
```

## `ate_rpe.py`
Computes Absolute Trajectory Error and Relative Pose Error from the CSVs produced by
`pose_recorder.py`, using Umeyama alignment (same method as the standard TUM RGB-D
benchmark tools).

```bash
python3 ate_rpe.py ground_truth.csv lio_sam.csv
python3 ate_rpe.py ground_truth.csv slam_toolbox.csv
```

## `map_completeness.py`
Compares two saved maps (standard ROS `map_saver_cli` `.pgm`/`.yaml` format) and reports
how complete one is relative to the other — free-space completeness and wall/occupied
completeness (both strict pixel-exact and a tolerant version that accounts for
session-to-session SLAM registration noise).

```bash
# Save maps first, e.g.:
ros2 run nav2_map_server map_saver_cli -f reference_map --ros-args -p use_sim_time:=true

python3 map_completeness.py reference_map.yaml comparison_map.yaml
```

See `docs/PROGRESS_LOG.md` §5.3 for an important finding about interpreting the wall
completeness numbers.

## Dependencies
All tools use only `numpy`, `pyyaml`, and standard ROS 2 Python packages (`rclpy`,
`nav_msgs`, `tf2_ros`, etc.) — nothing beyond what a normal ROS 2 Humble install already
has, aside from:
```bash
pip install numpy pyyaml
```
