# Husky Tunnel Inspection Project — Progress Log

**Branch:** `feature/autonomous-exploration`
**Session dates:** July 24–26, 2026
**Environment:** WSL2, Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, Clearpath Husky A200 simulation stack

This document covers everything accomplished across a long debugging and development session: getting the project running reliably, finding and fixing five real bugs, building an anomaly detector, and computing three evaluation metrics with real data.

---

## 1. Environment & Infrastructure

Before any real project work could happen, a long chain of environment issues had to be resolved. These aren't project bugs — they were local machine/setup problems, but they consumed significant time and are worth recording so they don't get re-debugged from scratch later.

### 1.1 Environment pollution (root cause of much early chaos)
- `.bashrc` was auto-sourcing the user's *other* project (`~/ros2_ws`) on every new terminal, including terminals meant for this Husky project.
- A separately, manually-built GTSAM 4.2.0 lived in `/usr/local/lib`, and `.bashrc` unconditionally prepended it to `LD_LIBRARY_PATH` on every shell.
- Combined, these caused `lio_sam` to silently link against the wrong GTSAM at build time, producing ABI-mismatch crashes (`undefined symbol` errors) at runtime that looked unrelated to the actual cause.
- **Fix:** commented out the auto-sourcing in `.bashrc`, wrapped it in an on-demand `use_ros2ws` alias instead. Verified via `env | grep -E 'AMENT_PREFIX_PATH|LD_LIBRARY_PATH|...'` coming back empty in a genuinely fresh terminal.
- **Extra wrinkle:** the terminal application in use was found to cache environment variables across "new" tabs within the same window — a true fresh environment required opening a brand-new terminal via `wsl.exe` from PowerShell, or a full `wsl --shutdown`.

### 1.2 Gazebo GUI crash (`Ogre::UnimplementedException`)
- Gazebo's GUI reliably crashed shortly after startup with a `GL3PlusTextureGpu::copyTo` exception, tied to Thermal-sensor rendering assets inherited from the world's SubT-competition origins.
- **Fix:** `export LIBGL_ALWAYS_SOFTWARE=1` before every launch, forcing Mesa software rendering. Confirmed this must be set explicitly — Mohamad's own working machine uses the same software renderer but for whatever reason didn't need the variable set explicitly; this machine did.

### 1.3 Robot spawn stall (`create-21` node stuck)
- The robot spawner (`create-21`) would hang forever waiting on `robot_description`, even though `robot_state_publisher` had successfully parsed the URDF.
- Root cause: this was **downstream of the environment pollution** in §1.1, not a DDS/FastRTPS issue as initially suspected. Once the polluted environment was fixed, this resolved on its own. (A `Cyclone DDS` switch was investigated as a possible fix for the many `RTPS_TRANSPORT_SHM Failed init_port` errors seen throughout logs, but was not ultimately needed once the real root cause was found.)

### 1.4 Hardcoded path bug (genuine repo bug, not local)
- `exploration_nav2.yaml` had a hardcoded absolute path to Mohamad's own machine: `/home/momo1/husky_ws/.../tunnel_no_spin_wait.xml`.
- **Fix (included in the nav_fixes patch, §2.2):** moved the BT XML path into `tunnel_navigation.launch.py`'s existing `RewrittenYaml`/`param_substitutions` mechanism, computed at launch time via `get_package_share_directory()`. This works on any machine/username, not just a local hand-edit.

### 1.5 Dependency/build issues resolved along the way
- `clearpath_config`'s `humble` branch was missing a module (`common.types.exception`) that `clearpath_generator_gz` required — fixed by switching to the `jazzy` branch of `clearpath_config` (a pure-Python package, not tied to ROS distro despite the branch name).
- `control_msgs` needed `BatteryStateArray`, which only exists on its newest branch, not the Humble apt release — built from source and overlaid.
- Eventually, Mohamad's own `scripts/install_dependencies.sh` (added mid-session) resolved most of this automatically via Clearpath's official apt repository, which was a much cleaner path than manual source builds.
- The install script has a known bug: `set -euo pipefail` conflicts with a ROS 2 `setup.bash` internal (`AMENT_TRACE_SETUP_FILES`/`AMENT_PYTHON_EXECUTABLE` unbound variables). Workaround: `sed -i 's/set -euo pipefail/set -eo pipefail/' scripts/install_dependencies.sh`.

### 1.6 A dead-end explored and abandoned: custom "mini" test world
- To speed up iteration (full exploration runs took up to an hour), a smaller custom closed-loop test world (`tunnel_mini.sdf`) was built, with matching duplicate launch files.
- Went through two geometry iterations (v1 had a wall-topology bug sealing every corridor leg into a dead end; v2 fixed this, verified via a flood-fill connectivity check).
- Even after fixing geometry, **no sensor data was ever produced in Gazebo for the mini world** (confirmed via `ign topic -l` showing zero LiDAR/IMU topics). Multiple hypotheses were tested and ruled out: shared `setup_path` caching, world-name threading through Clearpath's generator (found and fixed a real path-concatenation bug there, but it wasn't the actual cause of the sensor issue).
- **Decision: abandoned.** Not worth further time; reverted to testing directly on the real tunnel world, which was already confirmed reliable. All mini-world files were later deleted from the workspace. This remains a real, unexplained limitation if anyone wants to revisit building a fast test world later.

---

## 2. Bugs Found and Fixed (5 patches, all confirmed working)

All patch files are saved at `~/husky_ws/*.patch`. **None of these have been merged into Mohamad's actual repo yet** — they exist only as local patches, applied to this machine's working copy.

### 2.1 `lio_sam_imu_dt_guard.patch`
- **Bug:** `lio_sam_imuPreintegration` crashed almost immediately after every launch with `PreintegratedImuMeasurements::integrateMeasurement: dt <=0`. GTSAM refuses to integrate an IMU measurement with a zero/negative time delta, which can happen if two consecutive IMU messages arrive with identical or out-of-order timestamps — most likely right at simulation startup before the clock/sensor stream stabilizes.
- **Fix:** three call sites in `imuPreintegration.cpp` (all `dt` computations feeding `integrateMeasurement`) now clamp `dt` to a safe fallback (`1/500s`, matching the existing first-sample fallback) instead of passing a bad value through to GTSAM.
- **Confirmed:** stable across multiple long runs afterward (including a full ~50-minute exploration with zero crashes), versus reliably crashing within ~1 second before the fix.

### 2.2 `husky_tunnel_bringup_nav_fixes.patch`
Bundles three related fixes in one patch, across `exploration_nav2.yaml`, `tunnel_no_spin_wait.xml`, and `tunnel_navigation.launch.py`:
- **Hardcoded path fix** — see §1.4.
- **Stuck-navigation fix** — the robot would repeatedly fail `Failed to make progress` in tight tunnel corners for extended periods (one test run: 33 minutes of continuous failure loop). Root cause: `inflation_radius: 1.00`m combined with the Husky's ~0.76m-wide footprint left almost no usable free space in tight sections, and the existing behavior tree (`tunnel_no_spin_wait.xml` — deliberately excludes `Spin` recovery, presumably to avoid wall contact in narrow tunnels) only allowed 1 retry before giving up entirely.
  - `inflation_radius: 1.00 → 0.60`
  - `movement_time_allowance: 15.0 → 25.0` (more patience before declaring "stuck")
  - Recovery `number_of_retries: 1 → 3`, added `DriveOnHeading` as a second recovery option alongside `BackUp` (still no `Spin`, respecting the original design intent)
- **Confirmed:** a subsequent ~12-minute exploration run showed **zero** `Failed to make progress` errors, versus 33 minutes of continuous failure before.

### 2.3 & 2.4 — The Return-Home Saga (full detail)

Getting the robot to reliably explore and then actually come home and stop turned out to be the single most involved piece of debugging in the whole session — it took two separate patches and, within the second patch, three full iterations to get right. This section walks through the whole investigation in order, since the intermediate failures are as instructive as the final fix.

#### Symptom 1: the robot never came home at all

After a long exploration run (~51 minutes) that had clearly covered a large, complex, multi-branch tunnel network, the robot was observed to keep exploring indefinitely — visually appearing to head back toward the start at times, then peeling off into the tunnel again instead of stopping. Searching the full run log for the code's own completion messages (`"exploration is complete, returning home"`, `"Returning to recorded home pose"`, `"Home pose reached"`) turned up **zero matches** across the entire ~34,500-line log. The mission-complete/return-home code path had simply never fired, even once.

#### Root cause: `visited_timeout_s` fighting the completion counter

Reading `tunnel_backtracking_explorer.cpp`'s candidate-selection logic directly (rather than guessing) showed the actual completion condition: a counter (`no_frontier_cycles_`) only increments when `select_candidate()` finds **zero** valid candidates on a given tick, and `send_home_goal()` only fires once that counter reaches a threshold (`completion_confirmations_`, default 20 consecutive empty checks). Critically, a parameter called `visited_timeout_s` (default `600.0`, i.e. 10 minutes) causes the coverage-fallback logic to "forget" that an area was already covered once that much time has passed — meaning on any run longer than ~10 minutes, patches of ground near wherever the robot has spent the most time (typically near home, since that's the start/end of many sub-loops) become eligible again as "new" targets, resetting `no_frontier_cycles_` back to zero every time. The counter could never reach 20 in a row, so completion — and therefore the return-home call — never triggered.

**Fix (`husky_tunnel_bringup_return_home_fix.patch`):**
```yaml
# tunnel_backtracking.yaml
visited_timeout_s: 600.0   →   86400.0   # effectively disabled for a single mission
min_frontier_cells: 3      →   6         # less sensitive to sensor noise right at the robot's own body when parked at home
```

This fixed the *logical* bug, but a separate, practical problem remained: on this particular world (a large multi-branch network spanning roughly 130m × 100m), pure coverage-based completion — waiting for genuinely zero unexplored patches anywhere — could still take an extremely long time in practice, since new small legitimate patches keep turning up across such a large area. A run left going for ~51 minutes still hadn't reached genuine completion even with the timeout fix in place. This motivated a second, complementary fix: a hard cap on total mission time, so a run is *guaranteed* to end and return home within a bounded window, trading full-map completeness for reliability.

#### Building the mission-duration cap — three iterations

**Design (v1):** added a new parameter `max_mission_duration_s` (default 1800s / 30 minutes) to `tunnel_backtracking_explorer.cpp`. The idea: track when the mission started, and once `max_mission_duration_s` has elapsed, call `send_home_goal()` directly regardless of `no_frontier_cycles_`, guarded by `!returning_home_` so it wouldn't call it again once already heading home.

**v1 result — rapid-fire retry storm.** Tested with a shortened 300-second cap for fast iteration. The cap *did* fire, and the mission *did* eventually finish (`"Home pose reached; autonomous mission finished"` did appear) — but only after **retrying every ~1 second for about 7 minutes** first. Root cause: `send_home_goal()`'s own result callback unconditionally resets `returning_home_ = false` on any non-success result — including a goal that was merely canceled/preempted, not genuinely failed. Since the guard was just `!returning_home_`, the moment that flag flipped back to false (which could happen almost immediately if the goal was rejected/preempted), the duration check fired again on the very next tick, sending another home goal, which itself could get preempted, resetting the flag again — a rapid self-sustaining loop.

**Design (v2):** replaced the simple boolean guard with a 20-second cooldown between attempts (`last_duration_cap_attempt_` timestamp + `duration_cap_retry_cooldown_s_`), so even if `returning_home_` reset to false, the code wouldn't try again until the cooldown elapsed.

**v2 result — retries spaced out correctly, but still never succeeded.** Confirmed via logs that retries were now genuinely ~20-29 seconds apart instead of ~1 second — the cooldown mechanism itself worked as designed. But a full test run showed the cap retrying for **~25 minutes straight** and still never reaching `"Home pose reached"`. Digging into the log around individual failed attempts revealed the real problem: **within the same second**, the code was sending the robot home *and* sending it to a completely different frontier goal far away (e.g., `(88.77, -59.13)`) — the two goals were directly fighting each other, each preempting the other (`result code 6` = ABORTED, seen on both sides of the conflict in the log). During the ~20-second cooldown window between duration-cap attempts, the *normal* frontier/coverage exploration logic was still running freely on every tick (since it's structurally located later in `tick()`, and nothing was stopping the code from reaching it once `returning_home_` was false again) — so it would find a legitimate-looking frontier and dispatch the robot there, directly canceling whatever home-goal was still in flight.

**Design (v3, final):** the real fix wasn't about retry timing at all — it was that the normal exploration code path needed to be **permanently, structurally unreachable** once the deadline was hit, not just conditionally skipped. Added a new flag, `mission_deadline_exceeded_`, set exactly once (a true one-way switch, never reset) the moment elapsed time first crosses `max_mission_duration_s`. Restructured `tick()` so this flag is checked immediately after the existing in-flight-goal stall-detection block (which is left untouched, so a stalled home-goal attempt still benefits from the existing cancel/retry logic) but **before** the code can ever reach `select_candidate()` / `send_goal()` for ordinary exploration:

```cpp
if (navigating_) {
  // ...existing stall-detection logic, unchanged...
  return;
}

if (mission_deadline_exceeded_) {
  // Never fall through to normal frontier/coverage exploration once the
  // deadline has been hit -- only retry getting home, on a cooldown.
  const double since_last_attempt_s =
    (get_clock()->now() - last_duration_cap_attempt_).seconds();
  if (since_last_attempt_s >= duration_cap_retry_cooldown_s_) {
    last_duration_cap_attempt_ = get_clock()->now();
    send_home_goal();
  }
  return;
}

// ...only reachable if mission_deadline_exceeded_ is still false...
nav_msgs::msg::OccupancyGrid::SharedPtr map;
// ...normal candidate selection continues here...
```

This guarantees that once the deadline trips, there is no code path left in `tick()` that can ever call `send_goal()` for a normal exploration target again — `returning_home_` can still churn through whatever transient states it wants internally, but it no longer gates whether competing goals get sent, because the competing-goal code is now entirely unreachable.

**v3 result — confirmed clean.** A full test with the real 1800-second cap showed exactly **one** `"Max mission duration of 1800 s reached"` warning, followed ~94 seconds later by exactly one `"Returning to recorded home pose"` (that gap being Nav2 normally canceling the in-progress far-away goal and replanning — expected, not a bug), then a genuinely uninterrupted ~9.5-minute drive with no competing goals at all, ending in `"Home pose reached; autonomous mission finished"` and the robot visibly stopping. `Goal succeeded` from Nav2 itself confirmed the clean result. This is the version shipped in `husky_tunnel_bringup_mission_duration_cap.patch`.

**Summary of both patches together:**
- `husky_tunnel_bringup_return_home_fix.patch` — fixes the *logical* reason completion was undetectable (`visited_timeout_s`)
- `husky_tunnel_bringup_mission_duration_cap.patch` — adds a *guaranteed* backstop so a run reliably ends within a bounded time regardless of how the coverage logic behaves on very large maps, with the permanent one-way `mission_deadline_exceeded_` switch being the key insight that took three attempts to land on
- **Confirmed:** a full-length test showed exactly **one** "Max mission duration reached" warning, one "Returning to recorded home pose," a clean uninterrupted ~9.5-minute drive home, and "Home pose reached; autonomous mission finished."

### 2.5 `husky_tunnel_bringup_speed_increase.patch`
- Modest increase to Nav2's DWB local planner speed limits, to reduce total time-per-run:
  - `max_vel_x: 0.60 → 0.90`
  - `max_vel_theta: 0.80 → 1.00`
  - `max_speed_xy: 0.60 → 0.90`
  - `acc_lim_x: 1.00 → 1.30`
- Deliberately conservative (not doubled) to avoid introducing motion-blur/scan-matching issues in LIO-SAM or IMU preintegration difficulty from overly aggressive rotation.

---

## 3. The Real Tunnel World

The actual `worlds/tunnel.sdf` (current version, confirmed via direct inspection — differs from an earlier, simpler hand-authored version seen at the very start of the session) is built almost entirely from Gazebo Fuel-hosted models: a `subt_tunnel_staging_area`, many `Jersey Barrier` segments forming the corridor walls, and several `Tunnel Tile` pieces extending the network across a large area (roughly x: -14 to 118, y: -60 to 42).

### Known real "artifacts" placed in the world (ground truth for anomaly detection):

| Object | World position (x, y, z) |
|---|---|
| `drill_1` (cordless drill) | (34, 2, 0.004) |
| `rescue_randy_1` (survivor mannequin) | (54, 42.2, 0.004) |
| `extinguisher_1` (fire extinguisher) | (42, -25, 0.004) |
| `phone_1` (cell phone) | (117.8, -60, -4.996) — note negative z, possibly in a pit/lower area |

(`artifact_origin`, a fiducial marker, is a reference point, not a detection target.)

---

## 4. Anomaly Detection

Built from scratch: `~/husky_ws/tools/anomaly_detector.py`, a standalone ROS 2 node (no repo integration yet — deliberately kept separate for fast iteration).

### Approach
Connected-component clustering on the `/map` occupancy grid published by SLAM Toolbox. Tunnel walls form long, thin, continuous blobs; anything else occupied — compact, isolated, not part of the wall boundary — is a candidate anomaly. A candidate must persist across several consecutive map updates before being confirmed (filters transient SLAM noise).

### Tuning journey (three false-positive root causes found and fixed)
1. **Diagonal wall fragmentation** — strict 4-connectivity let diagonally-touching wall-edge pixels count as separate tiny "objects." Fixed by switching to 8-connectivity, then further to a configurable gap-bridging radius (default 2 cells) to merge sparsely-mapped wall sections.
2. **"Ghost wall" offset artifacts** — a consistent band of false positives ~0.4–0.7m parallel to real walls, initially theorized as SLAM registration drift. Added a resolution-independent wall-proximity buffer (`wall_buffer_m`, default 0.7m) as a backup filter.
3. **Sparse-mapping fragmentation in less-explored regions** — walls farther from the spawn point (fewer repeated passes) fragment into many small pieces that individually fail the wall-size/elongation thresholds, so the proximity buffer has no qualifying wall blob to compare against. This is the same underlying phenomenon later confirmed and quantified in the map completeness work (§5.3) — walls need many repeated observations to reach high confidence, not just one pass.

### Status: paused, not fully validated
Testing was paused mid-session to wait for a more mature, complete map (the false-positive tuning was chasing a moving target on a map that was still actively forming). **Not yet re-run against a genuinely complete map** — this is the natural next step now that the mission-duration cap (§2.4) reliably produces complete runs.

---

## 5. Evaluation Metrics (all three completed with real data)

Three new reusable tools built and verified (via synthetic test cases with known correct answers) before trusting them against real project data. All live in `~/husky_ws/tools/`.

### 5.1 Navigation success rate — `nav_success_rate.py`
Parses an exploration run's log file (no live run needed, works retroactively) and computes goal success/failure statistics by type (frontier / coverage-fallback / return-home).

**Result** (from `run_20260725_164319.txt`, the confirmed mission-duration-cap success run):
```
Total goals attempted: 34
Succeeded: 33  Failed: 1  Success rate: 97.1%
  frontier:      33 attempted, 32 succeeded (97.0%)
  return_home:    1 attempted,  1 succeeded (100.0%)
Time-to-goal: mean=90.2s  median=41.7s  max=654.3s  min=5.1s
Failure breakdown: 1x result code 6 (ABORTED)
```

### 5.2 ATE/RPE (trajectory accuracy) — `pose_recorder.py` + `ate_rpe.py`
`pose_recorder.py` records three trajectories live during a run: Gazebo ground truth (via a one-time `ros_gz_bridge` bridge of `/world/tunnel/dynamic_pose/info`), LIO-SAM's estimated odometry, and SLAM Toolbox's estimated pose (via TF lookup). `ate_rpe.py` computes ATE (absolute trajectory error, via Umeyama alignment) and RPE (relative pose error over a fixed time delta) offline from the recorded CSVs.

**Two real bugs found and fixed during setup:**
- QoS mismatch: LIO-SAM's odometry publisher uses `BEST_EFFORT` reliability; the default subscriber requested `RELIABLE`, silently receiving nothing. Fixed by matching QoS.
- TF namespace: the project remaps `/tf` and `/tf_static` to `/a200_0000/tf` and `/a200_0000/tf_static` throughout; the recorder's TF listener needed the same remap passed via `-r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static`.
- Also found and fixed: the bridged ground-truth messages carried a broken zero (`0.0`) timestamp in their header, causing zero timestamp-association matches with the other trajectories. Fixed by using the recording node's own clock (respecting `use_sim_time`) at message-arrival time instead of trusting the message's embedded stamp.
- A genuine math bug was also found and fixed in `ate_rpe.py` itself before trusting it: the initial RPE implementation compared raw relative-motion vectors without correcting for a fixed rotational offset between the two trajectories' frames, which (verified via synthetic test) inflated RPE by ~40x purely from frame misalignment, not real error. Fixed by applying the same Umeyama alignment rotation used for ATE.

**Result** (from `pose_logs_20260725_182943/`, ~13,000–34,000 associated sample pairs):

| Metric | LIO-SAM | SLAM Toolbox |
|---|---|---|
| ATE (RMSE) | 0.094 m | 0.060 m |
| ATE mean / median | 0.084 / 0.073 m | 0.049 / 0.045 m |
| ATE max | 0.242 m | 0.310 m |
| RPE (RMSE, 1.0s) | 0.016 m | 0.025 m |

Both estimates are accurate to well under 10cm globally in a ~100m+ environment. LIO-SAM shows better local consistency (lower RPE); SLAM Toolbox shows slightly better global alignment (lower ATE) but a higher worst-case local spike, plausibly from occasional loop-closure corrections.

### 5.3 Map completeness — `map_completeness.py`
Since the real world is built mostly from external Gazebo Fuel meshes (not parseable offline without internet access to Fuel servers), true CAD-level ground truth wasn't available. Instead, compares a "comparison" map against a "reference" map (your most thorough run) — a standard practical substitute used in real-world SLAM evaluation when perfect ground truth doesn't exist. Both maps saved via `nav2_map_server`'s `map_saver_cli`.

**Two completeness metrics for occupied/wall cells** were computed: strict (exact-pixel overlap) and tolerant (a configurable-radius buffer, default 0.3m, to absorb session-to-session registration noise between two *independent* SLAM sessions — verified via synthetic test that a mere 2-pixel wall shift alone drops strict completeness to 2.5% while tolerant correctly recovers to 100%).

**Result** (reference = long/thorough run, comparison = ~5-minute short run):
```
Overall known-cell completeness:  58.8%
Free-space completeness:          58.7%
Occupied/wall completeness (strict):    5.2%
Occupied/wall completeness (tolerant, ±0.3m): 12.4%
```

**Follow-up investigation** (the tolerant number still being far below free-space completeness prompted deeper analysis): of the reference map's wall cells that *do* fall within the area the short run explored (66.6% of all reference walls), the vast majority (92%) were marked as **free**, not occupied, in the short run's map — only ~8% had crossed SLAM Toolbox's occupied-probability threshold.

**Key finding:** free space is confirmed from a single LiDAR pass (any ray traversal marks cells free), but walls require multiple repeated close observations to build enough confidence to cross the occupied threshold. A short/incomplete run "knows roughly where it's been" well before it "knows exactly where the walls are." This directly matters for anomaly detection, since the detector depends on confident occupied-cell classification — a short run risks missing real obstacles not because it never went near them, but because they hadn't been observed enough times yet to register as solid.

---

## 6. Tools Built This Session

All in `~/husky_ws/tools/` (kept separate from the tracked repo — not yet proposed for inclusion, but worth discussing with Mohamad since they're reusable):

| Tool | Purpose |
|---|---|
| `anomaly_detector.py` | Occupancy-grid connected-component anomaly detection (paused, needs re-validation) |
| `nav_success_rate.py` | Parses run logs for navigation goal success/failure statistics |
| `pose_recorder.py` | Records ground-truth + LIO-SAM + SLAM Toolbox trajectories live |
| `ate_rpe.py` | Computes ATE/RPE from recorded trajectory CSVs |
| `map_completeness.py` | Compares two saved maps for free-space and wall completeness |

---

## 7. File Organization (as of end of session)

```
~/husky_ws/
    *.patch                          5 patch files (see §2), not yet merged upstream
    tools/                           5 reusable scripts (see §6)
    results/
        maps/                        reference_map + comparison_map (.pgm/.yaml)
        nav_stats.csv                nav success rate output
        pose_logs_20260725_182943/   ATE/RPE dataset (ground_truth.csv, lio_sam.csv, slam_toolbox.csv)
        run_20260725_164319.txt      confirmed mission-duration-cap success run
        run_20260725_185118.txt      reference-map source run
        run_20260725_195414.txt      comparison-map source run
        old_debug_logs/              archived superseded debugging logs
```

Cleaned up during this session: removed empty/superseded debug run folders, stray `Zone.Identifier` Windows metadata files, and an old cached copy of `tunnel.sdf`.

---

## 8. Status Summary — What's Done, What's Left

### Done and confirmed working
- Reliable environment setup (documented root causes, not just workarounds)
- 5 real bugs found, fixed, and confirmed via repeated test runs
- Bounded, reliable "explore then return home and stop" mission behavior
- 3 evaluation metrics computed with real data and verified tooling
- Clean, organized project state

### Not yet done
1. **Send the 5 patches to Mohamad** — currently only exist locally; the actual upstream repo doesn't have any of tonight's fixes yet.
2. **Re-validate the anomaly detector** against a genuinely complete map now that reliable full runs are possible — this is the natural next step, paused mid-tuning.
3. **Anomaly detection precision/recall** — blocked on #2; can't measure until the detector is confirmed working against real targets.
4. Camera-based anomaly detection (Option B from the original plan) — explicitly deferred, "later if time permits."
5. Consider proposing the `tools/` scripts for inclusion in the actual repo.
6. Physical hardware validation — out of scope for now, simulation-only.
7. Optional/lower-priority, unresolved: a one-time robot "spawn snap" from grid origin into the tunnel at startup (cosmetic, non-blocking, cause unconfirmed); the `joint_state_broadcaster`/`platform_velocity_controller` controller spawner race inherited from Clearpath's own launch stack (self-heals, likely benign); an RViz GLSL shader warning (cosmetic, tied to software rendering).
