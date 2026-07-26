# Troubleshooting Guide

Every issue below was actually hit and fixed during real setup/testing of this project.
If something goes wrong, check here before debugging from scratch.

---

## Environment pollution from other projects

**Symptom:** LIO-SAM crashes at runtime with `undefined symbol` errors even though it
compiled fine. Or: nodes silently can't find `robot_description` / topics never publish
data, for no apparent reason.

**Cause:** If you have another ROS 2 project on the same machine (especially one with a
manually-built GTSAM), your shell's `.bashrc` may be auto-sourcing that project's
workspace and/or prepending its library paths to `LD_LIBRARY_PATH` on every terminal —
including terminals meant for this project. This causes `lio_sam` to link against the
wrong GTSAM version, or causes broader ROS graph confusion from mixed workspace state.

**Fix:**
```bash
# Check what your .bashrc auto-sources:
grep -n "source.*setup.bash\|LD_LIBRARY_PATH" ~/.bashrc
```
If you see another project's workspace being sourced unconditionally, comment it out and
wrap it in an on-demand alias instead:
```bash
alias use_other_project='source /opt/ros/humble/setup.bash && source ~/other_project/install/setup.bash'
```
Then verify a genuinely fresh terminal is clean:
```bash
env | grep -E '^(AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH)='
```
Should print nothing when no project is manually sourced.

**Important:** some terminal applications cache environment variables across "new" tabs
within the same window. If the above check still shows pollution in a "new" tab, close
the whole terminal application and reopen it fresh (on WSL2: `wsl --shutdown` from
PowerShell, then reopen).

---

## Gazebo GUI crash (`Ogre::UnimplementedException`) {#gazebo-gui-crash}

**Symptom:** Gazebo's GUI window crashes shortly after startup with a
`GL3PlusTextureGpu::copyTo` exception.

**Cause:** A rendering path issue with WSL2's GPU virtualization and Gazebo Fortress's
OGRE2 renderer, seemingly tied to a Thermal-sensor asset inherited from the world's
SubT-competition origins.

**Fix:** Force software rendering before every launch:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```
Add this to your launch routine every time — it's not persistent across terminals unless
you add it to `.bashrc` yourself.

---

## Robot spawns but sensors never produce data

**Symptom:** The robot appears in Gazebo and responds to `cmd_vel`, but `ros2 topic hz`
on the LiDAR/IMU topics shows nothing, and `ign topic -l` shows no sensor topics at all
for the robot model.

**Cause (in our case):** This was actually downstream of the environment pollution issue
above, not a sensor configuration problem — once the polluted environment was fixed, this
resolved itself.

**If you hit this on a custom/modified world file specifically:** we were unable to get
sensors working on a hand-built test world despite extensive investigation (ruled out:
world geometry, `setup_path` caching, world-name threading through Clearpath's generator).
If you're building a custom world for faster iteration, be aware this is an open,
unresolved issue — stick to the real project world if you hit it.

---

## `robot_description` / spawn stall (`create-21` node stuck)

**Symptom:** Launch hangs indefinitely with `create-21` (the robot spawner) printing
`Requesting list of world names` or `Waiting messages on topic [robot_description]`
forever.

**Cause:** Same root cause as the environment pollution issue above, OR (separately)
FastDDS shared-memory transport failures under WSL2 (`RTPS_TRANSPORT_SHM Failed
init_port` errors flooding the log).

**Fix:** Fix environment pollution first (see above) — this resolved it in our case
without needing to touch DDS settings. If it persists, consider switching to Cyclone DDS:
```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Install script crashes with `unbound variable`

**Symptom:** `scripts/install_dependencies.sh` dies partway through with an error like
`AMENT_TRACE_SETUP_FILES: unbound variable` or `AMENT_PYTHON_EXECUTABLE: unbound
variable`.

**Cause:** The script uses `set -euo pipefail`, which conflicts with internals of ROS 2's
own `setup.bash` files (they reference some variables without a default value under
strict mode).

**Fix:** Relax the script's strict mode:
```bash
sed -i 's/set -euo pipefail/set -eo pipefail/' scripts/install_dependencies.sh
```
This repo's `scripts/setup.sh` already has this fix applied.

---

## `apt install` hangs / dpkg lock held

**Symptom:** `apt install` or `apt-get update` hangs indefinitely, or errors with
`Could not get lock /var/lib/apt/lists/lock`.

**Cause:** Ubuntu's automatic background updater (`unattended-upgrades`) is running and
holding the lock — common right after a fresh WSL2/Ubuntu install.

**Fix — wait it out (usually 1-5 minutes), or:**
```bash
sudo killall unattended-upgrade unattended-upgrades 2>/dev/null
sudo dpkg --configure -a
sudo systemctl stop unattended-upgrades
sudo systemctl disable unattended-upgrades   # re-enable later if you want
```

---

## Nav2 costmap error: "Transform data too old" / robot appears to teleport at startup

**Symptom:** The robot briefly appears at the map origin (0,0,0) then snaps to its real
spawn position once TF settles.

**Cause:** This is a normal, one-time startup transient from how the spawn/TF chain
initializes — not a bug. It happens once, before navigation begins, and doesn't recur
during normal operation.

**Fix:** None needed. If you see this repeatedly *during* a run (not just at startup), that
indicates a real performance problem (see next entry).

---

## Long runs eventually show TF timing errors and Nav2 stops planning

**Symptom:** After 40+ minutes of a sustained run (Gazebo + LIO-SAM + SLAM Toolbox + Nav2
+ an exploration node all running together), you start seeing `Transform data too old`
and `Unable to transform robot pose into global plan's frame`.

**Cause:** Likely a real-time performance hiccup — this is a heavy stack, and WSL2 can
struggle to keep up with all of it simultaneously over a long sustained run.

**Fix:** No general fix; consider running shorter sessions, or check `wsl --shutdown` /
restarting between long runs to clear any accumulated resource pressure. Increasing
`.wslconfig` memory allocation may help:
```ini
# %USERPROFILE%\.wslconfig
[wsl2]
memory=10GB
processors=4
```

---

## Robot never returns home / explores forever

**Symptom:** Even with `return_to_start_on_complete: true`, the robot never actually
returns home, especially on a large map.

**Cause and fix:** See `patches/husky_tunnel_bringup_return_home_fix.patch` and
`patches/husky_tunnel_bringup_mission_duration_cap.patch`, and the detailed writeup in
`docs/PROGRESS_LOG.md` §2.3–2.4 for the full investigation (it took three iterations to
get the duration-cap fix fully correct — worth reading if you're modifying this logic).

---

## Robot gets stuck / `Failed to make progress` in tight corners

**Symptom:** The robot repeatedly aborts navigation in narrow sections of the tunnel,
sometimes for extended periods (30+ minutes in our testing).

**Cause and fix:** See `patches/husky_tunnel_bringup_nav_fixes.patch`. Root cause was
costmap inflation radius leaving too little usable space combined with only one recovery
retry configured.

---

## Terminal job confusion (`Ctrl+Z` instead of `Ctrl+C`)

**Symptom:** Multiple overlapping launches seem to be running at once, causing wildly
inconsistent behavior (competing controllers, doubled log output, `RTPS_TRANSPORT_SHM`
spam).

**Cause:** `Ctrl+Z` suspends a process rather than terminating it — the previous launch
(including Gazebo and every node it spawned) stays alive in the background.

**Fix:** Always use `Ctrl+C` to stop a launch, never `Ctrl+Z`. Check for stray suspended
jobs before starting a new one:
```bash
jobs
# if anything shows "Stopped":
kill -9 %1   # (or the relevant job number)
```

---

## Two `.pgm`/`.yaml` maps won't compare correctly

**Symptom:** Using `tools/map_completeness.py`, a strict wall-completeness result comes
back suspiciously low even when a run clearly covered the same area as the reference.

**Cause:** This is expected, not a bug — two independent SLAM sessions rarely register
thin walls to the exact same pixel, even for the same physical wall. Use the tool's
"tolerant" completeness number, not the strict one, when comparing two separate sessions.
See `docs/PROGRESS_LOG.md` §5.3 for the full explanation, including a real finding about
why wall completeness in particular tends to lag free-space completeness.
