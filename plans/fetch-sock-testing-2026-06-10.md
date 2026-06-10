# Fetch-Sock Mission — Testing Session Plan (2026-06-10)

Resume plan: bring up the full sim with the **Gazebo GUI visible** and drive the
fetch-sock mission end-to-end. All fixes from the 2026-06-09 session are in
source and built; a clean launch activates them with no manual node restarts.

---

## Step 0 — (recommended first) commit yesterday's fixes

6 sibling repos have uncommitted, validated changes. Commit per-repo before
testing so a bad test run can't lose them:

| Repo | Change |
|---|---|
| `jetank_navigation` | AMCL Phase-2 localization tuning (`nav2_params.yaml`) |
| `jetank_web_control` | `cmd_vel_bridge` silent-when-idle + tightened seed covariance |
| `jetank_manipulation` | grasp NameError fix + `grasp_reach` S2=106° (floor reach) |
| `jetank_ros_main` | `mobile_grasp.launch.py` passes `start_arm_active:=true` |
| `jetank_mission` | SEARCH centering + `docs/fetch-sock-mission.md` |
| `jetank_perception` | height-gate ground removal (`ground_filter=height`) |

(Ask Claude: "commit all 6 repos per-repo" — each pushes to its own `kvgork/*` main.)

---

## Step 1 — Launch full sim WITH Gazebo GUI + RViz

```bash
cd /mnt/data/workspaces/jetank
pixi run -- ros2 launch jetank_mission web_mission.launch.py \
    map:=$HOME/maps/sock_arena.yaml \
    gui:=true use_rviz:=true
```

- `gui:=true` → Gazebo GUI client (watch the robot + sock + arm).
- `use_rviz:=true` → MoveIt RViz (watch arm planning + costmaps).
- Bringup is staggered ~60 s: gazebo+controllers → nav2 (+50 s) → mission+web (+60 s).
- **Caveat:** GUI + RViz add load; the launch defaults them OFF because heavy load
  can race `controller_manager` startup. If a controller fails to come up, relaunch,
  or start headless first and open the GUI second. GUI windows may open BEHIND the
  editor/browser — alt-tab.

## Step 2 — Verify clean bringup

```bash
pixi run -- ros2 control list_controllers      # all 5 active, incl. arm_controller ACTIVE
pixi run -- ros2 action list | grep -E 'run_mission|grasp_object|approach|segment'
pixi run -- ros2 param get /sock_segmentation_server ground_filter   # -> height
```
`arm_controller` MUST read `active` (the start_arm_active fix). If inactive:
`ros2 control set_controller_state arm_controller active`.

## Step 3 — Drive the fetch mission (browser)

Open **http://localhost:8080**:
1. (once) "Set deposit" mode → click map → persists `~/.jetank/deposit_pose.json`.
2. "Fetch sock" mode → click the sock's location on the map.
3. Watch FSM: NAVIGATE_TO_SITE → SEARCH (centres sock) → PICK
   (SEGMENT → APPROACH → GRASP) → NAVIGATE_TO_DEPOSIT → DEPOSIT → DONE.

In Gazebo GUI watch: base drives, SEARCH rotates to centre the sock, base
approaches to 0.18 m standoff, arm reaches to floor (S2=106°), gripper closes.

## Step 4 — What to check / likely next issues

- **Does the gripper actually HOLD the sock?** Close is `gripper.close_width=0.0`
  (shuts fully). With a sock present it should stall at sock-width. If the sock
  slips out: raise `close_width` to ~sock half-width, and/or check Gazebo finger
  friction on the sock model. (Gazebo Fortress has no grasp-fix plugin — friction
  must hold it.)
- **SEGMENT needs the sock in range + not at frame edge** — SEARCH centring + the
  height gate handle the cases seen yesterday, but a far/occluded sock can still
  miss. Aim the goal so navigation ends with the sock within ~1.5 m.
- **grasp_reach is a fixed preset** — APPROACH centres the sock under it. If the
  arm reaches but misses laterally, the approach standoff / centring tolerance
  (`mission_coordinator center_tol_frac`) or `approach_standoff` are the levers.

---

## Reference — fixes already in place (2026-06-09)

nav2 Phase-2 localization · cmd_vel_bridge silent-idle (APPROACH drives) ·
arm_controller active · grasp DONE NameError fixed · SEARCH centres sock ·
height-gate ground removal · grasp_reach S2=106° reaches floor.
Full mission completed end-to-end once yesterday. Architecture doc:
`src/jetank_mission/docs/fetch-sock-mission.md`.

## Gotchas (don't re-discover)
- After editing a `.py` in `jetank_mission` / `jetank_manipulation` (or any
  ament_cmake pkg), `colcon build --symlink-install --packages-select <pkg>`
  BEFORE restarting — `ros2 run` runs the stale installed copy. `jetank_web_control`
  IS symlinked (edits live). C++ (`jetank_perception`) always needs a build.
- Kill cleanly between runs (PID list, not `pkill -f` alternation which misses).
- `pixi.lock` v6 warning is cosmetic (pixi ≥0.69); ignore.
