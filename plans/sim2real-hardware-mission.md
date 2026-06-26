# Sim→Real: Fetch-Sock Mission on Hardware (2026-06-26)

**Scope (locked with user):** off-robot CODE work only · greenfield (never run on HW) ·
deliver (1) hardware mission launcher, (2) sim/real code guards, (3) on-robot
calibration/bringup runbook. Calibration / model-training / nav-tuning / thermal
are **documented for the user to run on the Jetson**, NOT done here (robot not reachable).

**Verification is off-robot:** `colcon build`, unit tests for mux logic, launch-file
static import check. No real-robot run is possible this session.

Session: `20260626-132421-sim2real-fetch`.

---

## Root finding — the base-motion chain is sim-shaped

| Producer | Topic (current) | Type | Consumer on HW? |
|---|---|---|---|
| Nav2 `velocity_smoother` | **`/cmd_vel`** (`cmd_vel_smoothed→cmd_vel` remap, `nav2_bringup.launch.py:198`) | Twist | ✅ motor |
| `cmd_vel_bridge` (sim only) | `/diff_drive_controller/cmd_vel` | TwistStamped | gz only |
| `base_approach_node` | `/diff_drive_controller/cmd_vel` (param `cmd_vel_topic`) | TwistStamped | gz only |
| `mission_coordinator` (SEARCH) | `/diff_drive_controller/cmd_vel` (param `cmd_vel_topic`) | TwistStamped | gz only |
| **real motor** `robot_controller.cpp:44` | **`/cmd_vel`** | **Twist** | ✅ motor |

On hardware: (a) nothing converts the manip/teleop TwistStamped→Twist onto `/cmd_vel`
⇒ base won't move during SEARCH/APPROACH; (b) **Nav2 ALREADY owns `/cmd_vel`** via its
velocity_smoother (verified `nav2_bringup.launch.py:198`). So the arbiter must NOT also
publish to `/cmd_vel` for nav — that would double-publish AND bypass the smoother
(accel limits). **Corrected design: Nav2 keeps `/cmd_vel` during NAVIGATE; the arbiter
muxes only teleop + manip → `/cmd_vel`.** The mission FSM sequences NAVIGATE vs SEARCH/PICK
so the two are not simultaneously active (one active publisher at a time).

## Topic contract (FROZEN across phases — Phase 1 producer, Phase 2 consumer)

```
web teleop    --Twist-->        /cmd_vel_teleop  --\
base_approach --TwistStamped--> /cmd_vel_manip   ---> [cmd_vel_bridge HW] --Twist--> /cmd_vel --> motor
mission_coord --TwistStamped--> /cmd_vel_manip   --/    priority: manip > teleop; silent when both idle
Nav2 velocity_smoother ----------------------------------Twist--------------------> /cmd_vel --> motor
        (owns /cmd_vel during NAVIGATE; arbiter is idle/silent then)
```

Frozen param names / values:
- `cmd_vel_bridge` new params: `output_stamped` (bool, default `true`=sim), `manip_topic`
  (str, default `''`=disabled), `manip_timeout` (float, default `0.5`).
- **Empty-topic guard:** when `nav_topic` OR `manip_topic` is `''`, do NOT subscribe it.
  This both disables nav on HW and prevents a feedback loop (arbiter subscribing its own
  `/cmd_vel` output). Sim keeps `nav_topic=/cmd_vel` (no loop: sim output is the *gz* topic).
- HW arbiter config: `output_stamped:=false`, `output_topic:=/cmd_vel`,
  `teleop_topic:=/cmd_vel_teleop`, `manip_topic:=/cmd_vel_manip`, **`nav_topic:=''`** (Nav2 owns /cmd_vel).
- `base_approach_node` + `mission_coordinator` get `cmd_vel_topic:=/cmd_vel_manip` via launch param (NO code change).
- Sim path unchanged: `output_stamped` defaults true, `manip_topic` empty ⇒ identical behavior.
- Residual (documented, not guarded): manual teleop *during* active Nav2 = brief two-publisher
  overlap on `/cmd_vel`. Manual-override edge case; note in runbook.

---

## Phase 1 — Hardware cmd_vel arbiter (`jetank_web_control`)

**File:** `jetank_web_control/jetank_web_control/cmd_vel_bridge.py`

- Add params `output_stamped`, `manip_topic`, `manip_timeout` (above).
- **Empty-topic guard:** only subscribe `nav_topic` / `manip_topic` when non-empty (prevents
  the arbiter subscribing its own `/cmd_vel` output on HW, and disables nav on HW).
- If `manip_topic` non-empty: subscribe it as **TwistStamped**, highest priority.
- Priority in `_tick`: manip (if fresh) → teleop (if fresh) → nav (if fresh & subscribed) → silent.
- Output: when `output_stamped` true → TwistStamped on `output_topic` (sim, unchanged);
  false → **Twist** on `output_topic` (hardware).
- Preserve idle-silence (no zero-flood; motor watchdog `robot_controller.cpp` ~1.0s stops base —
  NOTE: on HW the motor watchdog is the ONLY stop, ~1s coast on handoff; documented in runbook).
- Keep backward-compat: defaults reproduce current sim behavior exactly.

**Tests:** `jetank_web_control/test/test_cmd_vel_bridge.py` (pure logic, no ROS spin or hardware):
priority ordering, staleness/timeout fallthrough, output type per `output_stamped`,
silence when all idle. Stub rclpy per the package's existing test stubbing pattern.

**Acceptance:** `python -m pytest test/test_cmd_vel_bridge.py` green; `colcon build --packages-select jetank_web_control` clean.

## Phase 2 — Hardware mission launcher (`jetank_mission`)

**File:** `jetank_mission/launch/web_mission_hw.launch.py` (mirror of `web_mission.launch.py`, hardware side)

**Prereq code change — `unified.launch.py` (`jetank_ros_main`):** add a `hardware` arg
(default `mock`) and pass it through to `moveit_bringup.launch.py` (which defaults
`hardware:=mock` → mock ros2_control backend → arm reports FollowJointTrajectory SUCCESS
while NO servo moves; real backend is `serial`/JetankSerialHardware). Also expose
`frames.left_frame_id`/`frames.right_frame_id` passthrough to `stereo_camera.launch.py`
(real config defaults non-optical `camera_*_link`; sim uses `*_optical_frame`).

Composes (staggered TimerActions, hardware-settle timing, all `use_sim_time:=false`):
1. `unified.launch.py` — `use_sim_time:=false enable_moveit:=true enable_navigation:=true
   navigation_mode:=nav2 map_file:=<map> enable_web_control:=false **hardware:=serial**`
   (brings up urdf, motor, stereo_camera real, imu, lidar, moveit_bringup[serial], nav2_bringup).
   Pass camera `frames.{left,right}_frame_id:=camera_{left,right}_optical_frame` so disparity/
   pointcloud reprojection matches the sim geometry (else wrong 3D centroid → base drives wrong).
2. Pick pipeline (hardware): `detect_real.launch.py` (model_path_real, `continuous:=true`),
   `sock_segmentation_server`, `grasp_server`, `base_approach_node`
   (`cmd_vel_topic:=/cmd_vel_manip`), `mobile_grasp_coordinator` — all `use_sim_time:=false`.
   Lifecycle configure+activate `/sock_detector` (mirror sim timing).
3. `mission_coordinator` — `use_sim_time:=false`, `cmd_vel_topic:=/cmd_vel_manip`.
4. `web_control.launch.py sim:=false cmd_vel_topic:=/cmd_vel_teleop` + **HW arbiter**
   (`cmd_vel_bridge` with frozen HW config above: `output_stamped:=false nav_topic:='' manip_topic:=/cmd_vel_manip`).

Args: `map` (required), `model_path_real` (default `~/models/sock_real.pt`),
`confidence` (0.3). Document timer periods as HW-tunable (greenfield estimate).

**Guards verified:** confirm `stereo_camera.launch.py` publishes the disparity/pointcloud
topics `sock_segmentation_server` consumes (real GPU path); if topic names differ from the
sim `stereo_camera_sim` source, remap in the launcher. Do NOT include `navigation_full`
(would double-start hardware vs `unified`) — use `unified`'s `nav2_bringup` path.

**Acceptance:** `colcon build --packages-select jetank_mission` clean; launch-file imports
and `generate_launch_description()` returns a `LaunchDescription` without error
(`python -c "import importlib.util; ...web_mission_hw..."`); `ros2 launch ... --show-args`
lists `map`, `model_path_real`. (Full launch not runnable off-robot — opens /dev/i2c.)

## Phase 3 — On-robot bringup + calibration runbook (`jetank_ros_main/docs/`)

**File:** `jetank_ros_main/docs/hardware-bringup-fetch-sock.md`

Step-by-step the USER runs on the Jetson. **SAFETY FIRST — first real motion:**
- **First bringup wheels-OFF-ground + arm clear of obstacles, hand near power.** A sim-validated
  mission's first run on real motors is the highest-risk moment (1.0s motor watchdog is the only
  base stop; arm now actuates via `serial`). Verify before any goal.
- e-stop / kill story: how to cut motor power; `ros2 topic pub /cmd_vel ... 0` does NOT stop a
  runaway if a node floods — kill the launch / power.

Bringup + checks:
- Device perms: `/dev/i2c-*`, `/dev/gpiochip*`, `/dev/ttyTHS1` (servos), `/dev/ttyUSB0` (lidar);
  JetPack GStreamer NVMM for HW camera decode (per CLAUDE.md gotcha — run perception against
  host `/opt/nvidia/gstreamer/*` outside pixi if conda GStreamer lacks NVMM).
- **Camera frames check:** confirm `stereo_camera_node` actually emits frames inside pixi (CSI/NVMM);
  echo disparity/pointcloud topics non-empty; confirm frame_id is `*_optical_frame`.
- Per-subsystem smoke tests (echo `/scan`, `/odom`, `/imu/data_raw`, camera, `/detections/socks`).
- **Arm actuation check:** with `hardware:=serial`, command a small MoveIt move; confirm a servo
  physically moves (catches the mock-backend silent-success failure).
- cmd_vel arbiter verification: `ros2 topic echo /cmd_vel` while (a) teleop, (b) Nav2 goal,
  (c) pick active — confirm exactly one active source; note the teleop-during-nav overlap caveat.

Pointers (LARGE independent efforts — this plan does NOT deliver them, only flags):
- Motor calibration (alpha/beta) → `motor_params.yaml`.
- Stereo calibration (checkerboard) → `calibration/{left,right,stereo}_camera.yaml`, baseline.
- Real sock model: collect dataset → train → place `sock_real.pt`; set `model_path_real`.
- AMCL / Nav2 tuning for real (drifting odom + noisy scan).
- Thermal soak (`tegrastats`) under full stack.
- Launch: `ros2 launch jetank_mission web_mission_hw.launch.py map:=<map> model_path_real:=<pt>`.

**Acceptance:** doc exists, covers safety-first bringup + all subsystem checks + arm + arbiter + launch command.

---

## Out of scope (needs robot; runbook pointers only — NOT cheap)
Real calibration values, `sock_real.pt` dataset+training, nav param tuning, thermal validation.
Each is a substantial standalone effort; bullet-listing them does not make them quick.
