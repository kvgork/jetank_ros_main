# Sim-to-Real Test Plan — Mobile Sock-Grasp (step by step)

Validates the `feature/sim2real-hardware-bringup` work
([gap analysis](./sim2real-gap-analysis.md) ·
[implementation plan](./sim2real-implementation-plan.md)) from a clean build all the
way to a full hardware pick.

**Golden safety rules (read before powering anything):**
1. **Wheels OFF the ground** until Phase 5b. **Arm clear of obstacles/people**
   until you trust it.
2. Keep a hand on the **power kill** (battery disconnect / e-stop). The ONLY
   software failsafe is `robot_controller`'s 1.0 s cmd_vel watchdog — it does NOT
   protect the arm.
3. **Read-only before motion.** Never enable servo torque before pinging
   (Phase 2). Never run `hardware:=serial` before `hardware:=mock` passes.
4. One joint / small amplitude / low speed first. Escalate only after each step
   passes.
5. The Feetech wire protocol in the driver is **assumed, not verified** — Phase 2
   + 3a are the gates that confirm it before any ros2_control torque.

Legend: ✅ = pass criterion · ⛔ = abort/rollback if seen · 🔧 = uses the
`hardware-test` MCP tools.

## Automated state check — `tools/robot_check.py`

Everything in this plan that a script can judge runs in one command. Use it before and after
any hardware session; it is the quickest way to know the robot's state.

```bash
python3 src/jetank_ros_main/tools/robot_check.py            # read-only, ~3 min, nothing moves
python3 src/jetank_ros_main/tools/robot_check.py --quick    # skip the mock grasp
python3 src/jetank_ros_main/tools/robot_check.py --tests    # also the full colcon test suite
python3 src/jetank_ros_main/tools/robot_check.py --motion   # + arm, gripper, track checks (asks first)
pixi run check                                              # same as the first line
```

- **Covers:** 0.3–0.6, 1.1–1.2, 2.1–2.6 (battery, I2C, all five servos read-only), 3a.1–3a.4,
  5.0 (motor outputs off at rest), 5a.1/5a.4, 6.0–6.4, 9.1–9.4, 10.1/10.3, 13.1, 14.1–14.2,
  blockers B2/B3/B4; with `--motion` also 3b.1/3b.3, 4.2, and the DC motor test 5a.2/5a.3/5a.5:
  forward, backward and both spins, each judged on the PCA9685 direction/PWM channels, the
  INA219 current rise, and the watchdog stop.
- **Safety:** a stack the script starts runs on private ROS domain 87, servos mocked, web drive
  output sent to a dead topic. `--motion` needs a terminal and a typed `yes` before torque and
  before each motion group; the arm goes limp when the run ends.
- **Output:** one line per check (PASS / FAIL / WARN / SKIP with the measured value), a summary,
  exit code 0 only when nothing fails, and `~/jetank_checks/check-<time>.json` plus launch logs.
- **Not automated:** 6.6, 7–8, 9.4 tilt test, 11, 12, browser-side 13.x, 15.4 — the script lists
  them as SKIP so a green run is not mistaken for a full pass.

---

## Whole-robot status board — 2026-10-11

The plan now covers **every part of the robot**, not only the sock-grasp path.
Phases 0–8 are the original grasp path; **Phases 9–15 are new** (IMU, lidar,
SLAM, Nav2, web control, detection setup, board peripherals). Status below is
**measured** on 2026-10-11 with no motion (servos mocked) unless marked
*from log*. Details: execution log at the end of this file.

| # | Part | Phase | Status 2026-10-11 | What it means / next action |
|---|------|-------|-------------------|-----------------------------|
| 1 | Workspace build + unit tests | 1 | ✅ 383 tests, only offline `xmllint` fails | Software baseline is green |
| 2 | **Battery / power rail** | 0.5 | ✅ 12.57 V (was 0.84 V earlier on 10-11; pack reconnected) | — |
| 3 | Servo bus (arm + gripper + camera tilt) | 2 | ✅ all 5 answer 10/10 at 6.5–6.6 V, err 0 (after the servo supply was restored) | Next: Phase 3b on real servos |
| 4 | Arm via ros2_control + MoveIt (mock) | 3a | ✅ controllers active, `/joint_states` 50 Hz, preset grasp SUCCEEDED (13 s, no gripper wait) | Software path works; real servos blocked by #2 |
| 5 | Arm on real servos | 3b–4 | 🔶 3b ✅ (incl. directions); Phase 4 ⏳ | Next: Phase 4 |
| 6 | Drive base + DC motors | 5 | ✅ 5a (wheels off ground): directions confirmed by eye, outputs, current, watchdog, odom ✅; spin only 11 % duty ⚠ | Next: 5b on the floor — check that spins still turn (B9) |
| 7 | Stereo camera | 6 | ✅ BGRx 4-channel, disparity 29.7 Hz / 19 ms, points 30 Hz / 22 ms | Raw `image_raw` **not published** with the shipped config — see blocker B3 |
| 8 | IMU (ICM-20948) | 9 | 🔶 100 Hz, |a| = 10.04 m/s², gyro ≈ 0 | Gravity is on **−Y** but URDF `imu_link` is unrotated → mounting/axis check needed |
| 9 | Lidar (RPLidar C1) | 10 | ✅ `/scan` 9.97 Hz | Plugged in, healthy |
| 10 | SLAM (slam_toolbox) | 11 | ⏳ not run on hardware this round | Needs drive (#2) to build a map |
| 11 | Nav2 + AMCL | 12 | ⏳ not run on hardware | Needs a saved map + drive |
| 12 | Web control UI | 13 | ✅ HTTP 200 in 3 ms; teleop/map/capture not exercised | Browser-side tests are manual |
| 13 | Sock detection | 14 | ⛔ no model (`~/models` missing), ultralytics only in `-e detect`, raw image topic absent | Blocks Phases 6.5–8 |
| 14 | Segmentation → 3D | 6.6 | ✅ *from 09-27* (synthetic detection, server responds) | Real run needs #13 |
| 15 | Approach + coordinator pipeline | 7–8 | ⏳ never run on hardware | Blocked by #2, #13 and blocker B2 |
| 16 | Battery telemetry (INA219) / OLED | 15 | ⚪ hardware present (0x41, 0x3C), **no code uses them** | Nothing to test in software |
| 17 | Mission (navigate→pick→deposit) | — | ⚪ `jetank_mission` package is not in `src/` | `/mission/*` web routes disabled |

Legend: ✅ pass · 🔶 partial · ⏳ not yet run · ⛔ blocked/failing · ⚪ not implemented

**Order of work from here:** 0.5 power → 2 servo presence → 3b/4 arm → 5 base →
9 IMU mount → 10/11 lidar + SLAM map → 12 Nav2 → 13 web → 14 detection → 6.5–8 pick.

---

## Phase 0 — Pre-flight (bench + environment)

| # | Step | Pass ✅ / Abort ⛔ |
|---|------|---------------------|
| 0.1 | Robot on bench, **wheels off ground**, arm folded to a safe rest pose, clear radius. | ✅ stable, nothing in arm sweep |
| 0.2 | Battery charged; kill-switch within reach; know how to cut power. | ✅ rehearsed once |
| 0.3 | User groups for device access: `groups` shows `i2c gpio dialout`. If not: `sudo usermod -aG i2c,gpio,dialout $USER` then re-login. | ✅ all three present |
| 0.4 | Devices present: `ls -l /dev/i2c-7 /dev/ttyTHS1` (and lidar `/dev/ttyUSB0` if used). | ✅ exist & readable ⛔ missing → check wiring/overlay |
| 0.5 | **Pack voltage** (read-only): 🔧 `i2c_read_register` bus 7, addr `0x41`, reg `0x02`, 2 bytes → V = (raw >> 3) × 4 mV | ✅ 11.1–12.6 V (3S) ⛔ < 10.5 V → charge/connect pack; **do not continue to servo or motor tests** |
| 0.6 | Power mode: `nvpmodel -q` | ✅ note mode (15 W today; MAXN SUPER gives more CPU for perception) |

---

## Phase 1 — Static / off-robot verification (no actuator power)

Pure software; safe to run anywhere. Most already passed during implementation —
re-run on the target Jetson.

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 1.1 | Build the branch | `~/.pixi/bin/pixi run build` | ✅ all packages finished, 0 failed |
| 1.2 | Unit/lint tests | `pixi run test && pixi run test-results` | ✅ no new failures (pre-existing `xmllint package.xml` only) |
| 1.3 | xacro selects correct plugin per mode | `pixi run bash -lc "source install/setup.bash && xacro src/jetank_description/urdf/jetank_ros2_control.urdf.xacro use_sim:=false hardware:=mock \| grep -m1 plugin"` (repeat `serial`, `use_sim:=true`) | ✅ mock→GenericSystem, serial→JetankSerialHardware, sim→IgnitionSystem; **bare (no args)→mock** |
| 1.4 | Plugin discoverable | `pixi run bash -lc "source install/setup.bash && ros2 control list_hardware_components"` after 1.5, or check `libjetank_serial_hardware.so` in `install/jetank_motor_control/lib/` | ✅ library present + plugin resource registered |
| 1.5 | Launch files parse | load `mobile_grasp_hw.launch.py` + `unified.launch.py hardware:=serial` with `--show-args` / dry import | ✅ no exceptions |

---

## Phase 2 — Hardware presence (READ-ONLY probes) 🔧

No torque, no motion. Confirms the buses + the (assumed) Feetech protocol before
ros2_control ever touches a servo. Use the `hardware-test` MCP tools.

| # | Step | Tool / Command | Pass ✅ / Abort ⛔ |
|---|------|----------------|---------------------|
| 2.1 | Scan I2C bus 7 and bus 1 | 🔧 `i2c_scan` bus 7, bus 1 | ✅ bus 7: `0x60` PCA9685 (+`0x70` all-call), `0x41` INA219, `0x3C` OLED; bus 1: `0x68` ICM-20948 ⛔ absent → wiring/bus |
| 2.2 | Probe PCA9685 | 🔧 `pca9685_probe` | ✅ responds |
| 2.3 | Ping each servo (1–3 arm, 4 gripper, 5 camera tilt) | Raw SCS ping (header `FF FF id 02 01 chk`, read-only) on `/dev/ttyTHS1` @ 1 Mbps — the MCP `feetech_servo_ping` is broken (`sms_sts` ImportError) and its motion bounds assume 12-bit servos: **do not use the MCP servo motion tools** | ✅ all reply ⛔ any silent → **STOP**: wrong baud/ID/protocol; do NOT proceed to serial ros2_control |
| 2.4 | Ping gripper servo | 🔧 `feetech_servo_ping` ID **4** | ✅ replies |
| 2.5 | Read present position + **voltage** | SCS read 8 bytes from `0x38` (pos, speed, load, volt, temp) | ✅ position 0–1023 (SCS15, centre 512); **voltage ≥ 6.0 V** ⛔ < 6 V → servo rail under-powered (go back to 0.5) |
| 2.6 | PCA9685 (drive motor PWM) | 🔧 `pca9685_probe` bus 7 `0x60` | ✅ ACKs (MODE1 sleep bit set while idle is normal). No Jetson GPIO is used by any code |

> **Gate:** if 2.3/2.4 fail, the driver's register map/endianness assumption is
> wrong. Fix `feetech_bus.hpp` constants (or baud) and re-probe **before** Phase 3.

---

## Phase 3 — Driver bring-up

### 3a. Mock backend first (NO motors move)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 3a.1 | Bring up MoveIt with mock | `ros2 launch jetank_moveit_config moveit_bringup.launch.py hardware:=mock use_sim_time:=false` | ✅ `ros2_control_node` starts |
| 3a.2 | Controllers active | `ros2 control list_controllers` | ✅ `joint_state_broadcaster`, `arm_controller`, `gripper_controller` = `active` ⛔ "joints parameter is empty" → **CM 2.54 blocker** (see gap analysis §3.1); resolve before serial |
| 3a.3 | Joint states flow | `ros2 topic echo /joint_states --once` | ✅ all arm+gripper joints listed |
| 3a.4 | MoveIt plans + "executes" (no motion) | RViz MotionPlanning → plan & execute a small move | ✅ reports SUCCEEDED, mock joints track |

### 3b. Serial backend — torque + single joint (wheels off ground, arm clear)

| # | Step | Command / Tool | Pass ✅ / Abort ⛔ |
|---|------|----------------|---------------------|
| 3b.1 | Single-joint bounded move OUTSIDE ros2_control first | 🔧 `feetech_servo_move_incremental` on ID 1, small delta | ✅ servo moves expected direction/amount ⛔ runaway → cut power; fix sign/scale |
| 3b.2 | Launch serial ros2_control | `ros2 launch jetank_moveit_config moveit_bringup.launch.py hardware:=serial use_sim_time:=false` | ✅ `on_activate` logs "torque enabled"; no ping errors |
| 3b.3 | State interfaces read real angles | `ros2 topic echo /joint_states --once` then move a joint **by hand is impossible (torque on)** → instead command tiny move | ✅ positions match Phase 2.5 baseline (rad↔tick mapping correct) ⛔ wildly off → fix `offset/direction/ticks_per_rad` params |
| 3b.4 | Tiny commanded move per joint | send a single-point goal to `/controller_manager/follow_joint_trajectory` (controllers are collapsed onto the controller_manager node on this RoboStack build), ≤0.1 rad — or use RViz MotionPlanning | ✅ correct joint, correct direction, stops ⛔ wrong joint/dir → fix servo_id map / direction |

> Calibration loop: tune per-joint `offset` (zero), `direction`, `ticks_per_rad`
> as xacro `<joint>` params until commanded rad == measured pose. These default to
> SCS15 values: centre 512, 1024 ticks over the servo range (S1/S3 `direction=-1`).

---

## Phase 4 — Arm via MoveIt on real servos (arm clear)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 4.1 | Named-pose move | trigger `home`/`grasp_reach` (small first) via MoveIt | ✅ real arm reaches pose, no jerk on first command |
| 4.2 | Gripper open/close | `ros2 action send_goal /controller_manager/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04}}"` then `0.0` | ✅ jaw opens/closes; current-limited close stalls safely on an object |
| 4.3 | Trajectory tracking | plan+execute a multi-waypoint move | ✅ follows within tolerance, no FollowJointTrajectory abort |
| 4.4 | Joint limits respected | command toward a limit | ✅ clamps, no over-travel/grinding |

---

## Phase 5 — Base + open-loop odometry

### 5a. Wheels OFF ground

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 5a.1 | Motor node up | `ros2 launch jetank_ros_main motor_controller.launch.py` | ✅ "Open-loop odometry enabled" log |
| 5a.2 | Direct drive | `ros2 topic pub -r10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}}"` | ✅ all 4 wheels spin forward ⛔ any reversed → fix motor sign |
| 5a.3 | Watchdog stop | stop publishing | ✅ wheels stop within ~1 s |
| 5a.4 | Odometry publishes | `ros2 topic echo /odom --once`; `ros2 run tf2_ros tf2_echo odom base_footprint` | ✅ `/odom` streams, TF present |
| 5a.5 | Odom integrates | pub linear.x a few s, watch `/odom` x grow; then angular.z, watch yaw | ✅ x↑ with forward, yaw↑ with rotate (sign sane) |
| 5a.6 | cmd_vel bridge | run `cmd_vel_bridge` (HW params); `ros2 topic pub /cmd_vel_manip geometry_msgs/TwistStamped ...` | ✅ plain `Twist` appears on `/cmd_vel`; wheels move |

### 5b. Wheels ON ground (open area)

| # | Step | Pass ✅ |
|---|------|---------|
| 5b.1 | Drive 0.5 m forward, measure | ✅ odom x within ~10–20% (open-loop drift expected) |
| 5b.2 | Rotate 90°, measure | ✅ odom yaw roughly tracks; note drift magnitude |

> Open-loop odom **will drift** (no encoders). Acceptance = "good enough for the
> short approach servo," not metric accuracy.

---

## Phase 6 — Perception (CSI stereo under SYSTEM ROS2)

The camera node must run under `/opt/ros/humble` (pixi OpenCV lacks GStreamer).
Run this in a **separate terminal sourced to system ROS2**, not pixi.

| # | Step | Command | Pass ✅ / Abort ⛔ |
|---|------|---------|---------------------|
| 6.1 | Both sensors open | `ros2 launch jetank_perception stereo_camera.launch.py` (system ROS2) | ✅ no GStreamer/NVMM errors ⛔ "cannot open" → check `nvarguscamerasrc`, sensor ids |
| 6.2 | Images publish | The shipped config is `compressed_only`: check `/stereo_camera/left/image_raw/compressed` (web UI) — raw `image_raw` only with `publishing.raw_images.compression.mode:=both` | ✅ ~30 FPS at 640×360; startup log prints `Frame channels: 4` (BGRx capture, raw encoding `bgra8`) |
| 6.3 | camera_info correct | `ros2 topic echo /stereo_camera/left/camera_info --once` | ✅ fx≈631, real baseline (NOT default 300/0.06 → would mean calibration URLs didn't expand; always launch via file, never bare `ros2 run`) |
| 6.4 | Disparity valid | `python3 src/jetank_perception/scripts/topic_rate_probe.py disparity` (system ROS). **Do not trust `ros2 topic hz` or the ros2-mcp rate tools for this topic** — they under-report multi-MB messages (showed 0.6 Hz when it was 30 Hz) | ✅ ~30 Hz, p50 latency < 50 ms; also `points` ~30 Hz. If GPU_BM fails to initialise, run with `camera.use_hardware_acceleration:=false` (CPU fallback) |
| 6.5 | Detector (real model) | in the detect env: `pixi run -e detect ros2 launch jetank_detection detect_real.launch.py model_path_real:=/path/sock_real.pt` + lifecycle configure/activate; needs raw `image_raw` (6.2 override) | ✅ `/detections/socks` populated ⛔ no model → detector no-ops (BLOCKER: `sock_real.pt` must exist) |
| 6.6 | Segmentation → 3D | `ros2 action send_goal /segment_socks ...` | ✅ `SockCloud` centroid at plausible metric XYZ in `base_link` |

---

## Phase 7 — Pipeline integration (action servers, no full pick yet)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 7.1 | Full HW stack | `ros2 launch jetank_ros_main mobile_grasp_hw.launch.py model_path_real:=/path/sock_real.pt` (+ camera in system ROS2). **First fix blockers B2 and B4** (bridge empty teleop topic; duplicate pixi camera) | ✅ comes up staggered, no crashes; `cmd_vel_bridge` stays alive |
| 7.2 | Action servers present | `ros2 action list` | ✅ `/segment_socks`, `/approach_target`, `/grasp_object`, `/mobile_grasp_coordinator/...` |
| 7.3 | Segment only | call `/segment_socks` with a sock in view | ✅ returns a target pose |
| 7.4 | Approach only (clear floor) | call `/approach_target` with a known point | ✅ base drives toward it via `/cmd_vel_manip`→bridge→`/cmd_vel`; stops at tolerance; uses `odom` snapshot |
| 7.5 | Grasp only (place sock at preset reach) | call `/grasp_object` | ✅ arm reaches, gripper closes on sock; verify it actually held (open-loop reports success regardless — confirm visually) |

---

## Phase 8 — End-to-end pick + regression

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 8.1 | One full pick | `ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger` | ✅ FSM SEGMENT→[APPROACH]→GRASP, sock lifted |
| 8.2 | Repeatability | run 8.1 ×10 at varied sock positions | ✅ record success rate; note failure modes |
| 8.3 | Recovery | trigger with no sock / unreachable sock | ✅ aborts cleanly, arm returns to safe pose, base stops |

---

# Part B — Whole-robot subsystems (added 2026-10-11)

Phases 9–15 cover the parts the grasp path above does not. Each phase is
independent unless it says otherwise; run them in this order. Commands run in
`pixi shell` (or `~/.pixi/bin/pixi run …`) with `ROS_DOMAIN_ID=42`, except the
camera, which runs under system ROS (`/opt/ros/humble` + `install_sys`).

## Phase 9 — IMU (ICM-20948, `/dev/i2c-1` 0x68)

| # | Step | Command | Pass ✅ / Abort ⛔ |
|---|------|---------|---------------------|
| 9.1 | Chip present | 🔧 `i2c_read_register` bus 1, addr `0x68`, reg `0x00` | ✅ `0xEA` (WHO_AM_I) |
| 9.2 | Node up + rate | `ros2 launch jetank_navigation imu.launch.py` → `ros2 topic hz /imu/data_raw` | ✅ ~100 Hz; also `/imu/magnetic_field`, `/imu/temperature` |
| 9.3 | Static sanity (robot still, level) | `ros2 topic echo /imu/data_raw --once` | ✅ |accel| ≈ 9.8 m/s²; gyro < 0.02 rad/s on all axes |
| 9.4 | **Axis/mount check** | Tilt the robot nose-down, then left-side-down; watch which accel axis changes sign | ✅ document the mapping. **Known today:** gravity reads on −Y while URDF `base_footprint→imu_link` is unrotated → add the rotation to the URDF imu joint (the IMU is a child of `camera_link`, so it also moves with S1) |
| 9.5 | Gyro sign | Rotate the robot by hand counter-clockwise (seen from above) | ✅ the axis that is vertical in the mounted frame reads positive (REP-103) |
| 9.6 | Range params | — | ⚠ `accel_range`/`gyro_range` in `icm20948.yaml` are **not read** by the node; node default bus is 7 but YAML (correctly) sets 1 |

## Phase 10 — Lidar (RPLidar C1, `/dev/ttyUSB0`)

| # | Step | Command | Pass ✅ / Abort ⛔ |
|---|------|---------|---------------------|
| 10.1 | Device present | `ls -l /dev/serial/by-id/*CP2102N*` | ✅ links to `/dev/ttyUSB0` ⛔ missing → replug USB (it was not enumerated once on 09-26) |
| 10.2 | Node healthy | `ros2 launch jetank_navigation lidar.launch.py` | ✅ log `RPLidar health status : OK` |
| 10.3 | Rate | `ros2 topic hz /scan` | ✅ ~10 Hz |
| 10.4 | Geometry | Put a box 0.5 m in front; `ros2 topic echo /scan --once` (front sector) or view in RViz on the desktop | ✅ range ≈ 0.5 m at the angle that the URDF `laser` frame calls "forward" ⛔ mirrored/rotated → fix `laser` joint yaw |
| 10.5 | Self-hits | Look at the closest ranges with nothing around | ✅ nothing < `min_range` from the robot body/arm (arm in rest pose) |

## Phase 11 — SLAM mapping (needs Phase 5 drive)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 11.1 | Launch | `pixi run slam -- rviz:=False` (navigation_full → unified: urdf+motor+imu+lidar+slam) | ✅ log shows IMU **and** lidar started (fixed 09-27 `use_sim_time` case bug) |
| 11.2 | Map grows | Drive slowly with the web UI (Phase 13) round a room | ✅ `/map` updates; web map view shows walls; no smeared double walls |
| 11.3 | Loop closure | Return to the start | ✅ map stays consistent |
| 11.4 | Save | Web "Save map" or `bash src/jetank_navigation/scripts/save_map.sh` | ✅ `~/maps/<name>.yaml` + `.pgm` written |

## Phase 12 — Nav2 + AMCL (needs a saved map)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 12.1 | Launch | `pixi run nav2 -- map:=$HOME/maps/<name>.yaml rviz:=False` | ✅ lifecycle manager: "Managed nodes are active" |
| 12.2 | Localise | Set initial pose (web UI or `/initialpose`); drive a little | ✅ `/amcl_pose` converges near the true pose |
| 12.3 | Short goal | Web "navigate" to a point 1 m away in free space | ✅ arrives within ~0.15 m; no oscillation |
| 12.4 | Obstacle | Put a box on the path | ✅ re-plans around it or stops; never touches it |
| 12.5 | cmd_vel ownership | While Nav2 drives, do **not** use web teleop | ⚠ web teleop and Nav2 both publish `/cmd_vel` with no mux on hardware |

## Phase 13 — Web control (`http://<jetson-ip>:8080`)

| # | Step | Action | Pass ✅ |
|---|------|--------|---------|
| 13.1 | Page loads | Open the URL in a desktop browser | ✅ "JeTank Controller"; HTTP 200 (measured 3 ms on 10-11) |
| 13.2 | Camera stream | Open the camera panel (camera must be running) | ✅ live image; node subscribes to the camera **only while a viewer is connected** (no subscription with the page closed) |
| 13.3 | Teleop (wheels off ground first) | Joystick forward/turn, release | ✅ wheels follow; stop within 0.5 s of release (`cmd_timeout_sec`) |
| 13.4 | Map + pose | Start mapping, open map view | ✅ map PNG updates; robot pose marker moves |
| 13.5 | Capture + label | Capture a frame, open captures, label a box | ✅ JPEG + YOLO label saved under `~/datasets/detection` |
| 13.6 | Grab button | With the arm stack up, press Grab | ✅ calls `/grasp_object`; arm runs the preset grasp |
| 13.7 | Mission routes | — | ⚪ `/mission/*` disabled: `jetank_mission` is not in `src/` |

## Phase 14 — Sock detection setup (blocks 6.5–8)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 14.1 | Detect env | `pixi install -e detect` then `pixi run -e detect python -c "import ultralytics, torch; print(torch.cuda.is_available())"` | ✅ imports; `True` for GPU on the Orin (aarch64 torch wheel not yet verified) |
| 14.2 | Model file | `ls ~/models/sock_real.pt` | ✅ exists (does **not** today — train or copy one) |
| 14.3 | Raw image input | camera with `publishing.raw_images.compression.mode:=both` | ✅ `/stereo_camera/left/image_raw` publishes (`bgra8`; detector converts to bgr8) |
| 14.4 | Detector lifecycle | `ros2 launch jetank_detection detect_real.launch.py …` → configure → activate | ✅ `/detections/socks` at ≤ 5 Hz (`max_rate_hz`); first goal not slowed by model warm-up |
| 14.5 | Accuracy | 20 frames with a sock at varied positions/lighting | ✅ record precision/recall; note false positives on similar objects |

## Phase 15 — Board peripherals and power

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 15.1 | Pack voltage | Same read as 0.5, at rest and while driving (Phase 5) | ✅ sag < 1 V under drive load |
| 15.2 | Servo rail | Servo voltage from 2.5 at rest and while the arm holds a pose | ✅ ≥ 6 V |
| 15.3 | INA219 / OLED in software | — | ⚪ no node reads them; optional feature: publish `sensor_msgs/BatteryState` from INA219 (0x41) |
| 15.4 | Thermal (long run) | Run camera + Nav2 for 20 min; `tegrastats` | ✅ no throttling; note CPU/GPU temps |

## Optional — Desktop simulation (linux-64 workstation, `-e dev`)

`pixi run -e dev gazebo` and `pixi run -e dev detect` (sim world). Validates the
September simulation fixes (spawner sequencing, physics step 2 ms, controller
dependencies) — not runnable on the Jetson.


## Blockers found 2026-10-11 (code + hardware) — fix before the phase that needs them

| # | Blocker | Blocks | Fix |
|---|---------|--------|-----|
| B1 | **Pack voltage 0.84 V**; servo rail 4.3 V | 2–5, 11–13 motion, 15 | Charge/connect/switch on the 3S pack; re-check 0.5 and 2.5 |
| B2 | `cmd_vel_bridge` subscribes to the **empty `teleop_topic: ""`** passed by `mobile_grasp_hw.launch.py:106` (only nav/manip are guarded, `cmd_vel_bridge.py:108`) → bridge dies → base never moves in APPROACH | 7.4, 8 | Guard the teleop subscription like nav/manip |
| B3 | Shipped camera config is `compressed_only` → raw `image_raw` never published; detector, `capture_frames`, `test_cameras` need it | 6.5, 7–8, 14 | Set raw `mode: both` for detection runs, or give the detector a compressed input |
| B4 | `mobile_grasp_hw.launch.py` includes `unified` without `enable_perception:=false` → a second, pixi-built camera node starts (and fails: no GStreamer in pixi) | 7.1 | Pass `enable_perception: "false"` (camera runs under system ROS) |
| B5 | No sock model; ultralytics only in `-e detect` | 6.5, 7–8, 14 | Phase 14 |
| B6 | `jetank_mission` not in `src/` | mission, web `/mission/*` | Restore the package or drop the dependency |
| B7 | IMU frame mismatch (gravity on −Y, URDF identity) | SLAM/EKF use of IMU | Phase 9.4 → URDF joint rotation |
| B8 | `grasp.launch.py` and `grasp_poses.yaml` default `use_sim_time: true` | stand-alone grasp on hardware | Override `use_sim_time:=false` (the HW launch already does) |
| B9 | **To verify on the floor (5b):** spin in place reaches only **11 % duty** per track (tracks did turn off the ground): `max_angular_velocity` 2.0 × track_width 0.11 / 2 = 0.11 m/s of 1.0 max | Nav2 rotations, SLAM turns, base_approach heading | Raise `max_angular_velocity` (≈ 7 rad/s for 40 %) or add a minimum duty for turns; measure the real deadband first |
| B10 | `motor.cpp` also writes PCA9685 channels 0–3 and never clears them on stop/zero | none today (measured: nothing is wired to 0–3) | Remove the channel 0–3 writes or clear them in `stop()` |

---

## Known blockers that will stop this plan (fix first)

These come straight from the gap analysis and bound how far you can get today:

- **`sock_real.pt` does not exist** → Phases 6.5–8 blocked until trained.
- **Feetech protocol unverified** → Phase 2 is the gate; a failed ping there
  blocks Phases 3b–8.
- **CM 2.54 "joints empty"** → may block standalone `controller_manager`
  (Phase 3a.2 / 3b.2) on RoboStack; needed for the arm to execute.
- **No `jetank_mission`** → only affects the full navigate→deposit mission, NOT
  the bare pick in this plan.
- **torch/ultralytics not in env + no TensorRT** → detector backend must be
  installed (Jetson aarch64 wheel) before 6.5.

## Abort / rollback at any phase
1. Cut power (kill switch) — fastest, always safe.
2. `Ctrl-C` the launch; `ros2 lifecycle set /sock_detector deactivate`.
3. Re-home arm in mock before re-attempting serial.
4. Revert: `git checkout main` in the three repos (branch
   `feature/sim2real-hardware-bringup` is isolated).

---

## Test execution log — 2026-06-27 (hardware bring-up session)

Arm hardware: **4× SCS15-AP + 1× SCS15-S** on `/dev/ttyTHS1` @ 1 Mbps (SCS/SCSCL:
big-endian, 0–1023, center 512). Branch `feature/sim2real-hardware-bringup`.

| Phase | Status | Notes |
|-------|--------|-------|
| 0 Pre-flight | ✅ | i2c/gpio/dialout groups, /dev/i2c-7 + /dev/ttyTHS1 present |
| 1 Static | ✅ | build, xacro 3-mode plugin (bare=safe mock), pluginlib registered |
| 2 HW presence | ✅ | PCA9685 @0x60; all 5 servos ping+read healthy ~6.5 V, err 0x00 |
| 3a Mock | ✅ | jsb + arm + gripper controllers ALL active — **CM 2.54 blocker fixed** |
| 3b minimal (raw) | ✅ | per-servo torque+move on all 5: no snap, correct dir, returned, relaxed |
| 3b full serial | ✅ | real JetankSerialHardware lifecycle: on_activate torque-all-5, real joint_states, controllers active, clean on_deactivate torque-off |
| 4 Arm via MoveIt | ✅ | 2026-06-28: arm 3-DOF (S1,S2,S3) plans+executes via RViz; S1/S3 direction fixed; gripper calibrated + user-confirmed. See 06-28 log. |
| 5 Base + odom | 🔶 | 5a partial (06-28): node up + /odom @30 Hz + odom→base_footprint TF ✓; drive checks pending |
| 6 Perception (CSI) | ⏳ | not yet run; needs system ROS2 + sock_real.pt |
| 7–8 Pipeline / pick | ⏳ | not yet run |

**Root cause fixed — CM 2.54 "joints parameter is empty":** controller_manager 2.54
loads a controller's `params_file` into the controller node where rclcpp matches
ONLY a bare `/**` wildcard key (bare name / `/<name>` / `/**/<name>` all fail).
Fix = per-controller files `jetank_motor_control/config/controllers/{arm,gripper}_controller.yaml`
under `/**`, passed via spawner `--param-file` in `moveit_bringup.launch.py`.

**Known issues:**
- Headless `ros2 action send_goal` / `ros2 topic pub` fail (`rcl context is invalid`)
  from background shells (echo/list work). Command the arm interactively (RViz).
- Calibration TODO: per-servo zero/direction; `gripper_left_joint` is prismatic
  (m) but scaled as rotary rad — needs its own mapping.

---

## Test execution log — 2026-06-28 (Phase 4 COMPLETE + camera/gripper rework)

Phase 4 (arm via MoveIt on real servos) is **done**; the three issues found on the
first real motion were fixed, committed and pushed (jetank_motor_control `910edc0`,
jetank_moveit_config `1808b2f`):

- **S1 & S3 reversed** vs RViz → `direction=-1` on S1/S3 in `ros2_control.xacro`
  (driver layer; rad↔tick write/read are inverses, so motion + reported state flip
  together). User-confirmed.
- **MoveIt execute aborted** ("Action client not connected"): this robostack
  ros2_control build collapses all controllers onto the `controller_manager` node,
  so their action servers come up at `/controller_manager/<action>` not
  `/<controller>/<action>`. Worked around by remapping move_group's arm+gripper
  action sub-interfaces in `moveit_bringup.launch.py`.
- **S5 = camera joint** (not gripper): per user, S5 removed from the MoveIt `arm`
  group (now S1,S2,S3; chain tip + EE parent_link = S4_link) and from
  `arm_controller`; no camera group; **URDF left untouched** (camera modelling is
  the user's). S5 is an uncommanded torque-held interface.
- **Gripper group couldn't plan**: `gripper_left_joint` (prismatic 0–0.04 m) was
  scaled as rotary → state ~-0.49 (out of bounds). Fixed via a metres↔ticks linear
  map in `ros2_control.xacro` (reusing offset + ticks_per_rad), **calibrated on
  hardware**: closed = tick 202 (0.0 m), open = tick 437 (0.04 m) → offset=202,
  5875 ticks/m, dir +1. User-confirmed "gripper is good".

**Operational:** launch the arm with
`ros2 launch jetank_moveit_config demo.launch.py hardware:=serial use_rviz:=false`
— NOT `moveit_bringup` alone (it has no robot_state_publisher, so RViz shows no
TF/model).

**NEXT: Phase 5 — base + open-loop odometry** (wheels OFF ground first).

---

## RESUME POINT — paused 2026-06-28 (mid Phase 5a)

**Hardware at pause:** all ROS nodes stopped; arm DE-TORQUED (limp); base idle; wheels were off the ground.

**Phase 5a progress:**
- ✅ 5a.1 motor node up — log "Open-loop odometry enabled (odom→base_footprint @ 30 Hz)"
- ✅ 5a.4 `/odom` streams @ 30 Hz; `odom→base_footprint` TF present (pose 0,0,0)
- ⏳ 5a.2 drive forward, watch all 4 wheels spin **FORWARD** (motor-sign check) — NOT YET DONE
- ⏳ 5a.3 watchdog stop (mechanism already seen: "No cmd_vel … motors stopped")
- ⏳ 5a.5 odom integrates (x↑ forward, yaw↑ on rotate)
- ⏳ 5a.6 `cmd_vel_bridge`: `/cmd_vel_manip` (TwistStamped) → Twist on `/cmd_vel`
- then **5b** (wheels ON ground): drive 0.5 m / rotate 90°, check odom drift

**To resume Phase 5a:**
1. `ros2 launch jetank_ros_main motor_controller.launch.py` — base ONLY. Do NOT also run the arm
   `demo.launch.py`: its static `odom→base_footprint` (was `world→base_footprint` before 2026-09-27) clashes with the base's `odom→base_footprint`. Use `unified.launch.py enable_moveit:=true hardware:=serial` to run arm + base together.
2. Publish `/cmd_vel` from a NON-background terminal (Claude's background `ros2 topic pub` fails
   "rcl context invalid"): `ros2 topic pub --times 30 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"`
   (then `"{angular: {z: 0.3}}"` to rotate). Claude reads `/odom` to confirm integration.

**To resume the ARM instead:** `ros2 launch jetank_moveit_config demo.launch.py hardware:=serial use_rviz:=false`
(RSP+TF+move_group; arm group = S1/S2/S3, gripper calibrated). RViz on the desktop, domain 42.


---

## Test execution log — 2026-10-11 (whole-robot status, no motion)

Branch state: all repos at the 09-27 merges (efficiency fixes), nothing newer.
Servos mocked; no wheel or arm motion.

| Phase | Result | Measured |
|-------|--------|----------|
| 0.1–0.4 | ✅ | groups i2c/gpio/dialout; `/dev/ttyTHS1`, `/dev/ttyUSB0` (CP2102N), `/dev/i2c-1`, `/dev/i2c-7`, `/dev/video0-1`; `ttyTHS1` free |
| 0.5 | ⛔ | INA219 bus voltage raw `0x0692` → **0.84 V** |
| 0.6 | ℹ | power mode 15 W |
| 1 | ✅ | `pixi run test`: 383 tests, 0 errors, 10 failures = offline xmllint only |
| 2.1 | ✅ | bus 7: 0x3C, 0x41, 0x60 (MODE1 0x11, asleep), 0x70; bus 1: 0x68 (WHO_AM_I 0xEA) |
| 2.3–2.5 | ⛔ | raw SCS ping ×10: ID 2 10/10 (pos 522, **4.3 V**, 19 °C, err 0); IDs 1,3,4,5 0/10 |
| 3a | ✅ | mock: jsb/arm/gripper active; `/joint_states` 50 Hz; `/grasp_object` preset SUCCEEDED in 13.0 s, no gripper wait |
| 5a.1/5a.4 | ✅ | `/odom` 30.3 Hz; TF `odom→base_footprint` present; drive not tested (B1) |
| 6.1–6.4 | ✅ | camera BGRx (`Frame channels: 4`); disparity 29.7 Hz p50 19 ms; points 30.0 Hz p50 22 ms (topic_rate_probe) |
| 9.1–9.3 | 🔶 | 99.8 Hz; accel (0.19, −9.98, −1.08) |a| 10.04; gyro ≈ 0 → parser OK, mounting mismatch (B7) |
| 10.2–10.3 | ✅ | `/scan` 9.97 Hz |
| 13.1 | ✅ | HTTP 200 in 3 ms |
| 0.5 / 2.3 / 2.5 (re-check, later 10-11) | ✅ | after the servo supply was restored: pack 12.57 V; servos 1–5 ping 10/10; pos 509/522/463/437/509; 6.5–6.6 V; 16–20 °C; err 0. Servo supply had been off since between 06-28 and 09-12 (09-12 already showed ~4 V at the servos). |
| 3b.1 (10-11) | ✅ | `unified.launch.py hardware:=serial enable_moveit:=true` (camera/lidar/IMU/web off): "Activated — torque enabled on all servos"; jsb, arm, gripper controllers active |
| 3b.2 (10-11) | ✅ | joint_states match ticks exactly: S1 509→0.0153, S2 522→0.0511, S3 463→0.2506, S5 509→−0.0153 rad, gripper 437→0.040 m (centre 512, 195.6 ticks/rad, S1/S3 dir −1) |
| 3b.3 (10-11) | ✅ | ±0.1 rad per joint via `/controller_manager/follow_joint_trajectory`, all SUCCEEDED; each move lands 0.01–0.04 rad short (2–7 ticks, inside the 0.05 rad goal tolerance). Directions confirmed by eye (user). |
| 5a.2/5a.3/5a.5 DC motors (10-11) | ✅ | `motor_controller.launch.py` alone, private domain 87, tracks off the ground. Forward / backward at 0.4 m/s: left+right direction pins correct, PWM 39.8 % (ch 8/13), shunt +3.8 to +7.4 mV over idle 3.3 mV, all outputs off and current back to idle 2 s after the last command (watchdog). Spins at 2.0 rad/s: directions correct (left rev/right fwd for a left turn) but only 10.9 % duty and +1.5 mV — tracks probably did not turn (B9). Channels 0–3 keep stale duty after a stop with idle current → not wired (B10). Track directions confirmed by eye (user), including both spins. |
