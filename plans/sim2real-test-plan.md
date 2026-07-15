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

---

## Phase 0 — Pre-flight (bench + environment)

| # | Step | Pass ✅ / Abort ⛔ |
|---|------|---------------------|
| 0.1 | Robot on bench, **wheels off ground**, arm folded to a safe rest pose, clear radius. | ✅ stable, nothing in arm sweep |
| 0.2 | Battery charged; kill-switch within reach; know how to cut power. | ✅ rehearsed once |
| 0.3 | User groups for device access: `groups` shows `i2c gpio dialout`. If not: `sudo usermod -aG i2c,gpio,dialout $USER` then re-login. | ✅ all three present |
| 0.4 | Devices present: `ls -l /dev/i2c-7 /dev/ttyTHS1` (and lidar `/dev/ttyUSB0` if used). | ✅ exist & readable ⛔ missing → check wiring/overlay |

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
| 2.1 | Scan I2C bus 7 | 🔧 `i2c_scan` (bus 7) | ✅ device at `0x60` (PCA9685) ⛔ absent → wiring/bus |
| 2.2 | Probe PCA9685 | 🔧 `pca9685_probe` | ✅ responds |
| 2.3 | Ping each arm servo | 🔧 `feetech_servo_ping` IDs **1,2,3,5** on `/dev/ttyTHS1` @ 1 Mbps | ✅ all reply ⛔ any silent → **STOP**: wrong baud/ID/protocol; do NOT proceed to serial ros2_control |
| 2.4 | Ping gripper servo | 🔧 `feetech_servo_ping` ID **4** | ✅ replies |
| 2.5 | Read present positions | 🔧 `feetech_servo_ping`/read on each ID | ✅ plausible 0–4095 values → record as calibration baseline |
| 2.6 | GPIO sanity (motor dir pins) | 🔧 `gpio_read` on the configured pins | ✅ readable |

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
| 3b.4 | Tiny commanded move per joint | `ros2 topic pub` a single-point JointTrajectory to `/arm_controller/joint_trajectory`, ≤0.1 rad | ✅ correct joint, correct direction, stops ⛔ wrong joint/dir → fix servo_id map / direction |

> Calibration loop: tune per-joint `offset` (zero), `direction`, `ticks_per_rad`
> as xacro `<joint>` params until commanded rad == measured pose. These default to
> center=2048, dir=+1, 4096 ticks/rev.

---

## Phase 4 — Arm via MoveIt on real servos (arm clear)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 4.1 | Named-pose move | trigger `home`/`grasp_reach` (small first) via MoveIt | ✅ real arm reaches pose, no jerk on first command |
| 4.2 | Gripper open/close | `ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04}}"` then `0.0` | ✅ jaw opens/closes; current-limited close stalls safely on an object |
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
| 6.2 | Images publish | `ros2 topic hz /stereo_camera/left/image_raw` | ✅ ~steady FPS at 640×360 |
| 6.3 | camera_info correct | `ros2 topic echo /stereo_camera/left/camera_info --once` | ✅ fx≈631, real baseline (NOT default 300/0.06 → would mean calibration URLs didn't expand; always launch via file, never bare `ros2 run`) |
| 6.4 | Disparity valid | `ros2 topic hz /stereo_camera/disparity`; set `stereo.algorithm: CPU_BM` unless OpenCV is CUDA-built (GPU_BM hard-crashes, no fallback) | ✅ publishes; non-empty |
| 6.5 | Detector (real model) | `ros2 launch jetank_detection detect_real.launch.py model_path_real:=/path/sock_real.pt` + lifecycle configure/activate | ✅ `/detections/socks` populated ⛔ no model → detector no-ops (BLOCKER: `sock_real.pt` must exist) |
| 6.6 | Segmentation → 3D | `ros2 action send_goal /segment_socks ...` | ✅ `SockCloud` centroid at plausible metric XYZ in `base_link` |

---

## Phase 7 — Pipeline integration (action servers, no full pick yet)

| # | Step | Command | Pass ✅ |
|---|------|---------|---------|
| 7.1 | Full HW stack | `ros2 launch jetank_ros_main mobile_grasp_hw.launch.py model_path_real:=/path/sock_real.pt` (+ camera in system ROS2) | ✅ comes up staggered, no crashes |
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
| 4 Arm via MoveIt | ⏳ | blocked only by the headless-CLI trigger bug — drive via RViz/interactive |
| 5 Base + odom | ⏳ | not yet run |
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
