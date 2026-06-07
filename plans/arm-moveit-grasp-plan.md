# Arm + Full MoveIt2 + Web "Grab" Button — Implementation Plan

Status: **active**
Created: 2026-06-06
Branch (all repos): `feature/arm-moveit-grasp`

## Goal

Get the JeTank arm working end-to-end through MoveIt2, and add a **Grab** button
to the web control panel that makes the robot grasp something directly in front
of it. Must work in **both Gazebo sim and on the real robot** (same code path,
sim/real selected by the existing `use_sim` ros2_control toggle).

## Locked decisions (from user, 2026-06-06)

1. **Grab semantics = phased.**
   - *Phase 1 (this plan):* open-loop **preset** grasp — no perception. The
     button triggers a deterministic sequence: `ready` pose → reach a calibrated
     pre-grasp/grasp joint pose in front → close gripper → retreat to `home`.
   - *Phase 2 (later, NOT this plan):* vision-targeted grasp using the sock
     detector + stereo depth → 3D pose → MoveIt Cartesian plan. Tracked in
     "Deferred" below.
2. **Real arm hardware = sim-first.** Full MoveIt2 + grab verified in Gazebo
   first. The real serial-bus-servo `ros2_control` plugin is the final phase,
   gated behind on-hardware testing (cannot be verified in CI / this session).

## Current state (from stack map)

**Exists & working:**
- Arm URDF: 4-DOF revolute chain `S1,S2,S3,S5` (S4 fixed) + gripper
  (`gripper_left_joint` prismatic 0–0.04 m, `gripper_right_joint` mimic).
  Tip link `S5_link`, EE `gripper_ee`. Servo IDs 1,2,3,5 (arm), 4 (gripper).
- `ros2_control` URDF has the `use_sim` split:
  `ign_ros2_control/IgnitionSystem` (sim) vs
  `jetank_motor_control/JetankSerialHardware` `/dev/ttyTHS1` @1Mbaud (real).
- MoveIt config: `arm` + `gripper` planning groups, KDL IK, named poses
  `home/ready/vertical` (arm) and `open/closed/half_open` (gripper).
  `arm_controller` = FollowJointTrajectory; gripper commanded directly on
  `/gripper_controller/commands` (Float64MultiArray) — intentionally NOT a
  MoveIt controller.
- Gazebo (gz_ros2_control) spawns `joint_state_broadcaster`, `arm_controller`,
  `gripper_controller`. Planning works in RViz.
- Web control (`jetank_web_control`, aiohttp :8080): drive, MJPEG stream,
  nav, capture/label, and a detection overlay subscribed to `/detections/socks`.

**Missing / gaps:**
- No grasp / pick / task-level action anywhere. No move_group action client.
- `moveit_bringup.launch.py` notes the **standalone** (non-Gazebo) controller
  manager currently fails to ingest controller `joints`/`command_interfaces`
  params on this RoboStack build → trajectory execution off-Gazebo unverified.
- Real `JetankSerialHardware` arm path referenced but not confirmed functional.
- No web UI arm/gripper control.
- Detection is 2D bbox only — no depth (Phase 2 blocker, out of scope here).

---

## Phases

### Phase A — MoveIt2 arm execution verified in sim   ✅ DONE (2026-06-06)
**Repos:** `jetank_moveit_config`, `jetank_description`, `jetank_motor_control`, `jetank_simulation`, `jetank_manipulation`
**Goal:** prove move_group can PLAN and EXECUTE arm trajectories + gripper open/close in Gazebo.

**Outcome:** A2 (arm plan+execute via MoveIt2) and A3 (gripper open/close) both
verified headless in Gazebo Fortress. Full `GraspObject` sequence runs
`success=true` with arm + gripper actuating; `/joint_states` confirmed.

**Key fixes discovered (gotchas for future work):**
- `--headless-rendering` segfaults the conda-forge Ogre2 build (no EGL). Removed
  from `jetank_simulation/launch/gazebo_headless.launch.py`; with `DISPLAY` set,
  GLX works in server-only (`-s`) mode.
- **Gazebo Fortress does NOT enforce the URDF `<mimic>` tag in physics.**
  The final working pattern (2026-06-06, gripper-defect fix): URDF `<mimic>` tag is
  RESTORED on `gripper_right_joint` (RSP needs it for TF + MoveIt model correctness).
  The ros2_control NATIVE mimic params (`<param name="mimic">`) are REMOVED because
  `libgz_hardware_plugins.so` appends `_mimic` to the published state interface name
  → `gripper_right_joint_mimic` in `/joint_states`, breaking MoveIt.
  Instead, `gripper_right_joint` gets its OWN position command interface and is driven
  by a `ForwardCommandController` (`gripper_right_mimic_controller`) + a small relay
  node (`gripper_mimic_relay` in `jetank_ros_main`) that mirrors `gripper_left_joint`
  state → both fingers physically move in Gazebo AND appear correctly named in
  `/joint_states` AND move_group spam is gone.
- Gripper uses `position_controllers/GripperActionController` on
  `gripper_left_joint` (`/gripper_controller/gripper_cmd`,
  `control_msgs/action/GripperCommand`), registered with MoveIt as a
  `GripperCommand` controller. `grasp_server` commands it via an action client
  (replaced the old Float64MultiArray publisher).
- Prismatic finger joints needed effort raised 0.5→5 N and contact `kp` 1e6→1e4 +
  reduced damping/friction so ign_ros2_control's low internal gain can move them.

**Gripper defect fixed (2026-06-06):** Both fingers now appear in `/joint_states`
as `gripper_left_joint` / `gripper_right_joint` (no `_mimic` suffix), TF resolves for
`gripper_right_link`, both fingers move physically in Gazebo, and the 50 Hz
`gripper_right_joint_mimic not found` spam from move_group is eliminated. All 5 goals
from the gripper-defect task verified.

---

#### Original Phase A task list (for reference)

Tasks:
- A1. Launch `moveit_sim.launch.py` (Gazebo + move_group + controllers). Confirm
  `arm_controller` and `gripper_controller` reach `active`, `/joint_states`
  publishes, and `move_group` advertises `/move_action`.
- A2. From RViz (or a scripted goal), plan+execute arm `home`→`ready`→`home`.
  Confirm the Gazebo arm moves and FollowJointTrajectory reports SUCCEEDED.
- A3. Command gripper open/close via `/gripper_controller/commands` and confirm
  motion in Gazebo.
- A4. Fix any controller spawn / param-ingest issues found (the bringup note).
  Capture the working launch invocation.

**Acceptance:** scripted plan→execute of a named arm pose succeeds in Gazebo and
gripper opens/closes on command. No errors from controller_manager.

### Phase B — `jetank_manipulation` package + `GraspObject` action
**Repos:** NEW `jetank_manipulation` (ament_cmake, own repo `kvgork/jetank_manipulation`), `jetank_ros_main` (add to `jetank.repos`)
**Goal:** a single action that performs the preset grasp via MoveIt2.

Tasks:
- B1. Scaffold package `jetank_manipulation` (ament_cmake): `action/GraspObject.action`,
  grasp server node, `config/grasp_poses.yaml`, launch `grasp.launch.py`.
  - `GraspObject.action`: Goal `string object_hint` (unused Phase 1) +
    optional overrides; Result `bool success`, `string message`; Feedback
    `string stage`.
- B2. Define preset poses as **SRDF group_states** (so MoveIt plans to them with
  collision checking): add `grasp_pre` and `grasp_reach` arm states to
  `jetank.srdf` (joint values tuned to reach ~15 cm in front at table height —
  calibrated in A/B testing). Keep them in `jetank_moveit_config` (Phase A repo).
- B3. Grasp server (Python `moveit_py` if available in the RoboStack env, else a
  thin C++ `MoveGroupInterface` node — executor picks per env probe). Sequence:
  `ready` → `grasp_pre` → open gripper → `grasp_reach` → close gripper →
  `ready` → `home`. Arm via move_group plan+execute; gripper via direct
  `/gripper_controller/commands` publisher. Each step published as feedback
  `stage`. Abort + safe-retreat on plan/execute failure.
- B4. All poses + gripper open/close widths + dwell times in `grasp_poses.yaml`.
  `use_sim_time` plumbed through launch.

**Acceptance:** `ros2 action send_goal /grasp_object jetank_manipulation/action/GraspObject '{}'`
runs the full sequence in Gazebo and returns `success: true`; arm + gripper
visibly perform reach-grasp-retreat.

### Phase C — Web "Grab" button
**Repos:** `jetank_web_control`
**Goal:** control-panel button → triggers `GraspObject`.

Tasks:
- C1. In `web_control_node.py`, add a `GraspObject` **action client** (rclpy),
  guarded so the node still works if the action server is absent (button shows
  disabled / returns 503).
- C2. Add `POST /grab` aiohttp handler: sends a `GraspObject` goal, returns
  `{ok, status}`; expose `GET /grab/status` (or push over the existing WS) so
  the UI reflects stage feedback + final result.
- C3. Add a **Grab** button to the inline `_HTML` control panel near the drive
  controls; JS posts to `/grab`, disables during execution, shows stage text and
  success/fail toast. Mobile + desktop layout.

**Acceptance:** clicking **Grab** in the browser triggers the Gazebo arm to run
the grasp sequence; UI shows stages and a success/fail result. Works against the
sim stack launched by `sim_demo`/`moveit_sim`.

### Phase D — Real arm hardware (GATED, final) — HARDWARE IMPLEMENTATION PLAN
**Repos:** `jetank_motor_control` (plugin + controllers), `jetank_description` (real-servo
limits/zeros), `jetank_moveit_config` (SRDF collision pairs)
**Goal:** make the same MoveIt2 + Grab path drive the real serial-bus servos, with
the sim as the validated reference. **Cannot be verified without the robot** — ships
as code + the on-device checklist below.

#### Real arm kinematics (confirmed with the user, 2026-06-06) — calibration reference
The sim URDF limits were tuned to the real servo travel this session. The hardware
plugin MUST map each servo's raw range to these URDF joint radians, getting the
**zero offset and direction sign** right per servo (this is the crux of sim→real):

| Joint | Servo ID | Range | URDF zero = | URDF limit (rad) |
|-------|----------|-------|-------------|------------------|
| S1 (base yaw)    | 1 | ±135° | forward      | [-2.35, 2.35] |
| S2 (shoulder)    | 2 | 180°  | upper arm **UP**; +S2 = forward | [-1.57, **2.356**] |
| S3 (elbow)       | 3 | 270°  | forearm **colinear** with upper arm | [**-2.356, 2.356**] |
| S5 (wrist roll)  | 5 | 180°  | —            | [-1.57, 1.57] |
| gripper          | 4 | —     | see gripper note | left joint 0..0.04 m |

- **S2 zero = arm pointing up**, and it can rotate ~45° **past** horizontal-forward
  (hence the +2.356 upper limit) — needed for the low forward reach.
- **S3 zero = straight elbow**, ±135° travel.
- **Per-servo zero offset + sign calibration is mandatory.** A servo "center" tick
  rarely equals URDF 0; the plugin must add a configurable offset (and possibly
  invert sign) per joint so the real arm matches the planned poses. Wrong offset =
  arm drives to the wrong place / into itself — calibrate at LOW speed first.

#### Gripper on real hardware
- **One servo (ID 4)** drives `gripper_left_joint`; `gripper_right_joint` is a
  **mechanical** mimic (no servo) — command ONLY the left joint.
- **Do NOT launch `gripper_mimic_relay` or `gripper_right_mimic_controller`** on the
  real robot — they are **sim-only** (Gazebo Fortress doesn't enforce URDF `<mimic>`
  in physics; real hardware is mechanically linked). Real bringup spawns only
  `joint_state_broadcaster`, `arm_controller` (FollowJointTrajectory),
  `gripper_controller` (GripperActionController on `gripper_left_joint`).
- Calibrate `gripper.open_width`/`close_width` (grasp_poses.yaml) to the real servo's
  open/closed encoder positions. The sim finger geometry (origins ±0.018) is cosmetic;
  what matters on HW is the servo-4 → jaw mapping.

#### Camera-on-arm collision (affects reachable workspace)
- The camera rides the arm (`imu_link←camera_link←S1_link`), so deep forward-down
  poses self-collide `S5_link`↔`camera_link`; move_group refuses them (lowest
  collision-free reach ≈ 0.08 m above floor in sim).
- On hardware: verify whether the real parts actually collide at those poses. If the
  primitive collision boxes are over-conservative and the real wrist clears the
  camera, **disable the `S5_link`↔`camera_link` pair** in `jetank.srdf`
  `disable_collisions` to unlock lower reach. If they really collide, the camera
  mount must move to reach the floor.

#### Tasks
- D1. **Serial plugin** — implement/complete `JetankSerialHardware` (`ros2_control`
  SystemInterface, `/dev/ttyTHS1` @1 Mbaud): `on_init` (parse servo_id + per-joint
  zero-offset/sign params), `on_configure`, `on_activate`, `read()` (servo position →
  joint rad, apply inverse calibration), `write()` (joint rad → servo ticks, apply
  calibration + clamp to limits). Add per-joint `offset`/`direction` params to the
  `<ros2_control>` block in `ros2_control.xacro` (under `<xacro:unless use_sim>`).
- D2. **Calibration utility** — a small node/script to jog each servo and record the
  tick↔radian mapping (zero offset + sign), writing the values back into the URDF
  params. Run BEFORE any trajectory.
- D3. **Bringup** — `moveit_bringup.launch.py use_sim:=false` → `ros2_control_node` +
  spawners (real controller set only; no mimic relay). Resolve the controller
  param-ingest issue noted in `moveit_bringup.launch.py` so `arm_controller`
  configures on the RoboStack build.
- D4. **Bench test (low speed)** — checklist below.
- D5. **Tune** `grasp_poses.yaml` for the real table height / object; re-confirm the
  camera-collision decision on real geometry.

#### On-device test checklist (user runs)
1. E-stop / kill-switch reachable; `velocity_scaling`/`acceleration_scaling` low (≤0.1).
2. `gpiochip`/serial perms OK; plugin loads (`ros2 control list_hardware_interfaces`).
3. Per-servo calibration verified: command URDF 0 → arm in the documented zero pose.
4. Single-joint jogs (small) match expected direction for S1/S2/S3/S5.
5. Gripper open/close on servo 4; jaw matches commanded widths.
6. Named-pose move `home`→`ready`→`home` via `arm_controller`.
7. Full `GraspObject` at low speed; confirm safe retreat-to-home on abort.

**Acceptance (hardware-gated):** real arm executes a named-pose move and the Grab
sequence safely at low speed, with calibrated servo offsets. Not verifiable without
the robot.

---

## Cross-cutting

- **Sim/real selection:** reuse the existing `use_sim` xacro arg on the
  `ros2_control` block — no new toggle. Grasp node + web button are
  domain-agnostic (talk to move_group + controllers, which are the same topics in
  both domains).
- **Safety:** low velocity scaling for first real runs; gripper force/width
  conservative; every grasp sequence has a guaranteed retreat-to-home on failure.
- **Do-NOT-touch:** nav (`jetank_navigation`), perception camera nodes, motor
  diff-drive path are read-only from this work. Manipulation is additive.

## Deferred (Phase 2 — not this plan)

- Stereo depth at detection bbox centroid → 3D target pose.
- Vision-targeted Cartesian grasp (replace preset `grasp_reach` with a computed
  pose). Reuses the same `GraspObject` action (add a `use_vision` goal field).
- Place / drop-off behavior; multi-object; behavior-tree sequencing.

## Branches

`feature/arm-moveit-grasp` in all touched repos: `jetank_description`,
`jetank_motor_control`, `jetank_moveit_config`, `jetank_simulation`,
`jetank_web_control`, `jetank_ros_main`, and new repo `jetank_manipulation`.

Phases A–C done + verified in sim (arm via MoveIt2, web Grab button, gripper
closes, forward-down reach). Phase D (hardware) is the spec above. Merged to
`main` 2026-06-07.
