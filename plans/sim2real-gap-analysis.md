# JeTank Mobile Sock-Grasp: Sim-to-Real Gap Analysis

**Scope:** Getting the mobile pick-and-place workflow (`segment → approach → grasp`) running on the **physical** JeTank (Jetson Orin Nano Super). It currently runs end-to-end in Gazebo Fortress.

**Date:** 2026-06-27 · All claims grounded in workspace files at `/home/koen/workspaces/ros2_ws/src`.

---

## 1. Executive Summary

**No — the pick-and-place workflow cannot run on the physical robot today, and is not close.** The single biggest blocker is that **the arm and gripper have no working hardware backend**: `jetank_motor_control/JetankSerialHardware`, the ros2_control `SystemInterface` that MoveIt drives the Feetech servos through, is a documented stub with zero C++ implementation (`jetank_motor_control/jetank_hardware.xml` says verbatim "STATUS: stub / not yet implemented"; no source exists, no library is built, no pluginlib export). Without it `controller_manager` cannot load the arm, so every grasp fails before a servo moves.

Compounding this, **there is no hardware launch for the pipeline at all** — `mobile_grasp.launch.py` is hardcoded sim-only and the documented one-command entrypoint (`jetank_mission/web_mission_hw.launch.py`) belongs to a package that **does not exist in this workspace**. Three further hard blockers stand independently: the base has **no odometry** (`odom→base_link` TF) on hardware (open-loop PCA9685 motors, no encoders), the **base never receives drive commands** (a TwistStamped-vs-Twist topic mismatch), and there is **no real-trained sock model** (no `sock_real.pt`, the `/home/koen/models/` directory does not even exist).

The good news: the manipulation/perception **node logic is largely transport-portable**, real **camera calibration already exists and is wired in**, the **CSI camera launch and dual-backend node already exist**, the **cmd_vel arbiter is implemented and unit-tested**, and the **sim arena does contain socks**. The remaining work is dominated by writing one C++ hardware driver, providing an odom source, training a model, and authoring a hardware launch.

---

## 2. The Workflow & What "Works in Sim" Actually Means

### Node / topic graph (sim)

A pick is triggered by:
```
ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger
```

`mobile_grasp_coordinator.py` is a `MultiThreadedExecutor` FSM (`SEGMENT → REACH_CHECK → [APPROACH] → GRASP`) that chains three action clients:

```
                    /segment_socks (jetank_detection/SegmentSocks)
mobile_grasp_         │   served by jetank_perception/sock_segmentation_server
 coordinator   ───────┼── /approach_target (jetank_manipulation/ApproachTarget)
                      │   served by base_approach_node
                      └── /grasp_object (jetank_manipulation/GraspObject)
                          served by grasp_server
```

Data flow upstream of the FSM:
```
Gazebo stereo cam ─► /stereo_camera/left|right/image_raw
   └► stereo_camera_node (CPU SGBM) ─► /stereo_camera/disparity + left/camera_info
   └► sock_detector (YOLO, sock_sim.pt) ─► /detections/socks (Detection2DArray)
        └► sock_segmentation_server: reproject bbox→3D cloud, RANSAC ground removal,
           Euclidean cluster, TF to base_link ─► SockCloud(centroid, dims, score)
```

Actuation:
- `grasp_server` drives the arm via MoveIt `/move_action` (hand-built `MotionPlanRequest`s; `moveit_py` unavailable in RoboStack) and the gripper via `/gripper_controller/gripper_cmd` (`control_msgs/GripperCommand`).
- `base_approach_node` is a proportional rotate-then-drive servo publishing `TwistStamped` to `/diff_drive_controller/cmd_vel`, snapshotting the target into `odom` and re-TFing `odom→base_link` each tick.

### The sim shortcuts it depends on (the sim-to-real delta)

| Sim shortcut | File / evidence | Why it breaks on hardware |
|---|---|---|
| Arm/gripper controllers come from `ign_ros2_control` (Gazebo's `controller_manager`) | `ros2_control.xacro:22-26` (`use_sim` → IgnitionSystem) | Real branch needs `JetankSerialHardware` — unimplemented |
| Base driven by Gazebo `diff_drive_controller` on `/diff_drive_controller/cmd_vel` (TwistStamped) | `jetank_controllers.yaml` "simulation only" | Real base = `robot_controller.cpp` subscribing `Twist` on `/cmd_vel` (`robot_controller.cpp:44`) |
| `odom→base_link` TF from Gazebo wheel odometry | `diff_drive_controller` `enable_odom_tf` | Real base has no encoders, publishes no odom (`robot_controller.cpp` has zero `nav_msgs`/TF) |
| `use_sim_time: True` baked on every node | `mobile_grasp.launch.py:44` | No `/clock` on hardware → frozen clocks, every deadline/TF lookup misbehaves |
| Stereo from Gazebo clean rectified stream | `stereo_camera_sim.launch.py` (`input_source=ros_topics`) | Real path needs CSI/GStreamer under system ROS2 |
| YOLO model `sock_sim.pt` | `mobile_grasp.launch.py:97` | Sim model won't transfer; `sock_real.pt` doesn't exist |
| Gripper right finger via `gripper_mimic_relay` + `gripper_right_mimic_controller` | `gripper_mimic_relay.py:16-19` | Sim-only by design; real jaw is one mechanically-linked servo |
| Sock targets are Gazebo cylinder models | `jetank_ros_main/worlds/sock_arena.sdf` (13 sock refs) | Real socks differ in appearance/geometry |

**What "works in sim" therefore means:** the FSM, action interfaces, reprojection math, base servo, and MoveIt planning are exercised and correct — but every actuation, sensing, and timing primitive is supplied by Gazebo. The physical substrate underneath is largely absent or stubbed.

---

## 3. Sim-to-Real Gaps by Subsystem

Subsystems ordered by aggregate severity. Severity badges: 🔴 **blocker** · 🟠 **high** · 🟡 **medium** · ⚪ **low**. Effort: hours / days / weeks.

---

### 3.1 Control + Hardware Interface (highest aggregate severity)

#### 🔴 No `JetankSerialHardware` ros2_control SystemInterface — *weeks*
- **Now:** `jetank_hardware.xml` declares `jetank_motor_control/JetankSerialHardware` (base `hardware_interface::SystemInterface`), referenced by `ros2_control.xacro:28-34` (`/dev/ttyTHS1` @ 1 Mbaud). **Verified:** `grep JetankSerialHardware --include=*.cpp/*.hpp` returns nothing; `src/motor/` contains only `motor.cpp` + `robot_controller.cpp`; `CMakeLists.txt` builds no plugin library, no `PLUGINLIB_EXPORT_CLASS`, no `pluginlib_export_plugin_description_file`; no Feetech/ST/SC serial protocol code anywhere. The XML header itself says "STATUS: stub / not yet implemented."
- **Needed:** Implement the SystemInterface (`on_init`/`export_state_interfaces`/`export_command_interfaces`/`read`/`write`), Feetech ST/SC serial protocol over `/dev/ttyTHS1` (sync read/write position+velocity for servo IDs 1/2/3/5 = S1/S2/S3/S5 and ID 4 = gripper_left_joint), rad↔tick scaling + direction signs per servo, build/export the shared lib, register the plugin description.
- **This is the headline blocker.** All arm/gripper actuation on hardware depends on it. *Note:* the `hardware-test` MCP exposes typed `feetech_servo_*` tools, so the Feetech wire protocol is partly solved at the tooling level — lower bound could shrink toward days if reusable, but a production driver + calibration + bring-up justifies weeks.

#### 🔴 Base motors not in ros2_control — no odometry on hardware — *days–weeks*
- **Now:** `robot_controller.cpp` is a standalone node: subscribes `/cmd_vel` (Twist), drives PCA9685 PWM over I2C (`motor.cpp`, `/dev/i2c-7`, addr `0x60`), **open-loop, no encoders**. It publishes only a `std_msgs/String` status — no `joint_states`, no `nav_msgs/Odometry`, no TF. The `diff_drive_controller` + wheel joints in `jetank_controllers.yaml`/`ros2_control.xacro` are flagged "simulation only."
- **Needed:** Provide `odom→base_link`. Either (a) a base ros2_control velocity SystemInterface so `diff_drive_controller` runs, or (b) make `robot_controller.cpp` publish odometry. **Caveat:** no wheel encoders exist, so any odometry is open-loop (command-integrated, drifting) unless encoders are added or IMU/visual odom is fused. The base-approach servo and remembered-pose grasp recovery **strictly require** this TF (see §3.4).

#### 🔴 CM 2.54 "joints parameter is empty" blocks the *standalone* controller_manager — *days*
- **Now:** `moveit_bringup.launch.py:85-90` documents (verified in-file) that on `controller_manager 2.54` (RoboStack), `joint_state_broadcaster` activates but `arm_controller`/`gripper_controller` fail at configure with "'joints' parameter is empty"; only RViz planning works. **Scope correction:** this affects the *standalone* `ros2_control_node` path (i.e. the hardware/mock path), **not** the sim pick workflow, which runs through `gz_ros2_control` and executes trajectories fine. But the hardware path will hit this the moment the serial driver exists.
- **Needed:** Root-cause the param-namespacing regression (per-controller param files, or pin a working `controller_manager`/`ros2_controllers`).

#### 🟠 `hardware:=mock|serial` launch arg is silently ignored by the xacro — *hours*
- **Now:** `unified.launch.py` and `moveit_bringup.launch.py` pass `mappings={'hardware': hardware}`, but `jetank_ros2_control.urdf.xacro:7` declares **only** `use_sim` (default false) and threads only that into the macro (`:75`). **Verified:** there is no `hardware` arg and no `mock_components/GenericSystem` branch. Plugin selection is governed solely by `use_sim`. **Consequence — a safety hazard:** `hardware:=mock` (the documented "software-only, no motors move" mode) silently resolves to the **real** serial plugin, not a mock.
- **Needed:** Add a `hardware` xacro arg threaded into the macro with three branches: `sim`→IgnitionSystem, `mock`→`mock_components/GenericSystem`, `serial`→JetankSerialHardware. No C++ needed; `GenericSystem` ships with ros2_control.

#### 🟠 Gripper single-servo jaw path unimplemented (mimic exclusion already handled) — *weeks (gated by serial driver)*
- **Now:** Sim uses `gripper_right_mimic_controller` + `gripper_mimic_relay.py` (Gazebo ignores URDF `<mimic>`). **Already handled:** a single `gripper_controller` (`position_controllers/GripperActionController` on `gripper_left_joint`, `/gripper_controller/gripper_cmd`) is already defined (`jetank_controllers.yaml:19-20`) and matches `grasp_server`; the mimic controller/relay are spawned only by the Gazebo launches, so they are excluded from hardware by construction.
- **Needed:** The real blocker is the same missing serial driver: it must map `gripper_left_joint` (servo ID 4, prismatic 0–0.04 m) to ticks + a current/torque-limited close. Calibration of open/close width and effort is hours once that exists.

---

### 3.2 Workflow Orchestration (launch files / node wiring)

#### 🔴 `jetank_mission` package is missing (mission FSM + one-command stack) — *days*
- **Now:** `README.md`, `SIM_CONTROL.md`, `docs/hardware-bringup-fetch-sock.md:364` all reference `jetank_mission` (mission_coordinator FSM, `RunMission` action, `web_mission.launch.py`/`web_mission_hw.launch.py`) as the navigate→search→pick→carry→deposit entrypoint. **Verified:** `ls jetank_mission` → no such directory; neither `web_mission*.launch.py` nor a `mission_coordinator` node exists anywhere. `jetank_web_control` declares `<exec_depend>jetank_mission</exec_depend>` and imports `jetank_mission.action.RunMission`, guarded by try/except (`_MISSION_AVAILABLE=False`) — so the web node degrades gracefully and the build still succeeds.
- **Needed:** Clone/restore or rebuild. Per `web-mission-plan.md` the coordinator is a *thin* FSM over already-present interfaces (Nav2 `NavigateToPose`, the coordinator's `execute_sock_grasp` Trigger, gripper `GripperCommand`) — so days, not weeks. **The bare pick stack (`mobile_grasp.launch.py`) runs without it.**

#### 🟠 No hardware launch wires the pick pipeline — *days*
- **Now:** `mobile_grasp.launch.py` is the only launch chaining `segment→approach→grasp`, hardcoded sim-only (`sim_time={"use_sim_time": True}` at `:44`, applied to all four nodes `:79-87`; includes `gazebo_sim.launch.py`). **Verified:** `unified.launch.py` runs the real base/arm/sensors/MoveIt(`hardware:=serial`)/Nav2 but includes **zero** pick-pipeline nodes (`grep` count = 0). The four nodes appear in launches only in `mobile_grasp.launch.py` and `jetank_manipulation/grasp.launch.py`.
- **Needed:** Author the hardware pick launch (the spec already exists at `docs/hardware-bringup-fetch-sock.md:355-375`): `unified.launch.py hardware:=serial` + `detect_real.launch.py` + the four nodes with `use_sim_time:=false` + lifecycle configure/activate, with real-hardware stagger timers. Mechanical rewiring of existing pieces.

#### 🔴 cmd_vel mismatch: pick base motion never reaches the motors — *hours*
- **Now:** **Verified both ends.** Producer `base_approach_node.py:144` defaults `cmd_vel_topic=/diff_drive_controller/cmd_vel` and publishes `TwistStamped`. Consumer `robot_controller.cpp:44-45` subscribes `geometry_msgs/Twist` on `/cmd_vel`. Double mismatch (type **and** topic) → no connection forms; the base receives no drive commands during SEARCH/APPROACH. The fix-it bridge (`cmd_vel_bridge.py`) is implemented and unit-tested but only wired into the **sim-gated** `web_control.launch.py`.
- **Needed:** In a hardware launch, set `base_approach_node` `cmd_vel_topic:=/cmd_vel_manip` and run `cmd_vel_bridge` with `output_stamped:=false, output_topic:=/cmd_vel, manip_topic:=/cmd_vel_manip, nav_topic:=''`. Launch wiring only — no node code changes.

#### 🟠 `use_sim_time` hardcoded true; no flip arg in `mobile_grasp.launch.py` — *hours*
- **Now:** `mobile_grasp.launch.py:44` is a fixed dict with no override; `grasp_poses.yaml:47` and `grasp.launch.py` default true. With `use_sim_time:=true` and no `/clock`, node clocks freeze — `base_approach_node` deadlines and coordinator `_spin_until` timeouts never advance, TF lookups misbehave. *(Note: `grasp.launch.py` itself already accepts the arg; the hardcoding is specific to `mobile_grasp.launch.py` + the YAML default.)*
- **Needed:** Thread a `use_sim_time` arg (default false) through the hardware launch.

---

### 3.3 Perception / Detection

#### 🔴 No real-trained sock model (and model dir missing) — *weeks*
- **Now:** **Verified:** no `*.pt` anywhere under `/home/koen`; `/home/koen/models/` does not exist (not even `sock_sim.pt`). `detect_real.launch.py` exists and pins `sim:=false` with `input_image_topic` default `/stereo_camera/left/image_raw`, but `model_path_real` defaults empty. The detector **silently no-ops** without a model (`sock_detector_node.py:130-141` logs a warning and starts without inference) → zero detections → pipeline stalls.
- **Needed:** Capture/label real IMX219-83 imagery (`capture_frames.py`/`prepare_dataset.py` scaffolding exists; `datasets/detection/` has only `classes.txt`), train YOLO, place `sock_real.pt` on the Jetson, pass `model_path_real`. Sim model won't transfer.

#### 🟠 ultralytics/torch not present in pixi env; no TensorRT backend — *weeks (gated by model + Jetson torch)*
- **Now:** **Verified:** `pixi.toml`/`pixi.lock` have no `ultralytics`, no `torch` (only `libopenvino-pytorch-frontend`). `UltralyticsBackend` defers the import and raises if missing; `tensorrt` backend is `NotImplementedError` (`backends.py:126-128`).
- **Needed:** Install a Jetson-compatible torch (NVIDIA aarch64/JetPack wheel — non-trivial in RoboStack) + ultralytics; ideally a TensorRT engine for real-time inference on the Orin.

#### 🟠 CSI stereo frame delivery under system ROS2 (NVMM/GStreamer) — *hours*
- **Now:** **Partially exists.** `stereo_camera.launch.py` (real) wires the calibration URLs and the node defaults `input_source="csi"` (`stereo_camera_node.cpp:320`) → opens `nvarguscamerasrc` via `cv::CAP_GSTREAMER`. **`unified.launch.py` already includes this real launch.** The catch (per memory `project_csi_camera_needs_system_ros`): **pixi/RoboStack OpenCV has no GStreamer binding**, so the node can only run under system ROS2 (`/opt/ros/humble`). The runbook's "software decode inside pixi" mode is a documentation error — `GST_PLUGIN_PATH` won't help when the OpenCV *binding* is absent.
- **Needed:** Run `stereo_camera_node` under system ROS2; verify both IMX219 sensors open and publish `/stereo_camera/disparity` + `left/camera_info` at 640×360, `use_sim_time:=false`. Reconcile the system-ROS2 (camera) vs pixi (rest) process split.

#### 🟠 Real disparity/depth accuracy unvalidated end-to-end — *days*
- **Now:** `reproject_roi()` (`sock_reproject.hpp:54-62`) uses `disparity.f`, `disparity.t`, `cx/cy`. Real calibration exists (baseline 0.06267 m, fx≈631, dated 2025-09-21) and is loaded by the real launch. Never run on real textured socks at 0.3–1.5 m. **Trap:** if the camera node is run via bare `ros2 run` (the system-ROS2 workaround) instead of the launch file, the calibration URLs stay unexpanded and it falls back to default focal_length 300/baseline 0.06 → metrically wrong depth.
- **Needed:** Verify valid disparity (server logs valid-pixel counts) and metrically correct centroids; tune `min_points`/cluster/ground params for real noise.

#### 🟡 GPU_BM stereo requires CUDA OpenCV — node hard-crashes otherwise — *hours*
- **Now:** `stereo_camera_config.yaml:33` sets `GPU_BM`, `use_hardware_acceleration` true. **Worse than "fallback":** the BM path has **no auto GPU→CPU fallback** — `GPUStereoStrategy::initialize()` failing throws `std::runtime_error` and the node won't start. The only escape is the explicit `use_hardware_acceleration:=false` override (which sim sets, the real config does not).
- **Needed:** Either ensure the Orin's OpenCV is CUDA-built, or set `algorithm: CPU_BM`/`use_hardware_acceleration:false`. One YAML edit or a build decision.

#### 🟡 No real-robot top-level perception+detect+grasp orchestration — *days*
- Folds into §3.2's hardware pick launch; `detect_real.launch.py` exists but nothing chains it with the CSI camera + segmentation/grasp nodes + lifecycle automation.

---

### 3.4 Manipulation / Grasp Pipeline

#### 🔴 Real `odom→base_link` TF absent — approach servo + remembered-pose grasp fail — *weeks*
- **Now:** `base_approach_node.py:151` `stable_frame='odom'`; snapshots target once into odom (`:209-215`), re-TFs `odom→base_link` each tick. `mobile_grasp_coordinator.py:91` `world_frame='odom'`, stores grasp pose in odom and recovers after driving. In sim this TF is Gazebo's; on hardware it does not exist (see §3.1 base-odom blocker). `_snapshot_target` explicitly warns a base-frame-only target will **not** converge.
- **Needed:** Same odom source as §3.1. Hard dependency for the entire approach + grasp-recovery sequence.

#### 🟠 Preset grasp joint angles are sim-tuned — *days (gated by serial driver)*
- **Now:** `_SRDF_STATES` `grasp_reach S2=1.8326` (105°) etc. tuned in RViz against the **sim** floor (`grasp_server.py:331-351`; ≥107° jams the gripper into the floor). **Live inconsistency surfaced:** the node's `grasp_reach S2=105°` diverges from `jetank.srdf`'s `100°` (`:76`) despite a comment claiming they mirror — the node's value is authoritative for launched grasps, the SRDF for RViz buttons. *(The earlier "yaml carries pre-tuning values" claim is stale — `grasp_poses.yaml` already matches the speed-optimized node defaults.)*
- **Needed:** Re-tune presets on the real arm (Feetech zero offsets, real link geometry/floor); reconcile the 100°-vs-105° SRDF/node drift. Cannot be done until the serial driver exists.

#### 🟠 Gripper effort/width tuning + open-loop grasp has no pick verification — *weeks (gated by serial driver)*
- **Now:** Gripper commanded at 0.04 open / 0.0 close, max_effort 5.0 N. **The success path is open-loop and unverified:** `command_gripper`'s return is never checked at call sites (`grasp_server.py:643,654,760,771`), `result.success` is set unconditionally after motion (`:669`), and the received stall/width feedback (`:557-563`) is only logged. A silent no-grip reports success. Stall feedback the verification would rely on requires the (missing) serial driver reading Feetech `present_load`/`present_position`.
- **Needed:** Real current/torque-limited close; close the open-loop gate with a gripper-stall-width or re-detection check. Re-detection is hard (fixed S1-mounted camera loses the sock at close range), so verification leans on gripper feedback → loops back to the serial driver.

---

### 3.5 URDF Description + Gazebo Sim Assets

#### 🟠 Inertial / joint-limit / contact values are sim placeholders — *days*
- **Now:** Masses/inertias hand-set (chassis 0.8 kg, arm links 0.01–0.05 kg). Effort limits placeholder (arm 1.0, S5 0.3, gripper raised to 5.0 specifically so the **sim** controller wouldn't stall). Gripper/wheel kp/kd/mu tuned for Fortress. Position limits **do** exist and are consistent across `arm.xacro` and `ros2_control.xacro` command interfaces; MoveIt inherits position limits from the URDF (`joint_limits.yaml` sets only velocity/accel) — making the URDF position limits load-bearing.
- **Needed:** Calibrate position limits to real Feetech travel, effort limits to real torque, verify collision boxes vs the real arm. Gated behind the missing serial driver (you can't validate motion without it).

#### 🟠 Perception sensors are Gazebo-only in the URDF (drivers exist elsewhere) — *days (camera only, for pick)*
- **Now:** `camera.xacro`/`lidar.xacro`/`imu.xacro` define sensors inside `<gazebo>` tags only. **But for pick-and-place only the camera matters** — the grasp pipeline subscribes to no `/scan` or `/imu` (verified: no LaserScan/Imu subs in `jetank_manipulation`). The real camera path (`JetsonCSICamera`, calibration, dual-backend node) already exists (§3.3). Real lidar (`rplidar.launch.py`) and IMU (`icm20948_node.cpp`) drivers exist for SLAM/Nav2, a separate subsystem.
- **Needed:** Wire the real camera into the hardware pick launch (system ROS2). Lidar/IMU are Nav2 concerns, not pick blockers.

#### 🟡 `use_sim`/`serial_port`/`baud_rate` not parameterized end-to-end — *hours (cosmetic until serial driver exists)*
- **Now:** xacro cleanly branches on `use_sim`, but every launch passes `use_sim:'true'` literally and `serial_port`/`baud_rate` are hardcoded constants.
- **Needed:** Parameterize and thread a `use_sim`/`hardware` arg through the hardware launch. Trivial once the driver and the §3.1 `hardware` arg exist.

---

## 4. Critical Path (ordered, with dependencies)

Blockers that must be cleared, in dependency order. Items at the same depth can proceed in parallel.

```
[0] Bench safety + device perms  (hours, BLOCKING, do first)
     └─ usermod -aG i2c,gpio,dialout; confirm /dev/{i2c-7,ttyTHS1,ttyUSB0} open;
        rehearse kill path (only failsafe = 1.0s watchdog, robot_controller.cpp:119);
        first bringup wheels-off-ground.

[1] JetankSerialHardware SystemInterface   (weeks)  ◄── THE blocker
     ├─ Feetech serial driver over /dev/ttyTHS1 (IDs 1/2/3/5 + gripper 4)
     ├─ per-servo zero-offset + direction-sign calibration  (days, needs [1])
     └─ CMakeLists library + PLUGINLIB_EXPORT_CLASS + plugin description

[1b] Fix CM 2.54 'joints empty' on standalone controller_manager  (days)
      └─ parallel with [1]; both required before arm executes on the
         standalone (non-Gazebo) manager.

[2] Real odom source: odom→base_link  (weeks; open-loop integ. is the fast path, days)
     └─ independent of [1]; required by base_approach_node + coordinator.

[3] Real sock model sock_real.pt  (weeks)
     ├─ requires CSI camera emitting frames under system ROS2  (hours)
     └─ requires Jetson torch+ultralytics in the detector env  (days)

[4] cmd_vel wiring  (hours)
     └─ base_approach cmd_vel_topic:=/cmd_vel_manip + cmd_vel_bridge HW config.
        Independent; can be done anytime.

[5] hardware 'hardware' xacro arg + mock branch  (hours)
     └─ also closes the mock-runs-real-motors safety hazard.

[6] Hardware pick launch  (days)   ◄── integrates [1][2][3][4]
     ├─ unified.launch.py hardware:=serial + detect_real + 4 pipeline nodes
     │   with use_sim_time:=false + lifecycle configure/activate
     └─ (optional) restore jetank_mission for the full navigate→deposit mission (days)

[7] On-robot tuning  (days)
     └─ grasp presets, gripper width/effort, depth validation, stagger timers,
        motor alpha/beta, post-grasp verification.
```

**Earliest a single pick could run on hardware:** after [0]+[1]+[1b]+[2]+[3]+[4]+[6] — i.e. the two multi-week items ([1] serial driver, [3] model) plus odom ([2]) are the schedule drivers. Everything else is hours/days.

---

## 5. Reconciliation with the Team's Existing Plans

| Item | Plan status | Reality (verified) |
|---|---|---|
| Full sim pick-and-place (fetch-sock) | DONE, sim-verified end-to-end | Confirmed — FSM, segment→approach→grasp all exercised in Gazebo |
| `JetankSerialHardware` plugin | `arm-moveit-grasp-plan` Phase D = **written spec only** | Confirmed stub; no C++. Single biggest blocker |
| Per-servo zero/direction calibration | Plan D2 (jog utility) — **not written** | No calibration values, no utility (only camera calib exists) |
| cmd_vel arbiter (`cmd_vel_bridge`) | Phase 1 **done**: HW params + 38 unit tests, off-robot | Confirmed implemented + tested; **never wired into a HW pick launch** |
| `web_mission_hw.launch.py` / `jetank_mission` | "delivered off-robot" per plans | **Package absent from workspace** — newly load-bearing surfaced gap |
| `sock_real.pt` | Phase 4 — not done | Confirmed absent; `/home/koen/models/` doesn't exist |
| Camera intrinsics/baseline calibration | runbook 8.2 / Phase 3 "missing" | **ALREADY DONE** — real yaml files dated 2025-09-21, wired into `stereo_camera.launch.py`. Plan is stale; only on-robot depth *verification* remains |
| CSI/GStreamer frame delivery | Phase 1 gate "unverified" | Confirmed unverified; runbook's "pixi software-decode" mode is a doc error (no GStreamer binding in pixi OpenCV) |
| Motor alpha/beta calibration | runbook 8.1 — identity values | Confirmed `motor_params.yaml` all 1.0/0.0; transform is wired (`motor.cpp:66`) |
| AMCL/Nav2 tuning + real map | Phase 5 — not done | Confirmed sim-tuned; SLAM tooling exists, no saved map. (Nav2 leg, not core pick) |
| CM 2.54 'joints empty' | MEMORY blocker, open | Confirmed in `moveit_bringup.launch.py:85-90`; affects standalone/HW path only |
| 4-DOF Cartesian floor grasp infeasible | Known; PICK uses preset reach | Confirmed; camera is on **S1** (shoulder), not the wrist — collision concern over-stated |

**Newly surfaced vs the plans:** (a) the `jetank_mission` package is entirely absent, not just "off-robot"; (b) the camera calibration is already done (plan over-states it as missing); (c) the SRDF/node `grasp_reach` 100°-vs-105° drift; (d) the open-loop grasp reports success unconditionally (silent no-grip); (e) GPU_BM hard-crashes (no auto CPU fallback); (f) `hardware:=mock` silently engages real motors (safety hazard).

---

## 6. Non-Issues / Already Handled

- **Camera calibration (intrinsics + baseline).** Real, non-placeholder yaml exists (`left/right_camera.yaml`, `stereo_calibration.yaml`, baseline 0.06267 m, dated 2025-09-21, original `ost.txt` present) and is loaded by `stereo_camera.launch.py:154-156` + `stereo_camera_node.cpp`. Only an on-robot depth sanity-check remains. *(Verdict downgraded blocker→low.)*
- **CM 2.54 bug does NOT block the sim pick workflow.** The sim path runs through `gz_ros2_control` and executes trajectories; the 2.54 failure is confined to the standalone `ros2_control_node`. *(Mis-stated as "broken even in sim".)*
- **Sim world is NOT empty.** `jetank_ros_main/worlds/sock_arena.sdf` contains a furnished room + six colored sock models (13 `sock` refs). The grasp pipeline has been validated against sim sock geometry. *(Mis-stated as "empty ground plane".)*
- **Gripper mimic exclusion on hardware** is already handled by construction — the mimic relay/controller are spawned only by the Gazebo launches; the single `gripper_controller` matching `grasp_server` already exists.
- **`gripper_mimic_relay.py` is correctly sim-only** by design (docstring says do not run on hardware) — no change needed.
- **CSI camera launch + dual-backend node already exist** (`stereo_camera.launch.py`, `JetsonCSICamera`); `unified.launch.py` already includes the real stereo launch. The gap is the system-ROS2 runtime split, not authoring the launch.
- **`detect_real.launch.py` already exists** with real-model + real-topic wiring; the gap is chaining it, not writing it.
- **`cmd_vel_bridge` is implemented and unit-tested** (38 tests) with full HW param surface; the gap is launch wiring only.
- **Camera-on-arm self-collision** is largely a non-issue for PICK: camera is on S1 (near base), PICK uses a named joint preset (no Cartesian floor IK), and the cited `S5_link↔camera_link` disable pair doesn't even exist in the SRDF. *(Verdict downgraded medium→low.)*
- **MoveIt controller action namespaces** (`/arm_controller/follow_joint_trajectory`, `/gripper_controller/gripper_cmd`) already match between `moveit_controllers.yaml` and `jetank_controllers.yaml` — no work needed once controllers spawn.
- **Thermal soak** — genuinely a Phase 7 robustness item, not a first-run blocker.

---

## 7. Recommended Next 3 Concrete Actions

1. **Start the `JetankSerialHardware` C++ SystemInterface now — it is the long pole.** Create `jetank_motor_control/hardware/{include,src}`, implement `on_init`/`export_*_interfaces`/`read`/`write` against the Feetech ST/SC protocol on `/dev/ttyTHS1` for IDs 1/2/3/5 + gripper ID 4, add the shared-library target + `PLUGINLIB_EXPORT_CLASS` + `pluginlib_export_plugin_description_file(hardware_interface jetank_hardware.xml)` in `CMakeLists.txt`. Reuse the `hardware-test` MCP's verified `feetech_servo_*` protocol where possible. **In parallel**, root-cause the CM 2.54 "joints empty" param regression in `moveit_bringup.launch.py`, since the arm cannot execute on the standalone manager until both are fixed.

2. **Unblock perception independently** (no dependency on the arm): run `stereo_camera_node` under system ROS2 (`/opt/ros/humble`, via the launch file so calibration URLs expand — avoid bare `ros2 run`) and confirm `/stereo_camera/disparity` + `left/camera_info` emit at 640×360; flip `stereo.algorithm` to `CPU_BM` (or `use_hardware_acceleration:false`) unless the Orin's OpenCV is CUDA-built; then begin real sock dataset capture with `capture_frames.py` toward `sock_real.pt`. These are the items that, with the serial driver, gate the first hardware pick.

3. **Land the cheap, high-leverage launch/wiring fixes** while [1]/[2] proceed: (a) add the `hardware` xacro arg + `mock_components/GenericSystem` branch in `jetank_ros2_control.urdf.xacro`/`ros2_control.xacro` — this also **closes the safety hazard** where `hardware:=mock` drives real motors; (b) wire `base_approach_node cmd_vel_topic:=/cmd_vel_manip` + the `cmd_vel_bridge` HW config so base motion reaches `/cmd_vel`; (c) provide an open-loop `odom→base_link` from `robot_controller.cpp` (command-integrated, the days-scale path) to unblock bench testing of the approach servo before encoders arrive. Then draft the hardware pick launch from the spec in `docs/hardware-bringup-fetch-sock.md:355-375`.

---

*Key files cited:* `jetank_motor_control/{jetank_hardware.xml, CMakeLists.txt, config/ros2_control.xacro, config/jetank_controllers.yaml, src/motor/{robot_controller.cpp, motor.cpp}}` · `jetank_description/urdf/jetank_ros2_control.urdf.xacro` · `jetank_moveit_config/launch/moveit_bringup.launch.py` · `jetank_ros_main/launch/{mobile_grasp.launch.py, unified.launch.py}` · `jetank_manipulation/jetank_manipulation/{base_approach_node.py, mobile_grasp_coordinator.py, grasp_server.py}` · `jetank_perception/{launch/stereo_camera.launch.py, src/stereo_camera_node.cpp, config/calibration/*}` · `jetank_detection/launch/detect_real.launch.py` · `jetank_web_control/jetank_web_control/cmd_vel_bridge.py` · `jetank_ros_main/worlds/sock_arena.sdf` · `jetank_ros_main/docs/hardware-bringup-fetch-sock.md`.
