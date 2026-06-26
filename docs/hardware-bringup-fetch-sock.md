# Hardware Bringup and Fetch-Sock Mission Runbook

**Target:** Jetson Orin Nano Super running JetPack, pixi workspace at `~/workspaces/jetank`
**Goal:** Bring up the full fetch-sock mission stack on real hardware, with per-subsystem verification before issuing any goal.
**Date authored:** 2026-06-26
**Context:** The mission is sim-validated (sim2real-hardware-mission Phase 1-2). This runbook covers what the USER does on the Jetson.

---

## Section 1 — Safety First: First Real Motion Protocol

**Read this entire section before powering anything.**

The sim-validated mission's first run on real motors and real servos is the highest-risk moment. Treat it as a new system.

### Physical prerequisites before any launch

1. **Wheels off the ground.** Place the robot on a stand, blocks, or flip it. The motor driver (`robot_controller.cpp`) and the Nav2 stack will both send wheel commands the moment the controllers activate. There is no software dry-run.
2. **Arm clear of obstacles.** The arm (S1–S3 + gripper, `/dev/ttyTHS1`) activates when `moveit_bringup.launch.py` loads with `hardware:=serial`. Do not hold the arm; do not rest anything on or near it.
3. **Hand near the power switch.** Keep a finger on the physical power cut. Do not rely solely on software.

### How the motor watchdog works

`robot_controller.cpp:119` calls `stop()` if no `/cmd_vel` message arrives within **1.0 seconds**. That is the only base-motion failsafe in software. Consequences:

- If a node flooding `/cmd_vel` (e.g. a runaway Nav2 planner) gets stuck, publishing zero yourself (`ros2 topic pub /cmd_vel ...`) will NOT override it — the flooding node wins the race. The only reliable stop is **kill the launch process** (`Ctrl-C` in the terminal running the launch) or **cut power**.
- After a node flood the motors coast for up to 1 second before the watchdog fires.
- `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once` is useful for a quiescent system but is not an e-stop.

### Kill story

```bash
# In the launch terminal:
Ctrl-C          # kills all nodes; motor watchdog fires within 1 s

# If the launch is backgrounded or in a remote session:
pkill -f "ros2 launch"      # sends SIGTERM to all launch children

# Power cut: toggle the robot's main power switch
# (always available; use it if software kill doesn't stop motion)
```

**Do not use `ros2 lifecycle set` or `ros2 service call` to stop individual nodes as your primary e-stop** — there is no time-critical path through those.

---

## Section 2 — Device Permissions

All of these must be readable by the user running `pixi shell` **before** launching.

### I2C buses

The motor driver opens `/dev/i2c-7` (PCA9685 at address `0x60`, hardcoded in `motor.cpp:105`). The IMU driver (`icm20948_node`) opens `/dev/i2c-1` (ICM-20948 at `0x68`, set in `jetank_navigation/config/icm20948.yaml`). Note: the YAML comment mentions pins 3/5 = i2c-7, but the confirmed working `i2c_bus` value in the YAML is `1`.

```bash
sudo usermod -aG i2c $USER
# Log out and back in (or use 'newgrp i2c' in the current session)

# Verify:
ls -la /dev/i2c-7   # motor PCA9685
ls -la /dev/i2c-1   # IMU

# Confirm devices are present:
i2cdetect -y -r 7   # expect 0x60 (PCA9685)
i2cdetect -y -r 1   # expect 0x68 (ICM-20948)
```

### GPIO

`libgpiod` is in the pixi env. The user needs read access to `/dev/gpiochip*`:

```bash
sudo usermod -aG gpio $USER
# or set udev rule:
echo 'SUBSYSTEM=="gpio*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify:
ls -la /dev/gpiochip*
```

### Arm servos — UART

The `JetankSerialHardware` plugin opens `/dev/ttyTHS1` at **1000000 baud** (`ros2_control.xacro:31–32`).

```bash
sudo usermod -aG dialout $USER   # covers ttyTHS*

# Verify:
ls -la /dev/ttyTHS1
```

### RPLidar — USB

The lidar driver opens `/dev/ttyUSB0` at 460800 baud (`rplidar_c1m1.yaml:24`). Optionally add a persistent udev symlink:

```bash
sudo usermod -aG dialout $USER  # same group as above

# Optional symlink (from rplidar_c1m1.yaml instructions):
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"' \
  | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify:
ls -la /dev/ttyUSB0
```

### Apply all at once

```bash
sudo usermod -aG i2c,gpio,dialout $USER
# Then log out and back in, or:
newgrp i2c   # only activates i2c in current shell; open a fresh shell for full effect
```

---

## Section 3 — Jetson CSI Camera and GStreamer

**Known issue (from CLAUDE.md):** The conda-forge GStreamer inside the pixi shell lacks JetPack's NVMM hardware codec plugins. CSI capture falls back to software decode inside pixi, which works but is slower and uses more CPU.

### Two operating modes

| Mode | How to use | When to use |
|---|---|---|
| Software decode (inside pixi) | `pixi shell` then launch normally | Development, acceptable frame rate at 640×360 30 fps |
| HW decode (NVMM, outside pixi) | Source ROS2 from the system install; run `stereo_camera_node` against host GStreamer | Production; eliminates CPU load; requires system ROS2 Humble |

For the hardware decode path outside pixi:
```bash
# Source system ROS2 and point GStreamer at JetPack plugins:
source /opt/ros/humble/setup.bash
export GST_PLUGIN_PATH=/opt/nvidia/gstreamer/lib/gstreamer-1.0:$GST_PLUGIN_PATH
ros2 run jetank_perception stereo_camera_node
```

### Verify camera produces frames (inside pixi shell)

```bash
pixi shell
source install/setup.bash   # auto-done on pixi shell entry

# Launch the stereo node (namespace is 'stereo_camera' per launch file default):
ros2 launch jetank_perception stereo_camera.launch.py

# In a second terminal (pixi shell):
ros2 topic hz /stereo_camera/left/image_raw    # expect ~30 Hz
ros2 topic hz /stereo_camera/right/image_raw   # expect ~30 Hz
ros2 topic hz /stereo_camera/disparity         # expect ~30 Hz
ros2 topic hz /stereo_camera/points            # expect ~30 Hz
```

### Frame ID check

The `sock_segmentation_server` reprojects the disparity into 3D; it must see an `_optical_frame` frame_id to get correct geometry. The default config ships `camera_left_link` / `camera_right_link` (non-optical). The hardware launcher (`web_mission_hw.launch.py`) passes:

```
frames.left_frame_id:=camera_left_optical_frame
frames.right_frame_id:=camera_right_optical_frame
```

Verify after launching:
```bash
ros2 topic echo /stereo_camera/disparity --no-arr | grep frame_id
# expected: frame_id: camera_left_optical_frame
```

If the frame_id is `camera_left_link` instead, the launcher argument did not propagate — check the `unified.launch.py` passthrough.

---

## Section 4 — Per-Subsystem Smoke Tests

Run these **before** issuing any mission goal. Each check must pass before proceeding.

### 4.1 RPLidar

```bash
# Check the lidar node is publishing (topic: /scan):
ros2 topic hz /scan
# Expect: ~10 Hz (5000 Hz sample rate, 10 Hz scan frequency per config)

ros2 topic echo /scan --no-arr | head -20
# Expect: header.frame_id: "laser"  (matches rplidar_c1m1.yaml frame_id)
# Expect: ranges array populated (non-empty; some inf values are normal for open space)

# Sanity: check range bounds match lidar spec (0.05 m to 12 m):
ros2 topic echo /scan --no-arr | grep -E "range_min|range_max"
```

### 4.2 Motor odometry

```bash
ros2 topic hz /odom
# Expect: >10 Hz while robot_controller is running

ros2 topic echo /odom --no-arr | head -30
# Expect: pose.pose.position.x/y change when robot moves; should be 0,0,0 at start
```

### 4.3 IMU

```bash
ros2 topic hz /imu/data_raw
# Expect: ~100 Hz (publish_rate: 100.0 in icm20948.yaml)

ros2 topic echo /imu/data_raw --no-arr | head -20
# Expect: header.frame_id: "imu_link"
# Expect: linear_acceleration.z ~ 9.8 (gravity)

# Magnetometer (optional check):
ros2 topic hz /imu/magnetic_field
```

### 4.4 Stereo camera

```bash
# Already checked in Section 3; quick recheck:
ros2 topic hz /stereo_camera/left/image_raw
ros2 topic hz /stereo_camera/disparity

# Confirm camera info is populated (needed for 3D reprojection):
ros2 topic echo /stereo_camera/left/camera_info --no-arr | head -10
# Expect: width and height non-zero; K matrix populated (non-zero after calibration)
```

### 4.5 Sock detections

```bash
ros2 topic hz /detections/socks
# Expect: messages flowing when camera sees a sock; silent when nothing in view is normal

# Echo a detection:
ros2 topic echo /detections/socks
# Expect: Detection2DArray with bbox and score fields
```

---

## Section 5 — Arm Actuation Check

**This is the most important check before running a mission.** The MoveIt mock backend (`hardware:=mock`) reports `FollowJointTrajectory` SUCCESS while no servo physically moves. With `hardware:=serial`, the `JetankSerialHardware` plugin sends real commands to `/dev/ttyTHS1` at 1 Mbaud.

### Check the hardware arg is correct

```bash
# After launching unified.launch.py with hardware:=serial, verify the plugin loaded:
ros2 control list_hardware_interfaces
# Expect: JetankSystem  ACTIVE
# Expect: interface names include S1_joint/position, S2_joint/position, etc.
# If you see "JetankMockSystem" or "IgnitionSystem" — the mock backend loaded.

ros2 control list_controllers
# Expect: arm_controller  joint_trajectory_controller  active
#         gripper_controller  gripper_action_controller  active
```

### Command a small safe move and watch the servo

With the arm clear of obstacles and at a known safe pose (e.g. home), command S1 (base rotation) by a small angle (0.1 rad):

```bash
# Open a pixi shell and source the overlay:
pixi shell

# Use ros2_control forward command or MoveIt python script.
# Quickest physical test: command arm_controller directly with a trajectory:
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["S1_joint"], points: [{positions: [0.1], time_from_start: {sec: 2}}]}}'

# WATCH: the S1 servo (arm base rotation) must physically rotate ~6 degrees.
# If you hear the servo activate and see motion -> hardware:=serial is confirmed working.
# If the action succeeds but nothing moves -> you are running hardware:=mock.
```

If nothing moves, stop and verify `ros2 control list_hardware_interfaces` shows `JetankSystem` (serial), not mock. Do NOT proceed to mission run with mock hardware — the arm will never pick anything.

**Return to home before continuing:**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["S1_joint", "S2_joint", "S3_joint"], points: [{positions: [0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}'
```

---

## Section 6 — cmd_vel Arbiter Verification

The hardware arbiter (`cmd_vel_bridge` node, from Phase 1) muxes:
- `/cmd_vel_teleop` (web teleop, lowest priority)
- `/cmd_vel_manip` (TwistStamped, from `base_approach_node` / `mission_coordinator`, highest priority)

Nav2 owns `/cmd_vel` directly during NAVIGATE (arbiter is silent then). The arbiter is configured with `nav_topic:=''` on hardware, so it never subscribes to `/cmd_vel`.

### (a) Teleop only — no active mission

```bash
# In terminal A: watch /cmd_vel
ros2 topic echo /cmd_vel

# In terminal B: send a teleop command via web UI or:
ros2 topic pub /cmd_vel_teleop geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.1}}' --rate 10

# Expect in terminal A: Twist messages with angular.z: 0.1
# Expect: only ONE publisher on /cmd_vel:
ros2 topic info /cmd_vel
```

### (b) Nav2 goal active — no mission or teleop

```bash
# Send a Nav2 goal via RViz or:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}'

# In a second terminal:
ros2 topic echo /cmd_vel
# Expect: Twist messages from Nav2's velocity_smoother

ros2 topic info /cmd_vel
# Expect: publisher count = 1 (Nav2 velocity_smoother only)
# The arbiter is silent (nav_topic:='' so it does not subscribe /cmd_vel)
```

### (c) APPROACH/PICK active — mission sending /cmd_vel_manip

```bash
# During an active mission in APPROACH phase:
ros2 topic echo /cmd_vel_manip     # see TwistStamped from base_approach_node
ros2 topic echo /cmd_vel           # see Twist forwarded by arbiter (highest priority)

ros2 topic info /cmd_vel
# Expect: publisher count = 1 (arbiter only; Nav2 is idle while mission controls base)
```

### Documented residual: teleop during active Nav2

The plan documents one known two-publisher overlap: if a user sends manual teleop (`/cmd_vel_teleop`) **while Nav2 is actively navigating**, both Nav2 velocity_smoother and the arbiter publish on `/cmd_vel` simultaneously. This is a momentary edge case (Nav2 completes and goes silent; arbiter goes idle on timeout). It is not guarded. During normal mission flow the FSM sequences NAVIGATE vs SEARCH/PICK so they are not concurrent.

---

## Section 7 — Launching the Full Hardware Mission

### Prerequisites checklist (all must be green)

- [ ] Robot on stand, wheels off ground (or confirmed safe driving area with nobody in front)
- [ ] Arm has ~0.3 m clearance in all directions
- [ ] All device permissions confirmed (Section 2)
- [ ] Camera produces frames + correct frame_id (Section 3)
- [ ] All subsystem smoke tests pass (Section 4)
- [ ] Arm actuation physically confirmed (Section 5)
- [ ] Map file exists at `<map>.yaml` + `<map>.pgm` (created from a prior SLAM session)
- [ ] Real sock model at `~/models/sock_real.pt` (see Section 8 — POINTERS)

### Launch command

```bash
cd ~/workspaces/jetank
pixi shell

ros2 launch jetank_mission web_mission_hw.launch.py \
  map:=/path/to/your_map.yaml \
  model_path_real:=~/models/sock_real.pt \
  confidence:=0.3
```

The launcher brings up (staggered with hardware-settle timers, all `use_sim_time:=false`):
1. `unified.launch.py` — URDF, motor controller, stereo camera (real), IMU, lidar, MoveIt2 (`hardware:=serial`), Nav2 (`navigation_mode:=nav2`).
2. Sock pipeline — `detect_real.launch.py`, `sock_segmentation_server`, `grasp_server`, `base_approach_node`, `mobile_grasp_coordinator`.
3. `mission_coordinator`.
4. Web control node + hardware arbiter (`cmd_vel_bridge` with `output_stamped:=false`, `nav_topic:=''`).

### Available launch arguments

```bash
ros2 launch jetank_mission web_mission_hw.launch.py --show-args
# map             (required) path to the .yaml map file
# model_path_real (default: ~/models/sock_real.pt) YOLO model for real camera
# confidence      (default: 0.3) detection confidence threshold
```

### AMCL initial pose

`nav2_params.yaml` sets `set_initial_pose: true` with the origin `(0, 0, 0, yaw=0)`. If the robot does not start at the map origin, set the initial pose in RViz ("2D Pose Estimate") after launch or override in nav2_params.yaml before launching.

### Monitoring during a run

```bash
# Robot controller status (motor health):
ros2 topic echo /robot_status

# Active mission state:
ros2 topic echo /mission_coordinator/status  # or check the web UI

# TF tree (verify map->odom->base_link chain is live):
ros2 run tf2_tools view_frames  # generates frames.pdf
# or spot-check:
ros2 run tf2_ros tf2_echo map base_link

# System resources on Jetson:
tegrastats --interval 1000   # in a separate terminal
```

---

## Section 8 — POINTERS: Large Standalone Efforts

**IMPORTANT: These are substantial independent projects, each taking hours to days. This plan (sim2real-hardware-mission) does NOT deliver them. They are flagged here so the user knows what remains before the mission can succeed reliably.**

### 8.1 Motor alpha/beta calibration

The left and right motors almost certainly drive at different speeds for the same command due to manufacturing variation. Uncalibrated, the robot will curve during straight-line Nav2 goals.

- **What to calibrate:** `left_motor_alpha`, `left_motor_beta`, `right_motor_alpha`, `right_motor_beta` in `src/jetank_ros_main/config/motor_params.yaml`.
- **Current values:** all alpha=1.0, beta=0.0 (identity — no calibration applied).
- **Method:** command a fixed velocity, measure actual distance traveled per side over a known distance. Adjust alpha (gain) until both sides match. Repeat for left/right symmetry.
- **Apply:** edit `motor_params.yaml` and rebuild (`pixi run build-motor`).

### 8.2 Stereo camera calibration (checkerboard)

Without calibration the stereo 3D point cloud will have large errors. The `sock_segmentation_server` uses the point cloud centroid to drive the base to approach distance — a badly calibrated cloud means wrong approach distance.

- **What to produce:** `src/jetank_perception/config/calibration/left_camera.yaml`, `right_camera.yaml`, `stereo_calibration.yaml`.
- **Method:** ROS2 camera_calibration package with a known-size checkerboard. Print a checkerboard (e.g. 8×6, 30 mm squares). Run `ros2 run camera_calibration cameracalibrator` against the stereo pair.
- **Baseline:** the IMX219-83 module has a 60 mm baseline (documented in stereo_camera.launch.py comments). Verify this during calibration.
- **Known risk:** the camera is mounted on the arm (`camera_link` is a child of `S1_link`). The IMU also rides the arm (per CLAUDE.md — this is by design, matching hardware). Calibrate with the arm in its parked/home pose; the calibration is valid only in that pose geometry.

### 8.3 Real sock model (sock_real.pt)

The sim mission uses a Gazebo sock model. The real camera sees real socks in different lighting, texture, and scale.

- **Path expected by the launcher:** `~/models/sock_real.pt` (default) or override via `model_path_real:=`.
- **Method:** collect 100+ images of real socks with the robot's stereo camera. Label with bounding boxes (CVAT, Robolab, LabelImg). Fine-tune YOLOv8/v11 on the dataset. Export to the format the `jetank_detection` backend expects (ultralytics `.pt`).
- **Dataset tooling:** `src/jetank_detection/` contains dataset-prep tooling — read its README.

### 8.4 AMCL and Nav2 parameter tuning for real hardware

Real odometry drifts (encoder slip on tracked base, uneven surface) and the RPLidar scan is noisier than simulated scan. The default `nav2_params.yaml` was tuned for sim.

- **File:** `src/jetank_navigation/config/nav2/nav2_params.yaml`.
- **Key parameters to tune:** `min_particles`/`max_particles` (AMCL), `update_min_d`/`update_min_a` (update thresholds), robot footprint (`robot_radius` or polygon), `inflation_radius`, `max_vel_x`/`min_vel_x`, costmap resolution.
- **Approach:** start with a conservative footprint and low velocities. Monitor `/amcl_pose` covariance. Increase particle count if localization loses track. Reduce inflation_radius only after confirming the robot doesn't clip obstacles.
- **Phase 2 note:** the plan comments in `nav2_params.yaml` note AMCL tuning is "staged in plan Phase 2" — those comments describe future work, not completed work.

### 8.5 Thermal soak validation

The Jetson Orin Nano has thermal throttling. The full stack (Nav2 + MoveIt + stereo GPU disparity + YOLO inference + web server) may push the SoC to throttle.

```bash
# Run during a 15-minute mission attempt:
tegrastats --interval 1000 | tee ~/tegrastats_$(date +%Y%m%d_%H%M%S).log

# Watch for:
# - CPU/GPU freq drops (throttling)
# - CPU temperature > 80°C
# - Memory pressure (RAM + swap usage)
```

If throttling occurs: add a heatsink/fan, reduce camera resolution (`camera.width: 320` in `stereo_camera_config.yaml`), or switch from `GPU_BM` to `CPU_BM` disparity (lower GPU load).

---

## Quick Reference: Device Map

| Device | Path | Address/Baud | Subsystem | Permission group |
|---|---|---|---|---|
| PCA9685 motor driver | `/dev/i2c-7` | `0x60` | Motor base | `i2c` |
| ICM-20948 IMU | `/dev/i2c-1` | `0x68` | Navigation IMU | `i2c` |
| Arm servos | `/dev/ttyTHS1` | `1000000` baud | MoveIt2 / arm | `dialout` |
| RPLidar C1M1 | `/dev/ttyUSB0` | `460800` baud | Nav2 lidar | `dialout` |
| CSI cameras | GStreamer pipeline | — | Stereo perception | (no special group) |

## Quick Reference: Key Topic Names

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 (NAVIGATE) or arbiter (SEARCH/PICK) |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Web teleop |
| `/cmd_vel_manip` | `geometry_msgs/TwistStamped` | `base_approach_node` / `mission_coordinator` |
| `/odom` | `nav_msgs/Odometry` | `robot_controller` |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar node |
| `/imu/data_raw` | `sensor_msgs/Imu` | `icm20948_node` |
| `/stereo_camera/left/image_raw` | `sensor_msgs/Image` | `stereo_camera_node` |
| `/stereo_camera/disparity` | `stereo_msgs/DisparityImage` | `stereo_camera_node` |
| `/stereo_camera/points` | `sensor_msgs/PointCloud2` | `stereo_camera_node` |
| `/detections/socks` | `vision_msgs/Detection2DArray` | `sock_detector_node` |
| `/robot_status` | `std_msgs/String` | `robot_controller` |
