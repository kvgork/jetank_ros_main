# JeTank efficiency audit — digest (2026-09-26)

Full report: ros2_ws/plans/efficiency-audit-2026-09-26.md (5.8k lines). Nothing applied yet.

## Run conditions

- Baseline build: `pixi run build` exit 0, 10 packages, 2 min 3 s.
- Live stack used for measurement: system-ROS stereo camera (`/stereo_camera/stereo_camera_node`, install_sys binary), `robot_controller`, `icm20948_imu`, `web_control_node`, `move_group` (+ robot_state_publisher, joint_state_publisher, three stalled controller spawners).
- NOT live: rplidar (not connected, no /dev/ttyUSB0), ros2_control / servo bus (no ping response, ros2_control_node exited -6), detection (no sock_detector), simulation (no Gazebo/ros_gz).
- Scope: user chose to re-report everything; no July-2026-audit exclusions were applied.
- Inputs: 384 kept findings from 12 sources, 3 refuted findings. After de-duplication (same file:line reported by two sources) 359 findings are listed here; 25 were folded into their twin and are named in the twin's paragraph.
- Severity shown is the verifier-adjusted severity; where it differs from the auditor's original the original is shown in parentheses. Verdicts: `confirmed` (verifier re-checked), **UNVERIFIED** (low severity, not sent to verifier), **UNCERTAIN** (verifier could not confirm the premise).
- Totals by adjusted severity: high 7, medium 42, low 310.

## Top findings

Ranking: adjusted severity (high > medium > low), then measured evidence before static (est_gain is qualitative text, so evidence quality is used as the gain tie-breaker), then lower effort first, then id.

| rank | id | package | lens | title | file:line | severity | evidence | effort | verdict |
|---|---|---|---|---|---|---|---|---|---|
| 1 | jetank_perception-01 | jetank_perception | runtime | Disparity computed every frame even with no disparity/pointcloud/diagnostic subscriber | `src/jetank_perception/src/stereo_camera_node.cpp:1179` | high | measured | S | confirmed |
| 2 | jetank_perception-05 | jetank_perception | runtime | DisparityImage published RELIABLE depth-1 (~0.9 MB/msg) delivers at 0.6 Hz to subscribers | `src/jetank_perception/src/stereo_camera_node.cpp:741` | high | measured | S | confirmed |
| 3 | jetank_ros_main-01 | jetank_ros_main | runtime | Static world->base_footprint TF gives base_footprint two parents (odom + world) | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:203` | high | measured | S | confirmed |
| 4 | footprint-01 | workspace | footprint | Gazebo/Ignition sim stack installed unconditionally on the Jetson (175 MB compressed, 25 pkgs) | `/home/koen/workspaces/ros2_ws/pixi.toml:111` | high | measured | M | confirmed |
| 5 | jetank_perception-22 | jetank_perception | minimality | yaml-cpp loader parses the same files CameraInfoManager already parsed, reads only K/D, and the calibrated R/P are discarded | `src/jetank_perception/src/stereo_camera_node.cpp:1520` | high | measured | M | confirmed |
| 6 | jetank_web_control-01 | jetank_web_control | runtime | Camera subscription is always active: node deserializes ~30 fps of JPEG frames with zero viewers | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:493` | high | measured | M | confirmed |
| 7 | jetank_perception-02 | jetank_perception | runtime | Rectification remap runs on 3-channel BGR, then converted to gray; rect images mislabeled mono8 | `src/jetank_perception/src/stereo_camera_node.cpp:1058` | high | static | S | confirmed |
| 8 | footprint-02 | workspace | footprint | ros-humble-desktop metapackage pulls demos, turtlesim, rqt, tutorials never used | `/home/koen/workspaces/ros2_ws/pixi.toml:73` | medium | measured | S | confirmed |
| 9 | footprint-09 | workspace | minimality | Runtime deps reach the env only transitively; pixi.toml under-declares what src/ imports | `/home/koen/workspaces/ros2_ws/pixi.toml:127` | medium | measured | S | confirmed |
| 10 | footprint-28 | jetank_web_control | minimality | exec_depend on jetank_mission (absent from workspace) while aiohttp/PIL/numpy/cv2 go undeclared | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:20` | medium | measured | S | confirmed |
| 11 | jetank_moveit_config-01 | jetank_moveit_config | runtime | Controller spawners idle forever when controller_manager never appears | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:120` | medium | measured | S | confirmed |
| 12 | jetank_navigation-01 | jetank_navigation | runtime | IMU node reads I2C and publishes 3 topics at 100 Hz with zero subscribers (no lazy publishing) | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:260` | medium | measured | S | confirmed |
| 13 | jetank_perception-03 | jetank_perception | runtime | appsink has no drop/max-buffers; CAP_PROP_BUFFERSIZE is a no-op for the GStreamer backend, so frames queue and add latency | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:329` | medium (orig high) | measured | S | confirmed |
| 14 | jetank_perception-07 | jetank_perception | runtime | Per-frame get_parameter() in publish_disparity_image; f/t/delta_d inconsistent with calibration | `src/jetank_perception/src/stereo_camera_node.cpp:1326` | medium | measured | S | confirmed |
| 15 | jetank_perception-15 | jetank_perception | runtime | OpenCV worker thread pool unbounded; 5 internal threads each 13-47% CPU; camera.processing_threads never applied | `src/jetank_perception/src/stereo_camera_node.cpp:310` | medium | measured | S | confirmed |


## Refuted
- **jetank_perception-35** — Whole-PCL find_package and ${PCL_LIBRARIES} linked into every target and test: The runtime claim does not hold: the pixi/RoboStack toolchain links with -Wl,--as-needed (.pixi/envs/default/etc/conda/activate.d/activate-gcc_linux-aarch64.sh LDFLAGS), so unused PCL modules from ${PCL_LIBRARIES} (CMakeLists.txt:35, 149, 162) are already dropped from NEEDED — the findin
- **jetank_motor_control-28** — ign_ros2_control exec_depend pulls the Gazebo stack onto the hardware robot: The cited lines do not exist: jetank_motor_control/package.xml is 63 lines and the ign_ros2_control exec_depend is at line 47 (with a comment at 43-46 explaining the xacro use). More importantly the claimed gain is implausible: this workspace is pixi-managed with a single default environm
- **footprint-13** — ${OpenCV_LIBS} (all modules) linked twice into camera_node and stereo_camera_node: The fix is in-scope (no ament_export_* in jetank_perception/CMakeLists.txt, no downstream package links it, strategy/factory untouched) but its claimed gain does not hold: the pixi toolchain links with `-Wl,--as-needed` (pixi run printenv LDFLAGS), so unused libopencv_*/openvino libs are alrea


## Findings per package (adjusted severity)

| package | high | medium | low | total |
|---|---|---|---|---|
| jetank_perception | 4 | 10 | 26 | 40 |
| jetank_manipulation | 0 | 1 | 32 | 33 |
| jetank_web_control | 1 | 2 | 30 | 33 |
| jetank_ros_main | 1 | 5 | 27 | 33 |
| jetank_motor_control | 0 | 3 | 33 | 36 |
| jetank_detection | 0 | 4 | 24 | 28 |
| jetank_navigation | 0 | 3 | 25 | 28 |
| jetank_simulation | 0 | 3 | 23 | 26 |
| jetank_moveit_config | 0 | 1 | 19 | 20 |
| jetank_description | 0 | 0 | 30 | 30 |
| cross-duplication | 0 | 5 | 26 | 31 |
| workspace-footprint | 1 | 5 | 15 | 21 |
| **total** | 7 | 42 | 310 | 359 |

## Addendum 2026-09-27 — RPLidar measured

The lidar was not enumerated during the 2026-09-26 run but is plugged in via USB (CP2102N bridge, `/dev/ttyUSB0`, appeared 17:27 that day). Measured with `lidar.launch.py` alone (rplidar_c1m1.yaml: Standard mode, 460800 baud, angle_compensate on):

| metric | value |
|---|---|
| /scan rate | 10.0 Hz, 100 msgs / 10 s, 0 drops |
| /scan bandwidth | 182 KB/s |
| latency (header.stamp) | p50 101 ms, p99 102 ms (one scan period; stamp at scan start) |
| jitter | 5.3 ms |
| rplidar_node CPU | mean 8.6 %, p95 19.8 % |
| rplidar_node RSS | 26 MB, 12 threads, 19 fds |

No efficiency finding on the lidar node itself. Still open: the six Nav2/SLAM follow-ups (jetank_navigation-14/15/16/17/18/28) need the navigation stack running with this lidar.

## Measurement follow-up passes (2026-09-27)

| pass | stack | measured | blocked | new findings |
|---|---|---|---|---|
| 1 | Nav2 navigation-only + slam_toolbox, real lidar | 5 | 2 (AMCL: no map; RViz: no display) | navigation-29: `vth_samples` typo in nav2_params.yaml silently ignored (medium) |
| 2 | MoveIt + manipulation nodes on mock servos (bus still silent) | 9 | 6 (4 serial-bus, 2 demo/RViz) | manipulation-33: Python TF listeners idle at ~13% CPU each (medium); manipulation-34: grasp_server waits 10 s/goal for a gripper action it never finds (medium) |
| 3 | system-ROS camera + sock_segmentation_server | 2 | 1 (transform stage not reached) | perception-42: disparity period 1.6 s > server max_age 1.0 s (medium) |

Idle stack costs measured: Nav2+SLAM 16.8% CPU / 362 MB across 9 processes; arm stack 32.6% CPU / 332 MB across 5 (two Python TF listeners = 27 of the 32.6). Largest idle topic: /local_costmap/voxel_grid at 506 KB/s to zero subscribers.

42 follow-ups stay static with a verified blocker each (12 detector: no ultralytics/model; 14 Gazebo: pass declined; 4 servo bus silent; 3 RViz; 3 browser; rest listed in the report). Corrections: velocity_smoother is load-bearing (navigation-17 narrowed); motor_control-16/17 line anchors are stale; perception-41 worst case is 0.8 s not 0.4 s.

## Applied 2026-09-27

All 7 high + 42/46 medium findings applied on `chore/efficiency-fixes` (74 commits, 9 repos, unmerged). Stereo node CPU 190% → 47%; TF-listener nodes 13-14% → 0.4%; grasp goal 19.5 s → 13.7 s (server-side 9.6 s); web control no camera subscription with zero viewers; pixi env 5.9 → 5.2 GB. Still open: disparity delivery 0.6 Hz (perception-05), point cloud rate vs new density, 3 serial-bus findings, cross-07 track width. Details: post-fix addendum in the full report.

## Correction 2026-09-27 — disparity and point-cloud rates were measurement artifacts

A dedicated rclpy probe (reliable and best-effort, from both the system-ROS and the pixi environment) measured the real delivery on the post-fix stack:

| topic | ros2-mcp tool reported | real (probe) |
|---|---|---|
| /stereo_camera/disparity | 0.60–0.63 Hz, ~330 ms | 30.1 Hz, p50 latency 16–17 ms, reliable and best-effort, both environments |
| /stereo_camera/points | 4.9 Hz (27.8 Hz pre-fix) | 30.0 Hz, ~2,600 points per cloud, p50 latency 24 ms |
| stereo_camera_node CPU | 47 % "while subscribed" | 40 % idle, 120 % while serving disparity + points at 30 Hz (/proc sampling) |

Consequences: **jetank_perception-05** (disparity 0.6 Hz) was never a real defect — the only evidence was the ros2-mcp `measure_topic_perf` / `get_topic_hz` tools, which under-report large messages. Its KeepLast(5) change is harmless and stays. **jetank_perception-42** (disparity period > segmentation max_age) rests on the same false premise; its change is harmless. The "point cloud rate dropped / 9x denser, retune filters" item is withdrawn. The 190 % → 47 % "subscribed" figure is replaced by 190 % → 120 % at a real 30 Hz; the idle saving (≈190 % → 40 %) stands. Note: `ros2 topic hz` / `bw` on this Jetson produced no output for these topics within 25 s (sensor-data QoS CLI subscriber), so neither CLI nor MCP rates can be trusted for multi-MB messages here — use a dedicated probe.
