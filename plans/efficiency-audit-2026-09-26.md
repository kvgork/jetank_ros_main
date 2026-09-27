# Workspace Efficiency Audit — 2026-09-26

Session: `20260926-090131-workspace-efficiency-review`  
Date: 2026-09-26  
Workspace: `/home/koen/workspaces/ros2_ws` (Jetson Orin Nano Super, pixi/RoboStack Humble)

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

## jetank_perception

Coverage: 29 files read; 15 measurements run. Notes: All source, header, test, CMake, package.xml, config, calibration, launch and script files in the package were read in full. docs/*.md (750 lines total) were only skimmed (first 30-60 lines each) since they are prose, not code. The .git directory, build/, install/, install_sys/, log/ and __pycache__ were not read per constraints (only `ldd`, `stat`, and `ls` were run against the installed binary to identify the running process's provenance and linked libraries). Important caveat for the verification stage: the live /stereo_camera/stereo_camera_node is an install_sys binary built 2026-06-01 from an older revision (it exposes pointcloud.passthrough_filter.* params that no longer exist in source and publishes frame_id camera_left_link although the current launch overrides to *_optical_frame), so live CPU/latency numbers characterise that older build; all line anchors refer to the current source. The ros2-mcp bandwidth tool's byte totals appear inflated (per-message sizes derived from them are implausible), so bw figures were used only as relative evidence. No per-stage timing (rectify vs BM vs cloud filters) could be measured read-only; those attributions are static. The disparity f/t mismatch (-07) and discarded calibrated rectification (-22) are correctness issues reported because they sit on the same code the efficiency fixes touch.

Measurements:
- mcp__ros2-mcp__get_node_list: /stereo_camera/stereo_camera_node present; no sock_segmentation_server or camera_node running (only the stereo node could be measured)
- mcp__ros2-mcp__get_topic_list + get_node_info /stereo_camera/stereo_camera_node: 14 publishers (image_raw/rect + compressed L/R, camera_info L/R, disparity, points), services incl. /stereo_camera/set_camera_info (from CameraInfoManager) plus left/right/set_camera_info and calibrate_stereo
- mcp__ros2-mcp__get_node_params /stereo_camera/stereo_camera_node: 100+ params; includes pointcloud.passthrough_filter.* which do not exist in current source (running binary is an older build, see coverage_notes); stereo.algorithm=GPU_BM, compression mode compressed_only for raw and rect, quality_monitoring.enable=false, statistical_filter k=30, voxel leaf 0.005
- mcp__ros2-mcp__profile_node /stereo_camera/stereo_camera_node 10 s x3: cpu mean 190.2% / 194.3% / 195.0% (p95 216 / 215 / 208, peak 227), RSS mean 429.1 MB / 441.8 MB / 441.9 MB (stable after first sample; no leak evidence), 32 threads, ~102 fds. Runs 2 and 3 had no probe subscribed to disparity/points.
- mcp__ros2-mcp__measure_topic_perf /stereo_camera/disparity 10 s: 6 msgs, 0.602 Hz, bw 1.66 MB/s, jitter 33 ms, latency p50 349.5 ms p95 356.8 ms p99 358.0 ms (header.stamp)
- mcp__ros2-mcp__measure_topic_perf /stereo_camera/points 10 s: 276 msgs, 27.78 Hz, bw 1.48 MB/s, jitter 35.6 ms, drop_estimate 5, latency p50 141.2 ms p95 296.4 ms p99 306.5 ms
- mcp__ros2-mcp__get_topic_hz /stereo_camera/left/camera_info 5 s: 34.0 Hz (80 msgs)
- mcp__ros2-mcp__get_topic_hz /stereo_camera/left/image_raw 4 s: 0 Hz (confirms compressed_only mode / subscription gating)
- mcp__ros2-mcp__get_topic_bw /stereo_camera/left/image_raw/compressed 5 s: 3.96 MB/s, 30 msgs; /stereo_camera/left/image_rect/compressed 5 s: 2.49 MB/s, 20 msgs (tool byte totals imply 600+ KB per JPEG which is implausible for 640x360; treat bw values as relative only)
- mcp__ros2-mcp__read_topic /stereo_camera/left/camera_info 1 msg: frame_id camera_left_link, K fx=588.39423 cx=295.79, P fx=631.56285 cx=347.18, R non-identity (r[2]=-0.0448)
- Bash ps/top -H on pid 8013: 32 threads; internal worker threads at 46.7/26.7/20/20/13.3% CPU plus argus_t/nvargus/EglStrm threads; binary is install_sys/.../stereo_camera_node built 2026-06-01 while src/stereo_camera_node.cpp was modified 2026-07-15
- Bash ldd on the installed stereo_camera_node: 273 shared objects; both libopencv_{core,imgproc,imgcodecs}.so.410 (/usr/local) and libopencv_{core,imgproc,imgcodecs}.so.4.5d (/lib/aarch64-linux-gnu) mapped; libyaml-cpp.so.0.7; libpcl_{common,filters,sample_consensus,search,kdtree,octree}; libopencv_cudastereo.so.410
- Bash /usr/bin/python3 cv2.getBuildInformation: OpenCV 4.10.0, NVIDIA CUDA YES (12.6), GPU arch 87, GStreamer YES 1.20.3; gcc 11.4.0
- Bash grep: 24 declare_parameter keys with no get_parameter reader; image_transport_/E_/F_/getters unused; no external package includes jetank_perception headers; jetank_web_control is the only subscriber to left/image_raw/compressed; jetank_ros_main launches stereo_camera_node and sock_segmentation_server
- NOT RUN: read_diagnostics (node publishes no /diagnostics); profile/measure of sock_segmentation_server and camera_node (not running); per-stage timing inside the node (no instrumentation available read-only)

Findings: 40 (high 4, medium 10, low 26).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_perception-01 | runtime | Disparity computed every frame even with no disparity/pointcloud/diagnostic subscriber | `src/jetank_perception/src/stereo_camera_node.cpp:1179` | high | measured | Removes GPU BM + 2x cvtColor per frame when nothing consumes depth; expected tens of % CPU at 30 fps | S | confirmed |
| jetank_perception-05 | runtime | DisparityImage published RELIABLE depth-1 (~0.9 MB/msg) delivers at 0.6 Hz to subscribers | `src/jetank_perception/src/stereo_camera_node.cpp:741` | high | measured | Disparity consumers get full frame rate; avoids retransmission traffic and stale caches in sock_segmentation_server | S | confirmed |
| jetank_perception-22 | minimality | yaml-cpp loader parses the same files CameraInfoManager already parsed, reads only K/D, and the calibrated R/P are discarded | `src/jetank_perception/src/stereo_camera_node.cpp:1520` | high | measured | Removes yaml-cpp dependency, stereo_calibration.yaml, ~110 lines of loader code; uses the actual calibrated rectification | M | confirmed |
| jetank_perception-02 | runtime | Rectification remap runs on 3-channel BGR, then converted to gray; rect images mislabeled mono8 | `src/jetank_perception/src/stereo_camera_node.cpp:1058` | high | static | ~3x less remap work per frame (two 640x360x3 -> 640x360x1 remaps), one fewer cvtColor pass; fixes encoding mismatch on image_rect | S | confirmed |
| jetank_perception-03 | runtime | appsink has no drop/max-buffers; CAP_PROP_BUFFERSIZE is a no-op for the GStreamer backend, so frames queue and add latency | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:329` | medium (orig high) | measured | Latency drops to roughly one processing period; removes stale-frame accumulation on the Jetson | S | confirmed |
| jetank_perception-07 | runtime | Per-frame get_parameter() in publish_disparity_image; f/t/delta_d inconsistent with calibration | `src/jetank_perception/src/stereo_camera_node.cpp:1326` | medium | measured | Removes a per-frame parameter lookup; makes downstream reprojection consistent | S | confirmed |
| jetank_perception-15 | runtime | OpenCV worker thread pool unbounded; 5 internal threads each 13-47% CPU; camera.processing_threads never applied | `src/jetank_perception/src/stereo_camera_node.cpp:310` | medium | measured | Fewer context switches; leaves cores for Nav2/MoveIt on the same Orin; or delete the two dead params | S | confirmed |
| jetank_perception-20 | minimality | 24 declared parameters are never read (dead config surface in code and YAML) | `src/jetank_perception/src/stereo_camera_node.cpp:309` | medium | measured | ~70 lines of C++ and ~45 lines of YAML removed; smaller parameter surface | S | confirmed |
| jetank_perception-04 | runtime | CPU videoconvert BGRx->BGR in pipeline plus node BGR->GRAY; camera.format is ignored | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:329` | medium | measured | Removes one full-frame CPU color conversion per camera per frame (2x at 30 fps) and one cvtColor per frame | M | confirmed |
| jetank_perception-14 | footprint | Two OpenCV runtimes (4.10 CUDA + 4.5.4d) loaded into one process via cv_bridge/image_transport | `src/jetank_perception/CMakeLists.txt:34` | medium (orig high) | measured | Drops a second OpenCV (~tens of MB mapped), pluginlib/class_loader, and an ABI hazard; faster start-up and link | M | confirmed |
| jetank_perception-21 | minimality | ~320 lines of calibration services duplicate camera_info_manager and produce a fake stereo calibration | `src/jetank_perception/src/stereo_camera_node.cpp:1432` | medium (orig high) | measured | ~320 lines, 3 services, filesystem/sstream/iomanip/fstream includes and the std::filesystem link removed | M | confirmed |
| jetank_perception-09 | runtime | StatisticalOutlierRemoval (k=30) on every frame is the heaviest filter in the chain | `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:637` | medium | static | Likely the largest single per-frame CPU saving in the cloud path if disabled or replaced | S | confirmed |
| jetank_perception-23 | minimality | Pipeline template/cache machinery for a single hard-coded template (~120 lines) and a static defined in a header | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:280` | medium | static | ~120 lines removed, no header-defined static, one camera open per sensor | S | confirmed |
| jetank_perception-28 | minimality | stereo_camera.launch.py declares 8 arguments that are never applied; single/simple launches carry template placeholders | `src/jetank_perception/launch/stereo_camera.launch.py:126` | medium | static | ~80 lines across launch files; one launch file instead of two for camera_node | S | confirmed |
| jetank_perception-16 | runtime | CameraInfo message rebuilt from CameraInfoManager and copied twice every frame | `src/jetank_perception/src/stereo_camera_node.cpp:1119` | low | measured | Removes 3 small copies x2 per frame | S | **UNVERIFIED** |
| jetank_perception-17 | runtime | Up to four CPU JPEG encodes per frame (raw L/R + rect L/R) when compressed streams are subscribed | `src/jetank_perception/src/stereo_camera_node.cpp:1145` | low | measured | Each avoided encode saves a few ms/frame of CPU on Orin Nano | S | **UNVERIFIED** |
| jetank_perception-33 | footprint | image_transport declared, found, and linked but never used | `src/jetank_perception/CMakeLists.txt:25` | low (orig medium) | measured | One fewer heavy runtime dependency; contributes to removing the duplicate OpenCV | S | confirmed |
| jetank_perception-36 | footprint | CMake hygiene: stdc++fs, INSTALL_INTERFACE on executables, double OpenCV linking, misnamed CUDA define, global include_directories | `src/jetank_perception/CMakeLists.txt:165` | low | measured | Cleaner, faster configure; unambiguous CUDA gating | S | **UNVERIFIED** |
| jetank_perception-39 | footprint | yaml-cpp build/link dependency exists only for the redundant calibration loader/saver | `src/jetank_perception/CMakeLists.txt:40` | low | measured | One fewer dependency and shared object; depends on -21/-22 | S | **UNVERIFIED** |
| jetank_perception-08 | runtime | All large messages published by copy (publish(*msg) / by value) instead of unique_ptr | `src/jetank_perception/src/stereo_camera_node.cpp:1108` | low | static | Saves ~1-2 MB of memcpy per frame across image/disparity/cloud topics | S | **UNVERIFIED** |
| jetank_perception-10 | runtime | capture_loop clones every frame into latest_frame_ and sleeps 1 ms although camera_node only uses the async callback | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:406` | low | static | One 921 KB memcpy per frame removed in camera_node; less jitter | S | **UNVERIFIED** |
| jetank_perception-11 | runtime | Each CSI camera is opened twice at startup (compatibility test then real open) plus test frames | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:305` | low | static | Roughly halves camera bring-up time; removes ~120 lines (see -23) | S | **UNVERIFIED** |
| jetank_perception-12 | runtime | sock_segmentation_server scans the whole disparity image per goal only to log a pixel count, and logs INFO per detection stage | `src/jetank_perception/src/sock_segmentation_server.cpp:239` | low | static | Removes a 230k-pixel pass and ~8 log lines per goal | S | **UNVERIFIED** |
| jetank_perception-13 | runtime | Blob transform round-trips PCL->PointCloud2->tf->PCL and copies the result cloud twice | `src/jetank_perception/src/sock_segmentation_server.cpp:487` | low | static | Removes two (de)serialisation passes and two cloud copies per goal | S | **UNVERIFIED** |
| jetank_perception-18 | runtime | Quality metrics use per-pixel .at<>() loops and full-size value copies (three double vectors for cloud stats) | `src/jetank_perception/src/quality_monitor.cpp:100` | low | static | ~4x fewer passes and ~6 MB less allocation per analysed frame when quality monitoring is on | S | **UNVERIFIED** |
| jetank_perception-19 | runtime | GPU strategy uses pageable host Mats, synchronous stream, and reserves 2x64 MB buffer pool | `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:204` | low | static | Removes ~128 MB reserved GPU pool and 3 staging copies per frame | S | **UNVERIFIED** |
| jetank_perception-24 | minimality | Unused CameraInterface virtuals (set_parameter/get_parameter/set_buffer_size/get_config/get_camera_type/supports_hardware_acceleration) | `src/jetank_perception/include/jetank_perception/camera_interface.hpp:53` | low | static | ~55 lines removed; smaller vtable/interface to keep in sync | S | **UNVERIFIED** |
| jetank_perception-25 | minimality | Dead strategy code: concrete subclasses, update_config x3, get_optimal_strategy_for_platform, unused StereoConfig/PointCloudConfig fields | `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:505` | low (orig medium) | static | ~110 lines removed; three fewer test cases that only test dead code | S | confirmed |
| jetank_perception-26 | minimality | QualityMonitoringConfig helpers and metric flags unused; visualization gating bypasses the master switch | `src/jetank_perception/include/jetank_perception/quality_monitoring.hpp:124` | low | static | ~35 header lines and ~30 node lines removed; consistent gating | S | **UNVERIFIED** |
| jetank_perception-27 | minimality | Unused members in StereoCalibration/JetsonStereoNode (image_transport_, E_/F_, getters, base_frame_id_) | `src/jetank_perception/src/stereo_camera_node.cpp:120` | low | static | ~20 lines and one header include (image_transport) removed | S | **UNVERIFIED** |
| jetank_perception-29 | minimality | Config YAML carries $(find-pkg-share) URLs that rcl never expands plus a stale header | `src/jetank_perception/config/stereo_camera_config.yaml:50` | low | static | ~50 YAML lines removed; no misleading keys | S | **UNVERIFIED** |
| jetank_perception-30 | minimality | Duplicated RANSAC plane setup in remove_ground_plane and remove_ground_height | `src/jetank_perception/src/sock_segmentation_server.cpp:548` | low | static | ~15-40 lines removed | S | **UNVERIFIED** |
| jetank_perception-31 | minimality | Raw and rectified publish/compress blocks duplicated verbatim in process_stereo_frames | `src/jetank_perception/src/stereo_camera_node.cpp:1027` | low | static | ~30 lines | S | **UNVERIFIED** |
| jetank_perception-32 | minimality | Unit tests mainly assert names/defaults of dead code and link full OpenCV+PCL | `src/jetank_perception/test/test_stereo_math.cpp:130` | low | static | Faster `colcon test` link; tests track real behaviour | S | **UNVERIFIED** |
| jetank_perception-34 | footprint | std_msgs, geometry_msgs linked into stereo_camera_node and launch_xml/launch_yaml exec_depends without use | `src/jetank_perception/CMakeLists.txt:111` | low | static | Smaller dependency closure / rosdep set; faster configure | S | **UNVERIFIED** |
| jetank_perception-37 | footprint | Headers installed with no export, and config install ships unused ost.txt | `src/jetank_perception/CMakeLists.txt:195` | low | static | Smaller install tree; no misleading exported API | S | **UNVERIFIED** |
| jetank_perception-38 | footprint | ament_lint_common pulls every linter into colcon test though copyright/cpplint are already disabled | `src/jetank_perception/package.xml:50` | low | static | Faster `pixi run test`; fewer test deps | S | **UNVERIFIED** |
| jetank_perception-40 | runtime | ros_topics input path uses toCvCopy for both frames where toCvShare suffices | `src/jetank_perception/src/stereo_camera_node.cpp:917` | low | static | Two frame copies per pair removed in simulation mode | S | **UNVERIFIED** |
| jetank_perception-41 | runtime | sock_segmentation_server spawns a detached thread per goal and blocks up to 0.4 s on TF lookups | `src/jetank_perception/src/sock_segmentation_server.cpp:196` | low | static | No per-goal thread creation; bounded latency | S | **UNVERIFIED** |
| jetank_perception-06 | runtime | Point cloud path allocates and copies the full frame 4-5 times per frame before filtering | `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:86` | low (orig medium) | static | Removes ~6 MB of per-frame allocation/copies and one full filter pass; several ms per frame on Orin Nano | M | confirmed |

### jetank_perception-01 — Disparity computed every frame even with no disparity/pointcloud/diagnostic subscriber

`src/jetank_perception/src/stereo_camera_node.cpp:1179` · package jetank_perception · lens runtime · severity high · evidence measured · effort S · verdict confirmed

Description: compute_and_publish_disparity_and_pointcloud() converts both rectified frames to gray (lines 1164-1176) and calls stereo_processor_->compute_disparity() unconditionally; only the publishes afterwards (lines 1235, 1240) are gated on get_subscription_count(). The GPU upload/BM/download plus two cvtColor passes run at camera rate whenever the node is up, regardless of consumers. Live profile with no disparity/points subscriber still showed ~194% CPU.

Evidence detail: profile_node /stereo_camera/stereo_camera_node 10 s (no probe subscribed to disparity/points during 2nd and 3rd runs): cpu mean 194.3% / 195.0%, p95 215%/208%, RSS mean 441.8 MB, 32 threads. get_node_info shows no external subscriber on /stereo_camera/points or /disparity in the graph. The unconditional call is at line 1179.

Estimated gain: Removes GPU BM + 2x cvtColor per frame when nothing consumes depth; expected tens of % CPU at 30 fps

Fix sketch: Compute `bool need_disparity = (disparity_pub_ && subs) || (pointcloud_pub_ && subs) || (diag pubs && subs)` at the top of compute_and_publish_disparity_and_pointcloud() and return early before the cvtColor/compute_disparity when false (also skip the rectify remap when neither rect images nor disparity are consumed).

Verifier (confirmed, adjusted high): src/jetank_perception/src/stereo_camera_node.cpp:1090 calls compute_and_publish_disparity_and_pointcloud() gated only on is_calibrated(); inside, cvtColor (1164-1176) and stereo_processor_->compute_disparity() (1179) run unconditionally while every output is gated on get_subscription_count() only afterwards (1207, 1223, 1235, 1240). The one non-topic consumer, quality-metric logging at 1188, is disabled by default (config/stereo_camera_config.yaml:128 quality_monitoring.enable: false), so with no subscribers the disparity result is discarded every frame. Note the cvtColor cost is already mitigated on the ROS-image path (mono8 conversion at 915) but not the GPU/BM compute; CPU numbers are from the original finding, not re-measured here.

### jetank_perception-05 — DisparityImage published RELIABLE depth-1 (~0.9 MB/msg) delivers at 0.6 Hz to subscribers

`src/jetank_perception/src/stereo_camera_node.cpp:741` · package jetank_perception · lens runtime · severity high · evidence measured · effort S · verdict confirmed

Description: create_publisher<DisparityImage>("disparity", 1) uses default RELIABLE QoS for a 640x360x32FC1 (921 KB) message every frame. Over the default UDP transport, large reliable messages with KeepLast(1) are fragmented and dropped/retransmitted, so subscribers see only a fraction of frames with large latency. sock_segmentation_server subscribes RELIABLE KeepLast(5) (sock_segmentation_server.cpp:131-138) and its own comment records ~1 s disparity age. Points at line 744 has the same QoS but smaller payloads and got through.

Evidence detail: measure_topic_perf /stereo_camera/disparity 10 s: count 6, hz 0.602, latency p50 349.5 ms / p99 358 ms, while /stereo_camera/points measured 27.78 Hz in the same window and camera_info 34 Hz. Publisher confirmed via get_node_info.

Estimated gain: Disparity consumers get full frame rate; avoids retransmission traffic and stale caches in sock_segmentation_server

Fix sketch: Use rclcpp::SensorDataQoS() (best-effort, KeepLast(5)) for disparity/points/images and match in sock_segmentation_server; alternatively publish disparity as 16-bit (halves size) or enable intra-process/shared-memory transport.

Verifier (confirmed, adjusted high): Re-measured this session: /stereo_camera/disparity 0.647 Hz, p50 latency 336 ms (~2.7 MB/msg by bw/count) vs /stereo_camera/points 30.2 Hz, so the symptom holds and the fix is in-scope (QoS change inside stereo_camera_node.cpp:741, no package or strategy/factory change). But the fix_sketch is NOT safe as written for "images/points": a best-effort publisher will not match the existing RELIABLE subscribers in sock_segmentation_server.cpp:131-134 (explicitly documented at :119-130 as a deliberate choice), sock_detector_node.py:203/345 and capture_frames.py:74 (default reliable Image subs), and the RViz PointCloud2 displays in jetank_ros_main/rviz/unified.rviz:85 and jetank_navigation/rviz/navigation.rviz:40 (default Reliable), all of which would silently receive nothing. The fix must be scoped to disparity with sock_segmentation_server changed in the same commit (or only reduce payload / raise pub depth while keeping RELIABLE), and any points/images QoS change requires updating those consumers and RViz configs.

### jetank_perception-22 — yaml-cpp loader parses the same files CameraInfoManager already parsed, reads only K/D, and the calibrated R/P are discarded

`src/jetank_perception/src/stereo_camera_node.cpp:1520` · package jetank_perception · lens minimality · severity high · evidence measured · effort M · verdict confirmed

Description: StereoCalibration::load_calibration_from_yaml (1520-1553) reads only camera_matrix and distortion_coefficients from left/right_camera.yaml; load_stereo_calibration_yaml (1555-1582) reads R, T, Q from stereo_calibration.yaml. compute_rectification_maps (1965-1984) then calls cv::stereoRectify with R=identity/T=[-0.06267,0,0], overwriting the loaded Q and ignoring the per-camera rectification_matrix (0.0447 rad, ~2.6 deg) and projection_matrix that camera_calibration produced. So (a) two parsers read the same files (CameraInfoManager at 787-791 already holds K,D,R,P), (b) the stereo_calibration.yaml file and yaml-cpp dependency exist only to feed values that are immediately recomputed, and (c) rectification uses a parallel-camera assumption instead of the calibrated one, degrading disparity quality that the stereo.* tuning then compensates for. from_rectified_camera_info_pair (1817-1912) already shows how to build Q from P1/P2.

Evidence detail: read_topic camera_info: r = [0.99899748, -0.00093753, -0.04475661, ...], p[0]=631.56 vs k[0]=588.39 (published but unused internally). Static: Q_ set at 1571 is overwritten at 1971-1975; R1_/R2_ come from stereoRectify not from the YAML.

Estimated gain: Removes yaml-cpp dependency, stereo_calibration.yaml, ~110 lines of loader code; uses the actual calibrated rectification

Fix sketch: Take K,D,R,P for each camera from CameraInfoManager::getCameraInfo(); initUndistortRectifyMap(K,D,R,P); Q from P1/P2 (fx=P[0], cx=P[2], cy=P[6], Tx=P_right[3]); delete yaml-cpp loader/saver and the stereo yaml.

Verifier (confirmed, adjusted high): stereo_camera_node.cpp:1520-1553 reads only camera_matrix/distortion_coefficients from left/right_camera.yaml, while config/calibration/left_camera.yaml carries a non-identity rectification_matrix (R[0][2]=-0.0448) and projection_matrix (fx=631.56) that the loader ignores; load_stereo_calibration_yaml (1555-1582) reads R=identity/T=[-0.06267,0,0]/Q from stereo_calibration.yaml, and compute_rectification_maps (1965-1984) then passes those to cv::stereoRectify, which overwrites Q_ and derives R1_/R2_/P1_/P2_ from the parallel-camera assumption. CameraInfoManager instances at 785-791 already parse the same files and getCameraInfo() is used at 1441-1442, and from_rectified_camera_info_pair (1817+) already builds Q from P, so the duplicate parser and stereo yaml are redundant as described. One caveat to the fix sketch: yaml-cpp is also used by save_calibration_to_yaml (1584) and save_stereo_calibration (1647), called from the set_camera_info/calibrate_stereo service handlers (1382, 1418, 1454), so dropping the dependency also requires replacing those savers (e.g. CameraInfoManager::setCameraInfo + camera_calibration_parsers).

### jetank_perception-02 — Rectification remap runs on 3-channel BGR, then converted to gray; rect images mislabeled mono8

`src/jetank_perception/src/stereo_camera_node.cpp:1058` · package jetank_perception · lens runtime · severity high · evidence static · effort S · verdict confirmed

Description: The CSI pipeline delivers BGR (3 channels). rectify_images() runs cv::remap on both 3-channel frames (line 1058 -> 1996-1997), then compute_and_publish_disparity_and_pointcloud() does cvtColor(BGR2GRAY) on the rectified result (lines 1164-1176). Remapping 3x the bytes and then discarding 2/3 of them wastes ~3x remap bandwidth per frame. Additionally, publish_image(left_rect_pub_, left_rectified, ..., "mono8") at lines 1066-1067 and the compressed variant at 1074-1081 tag a 3-channel Mat as mono8, producing an Image with step=cols*3 but encoding mono8 (corrupt for raw subscribers).

Evidence detail: Pipeline template (camera_interface.hpp:329) ends in videoconvert->appsink producing BGR; config camera.format=BGR8. rectify_images (1986-1998) has no channel handling; publish encoding hard-coded mono8 at 1066. Not measured in isolation.

Estimated gain: ~3x less remap work per frame (two 640x360x3 -> 640x360x1 remaps), one fewer cvtColor pass; fixes encoding mismatch on image_rect

Fix sketch: Convert to gray once before rectification (or capture GRAY8 directly, see -04), remap single-channel, publish rect images as mono8 from the true mono Mat; only keep a BGR copy when publish_raw_images_ is on and someone is subscribed.

Verifier (confirmed, adjusted high): The fix is safe and in-scope: no package outside jetank_perception subscribes to left/right image_rect (grep over src/ shows jetank_detection and jetank_web_control use /stereo_camera/left/image_raw, capture_frames.py:46, sock_detector_node.py:93, web_control_node.py:336), and left_rectified/right_rectified are consumed only by publish_image/publish_compressed_image (stereo_camera_node.cpp:1066-1081) and compute_and_publish_disparity_and_pointcloud, which immediately greys them (1164-1176). The CSI pipeline is hard-coded BGRx->videoconvert->appsink (camera_interface.hpp:329) and rectify_images remaps whatever channels it gets (1996-1997), while publish_image uses cv_bridge::CvImage(header,"mono8",image) with no channel check (1105), so a 3-channel Mat really is published as mono8 with step=cols*3. Converting to gray before remap (keeping BGR only for the raw publish at 1027-1048, which happens before rectification) changes no consumer contract, touches no strategy/factory abstraction, and mirrors the existing mono8 shortcut already used in the topic-source path (916).

### jetank_perception-03 — appsink has no drop/max-buffers; CAP_PROP_BUFFERSIZE is a no-op for the GStreamer backend, so frames queue and add latency

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:329` · package jetank_perception · lens runtime · severity medium (orig high) · evidence measured · effort S · verdict confirmed

Description: The pipeline string ends with a bare `appsink` (no `max-buffers=1 drop=true sync=false`). OpenCV's GStreamer backend does not implement CAP_PROP_BUFFERSIZE, so cap_.set(cv::CAP_PROP_BUFFERSIZE, ...) at line 135 and set_buffer_size() at 241-247 do nothing. When processing is slower than capture, appsink queues frames and get_frame() returns stale ones, inflating end-to-end latency instead of dropping.

Evidence detail: measure_topic_perf /stereo_camera/points 10 s: 27.78 Hz but header.stamp->receive latency p50 141 ms, p95 296 ms, p99 306 ms, jitter 35.6 ms, drop_estimate 5. A 28 Hz output with 140-300 ms latency implies frames are buffered ahead of processing rather than dropped. The stamp is taken after capture (process_stereo_frames(left,right) -> now(), line 1005), so this latency is processing+queueing, not capture age; capture age would add on top.

Estimated gain: Latency drops to roughly one processing period; removes stale-frame accumulation on the Jetson

Fix sketch: Change template to `... ! videoconvert ! appsink max-buffers=1 drop=true sync=false` and delete the CAP_PROP_BUFFERSIZE calls / camera.buffer_size parameter.

Verifier (confirmed, adjusted medium): Mechanism holds statically: the only template ends in bare `appsink` (camera_interface.hpp:329, fallback :387), `enable_threading()` is never called by stereo_camera_node.cpp so get_frame() does a synchronous `cap_ >> frame` (camera_interface.hpp:194) from the processing loop (stereo_camera_node.cpp:946-947), and CAP_PROP_BUFFERSIZE (hpp:135, :245) is unimplemented in OpenCV's GStreamer backend, so nothing bounds the appsink queue. Re-measured /stereo_camera/points 10 s: 28.83 Hz vs the hard-coded 30/1 capture rate, so the consumer is slightly slower than the source and frames must accumulate (or be dropped upstream by nvarguscamerasrc, which I could not verify). However the finding's "measured" numbers are not reproducible and do not measure queue age at all: my run shows p50 36 ms / p95 48 ms / jitter 3.9 ms / 0 drops (vs the claimed 141/296 ms), and since the stamp is taken after capture (stereo_camera_node.cpp:1005) the stamp->receive latency is processing+DDS, not stale-frame age. Fix is correct and S-effort, but the est_gain magnitude is unquantified, so severity is medium rather than high.

### jetank_perception-07 — Per-frame get_parameter() in publish_disparity_image; f/t/delta_d inconsistent with calibration

`src/jetank_perception/src/stereo_camera_node.cpp:1326` · package jetank_perception · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: publish_disparity_image() calls get_parameter("calibration.default.baseline") every frame (mutex + string map lookup), and sets f from the unrectified K (fx=588.39) while the published CameraInfo P and the Q used for the cloud have fx=631.56 and baseline 0.06267 (stereo_calibration.yaml). t=0.06 vs 0.06267 and f mismatch give ~4-7% depth error to any consumer using disparity.f/t (sock_reproject.hpp:212-216 does). delta_d=1.0 is also wrong for the 1/16-px 16S output of CPU BM.

Evidence detail: read_topic /stereo_camera/left/camera_info: k[0]=588.39423, p[0]=631.56285. Live param calibration.default.baseline=0.06; config/calibration/stereo_calibration.yaml T=[-0.06267,0,0]. get_parameter call at line 1326 executes per frame (called from compute_and_publish... at 1236 when subscribed).

Estimated gain: Removes a per-frame parameter lookup; makes downstream reprojection consistent

Fix sketch: Cache f (from P1_/Q_) and t (from T_ or -P2[3]/P2[0]) once after calibration load into members; set delta_d = 1/16 for 16S matchers; drop calibration.default.baseline parameter.

Verifier (confirmed, adjusted medium): stereo_camera_node.cpp:1326 calls get_parameter("calibration.default.baseline") every frame when a disparity subscriber exists (called from :1236), and :1325 takes f from camera_matrix_left_, which the YAML path (:1536) fills with the unrectified K (left_camera.yaml:7 fx=588.39) while stereo_calibration.yaml:19/28 gives T=-0.06267 and rectified fx=631.56; the declared default 0.06 (:360, stereo_camera_config.yaml:57) therefore disagrees with T_. delta_d=1.0 at :1329 is hard-coded despite CV_16S matchers (stereo_processor :264/:302/:407) yielding 1/16-px steps. sock_reproject.hpp:54-58 consumes disparity.f/t directly, so the mismatch propagates to reprojected sock positions (~4-7%). Live re-measurement not possible: /stereo_camera_node not running this session.

### jetank_perception-15 — OpenCV worker thread pool unbounded; 5 internal threads each 13-47% CPU; camera.processing_threads never applied

`src/jetank_perception/src/stereo_camera_node.cpp:310` · package jetank_perception · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: camera.processing_threads (declared 310) and pointcloud.max_processing_threads (368 -> PointCloudConfig.max_threads) are never used; cv::setNumThreads() is never called, so cvtColor/remap/imencode/flip spawn the default pool (6 cores) and compete with the GStreamer/argus threads and the ROS executor. The process runs 32 threads.

Evidence detail: top -H -p 8013: threads 8133 (46.7%), 8128 (26.7%), 8130 (20%), 8131 (20%), 8129 (13.3%) all named stereo_camera_node in addition to argus_t/nvargus/EglStrm threads; profile_node num_threads=32. Parameter declared at line 310 has no get_parameter() reader.

Estimated gain: Fewer context switches; leaves cores for Nav2/MoveIt on the same Orin; or delete the two dead params

Fix sketch: Call cv::setNumThreads(get_parameter("camera.processing_threads")) once in the constructor (2-3 is enough at 640x360) or remove both parameters.

Verifier (confirmed, adjusted medium): stereo_camera_node.cpp:310 declares camera.processing_threads with no get_parameter() reader anywhere in jetank_perception; pointcloud.max_processing_threads is read at :521 into pointcloud_config_.max_threads (stereo_processing_strategy.hpp:54) but max_threads is never consumed by any code, so both parameters are effectively dead. cv::setNumThreads() is not called anywhere in src/, and a live top -H on the running node (PID 8013) shows 32 threads with 5 unnamed stereo_camera_node threads at 47/27/20/20/20% CPU alongside argus/nvargus threads, consistent with OpenCV's default worker pool. Minor precision: the finding's claim that pointcloud.max_processing_threads is "never used" is slightly off (it is read at :521) but the value is never applied, so the substance holds.

### jetank_perception-20 — 24 declared parameters are never read (dead config surface in code and YAML)

`src/jetank_perception/src/stereo_camera_node.cpp:309` · package jetank_perception · lens minimality · severity medium · evidence measured · effort S · verdict confirmed

Description: declare_parameter for camera.buffer_size (309), camera.processing_threads (310), camera.processing_quality (311), calibration.auto_load_calibration/min_calibration_samples/max_reprojection_error/default.focal_length/transforms.publish_camera_transforms (356-361), pointcloud.enable (366), publishing.qos_depth (391), performance.enable_multithreading/thread_priority/enable_memory_optimization/max_memory_usage_mb (398-401), logging.* (406-408), development.* (413-418) have no get_parameter() reader; quality_monitoring.metrics.* and calibration_validation.* (427-431, 437-439) are loaded into quality_config_ (548-573) but never consulted. The same keys occupy ~45 lines of config/stereo_camera_config.yaml (18-20, 53-58, 64-66, 83, 102-121, 135-152). Every declaration adds a parameter-server entry and a startup parse; the YAML misleads users into thinking they tune behaviour.

Evidence detail: Scripted grep over declare_parameter vs get_parameter in the session listed exactly these 24 unread keys; get_node_params on the live node shows all of them present.

Estimated gain: ~70 lines of C++ and ~45 lines of YAML removed; smaller parameter surface

Fix sketch: Delete the declarations, the load_parameters() lines and YAML keys; keep only parameters with a reader.

Verifier (confirmed, adjusted medium): Workspace grep confirms none of the 23 listed non-quality keys has a get_parameter() reader anywhere in src/ (only declare_parameter in stereo_camera_node.cpp:309-418 and the YAML at config/stereo_camera_config.yaml:18-121), and quality_config_.metrics.* / calibration_validation.* are assigned at stereo_camera_node.cpp:548-573 but their only consumer, any_expensive_metrics_enabled() in include/jetank_perception/quality_monitoring.hpp:68-71, has no caller. The fix is in-scope (no package merge, no strategy/factory change) and safe with one addition: jetank_perception/launch/stereo_camera.launch.py:151-152 still passes calibration.transforms.publish_camera_transforms as a node override (fed from jetank_ros_main/launch/unified.launch.py:264), so that override entry should be removed alongside the declaration (rclcpp silently ignores undeclared overrides, so it would not crash, but it would be a dangling reference); the launch argument itself must stay because lines 189/206 use it to gate static_transform_publisher nodes.

### jetank_perception-04 — CPU videoconvert BGRx->BGR in pipeline plus node BGR->GRAY; camera.format is ignored

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:329` · package jetank_perception · lens runtime · severity medium · evidence measured · effort M · verdict confirmed

Description: The only template requests NVMM NV12 -> nvvidconv -> BGRx -> videoconvert (CPU) -> BGR. The node then converts BGR->GRAY on CPU for stereo (stereo_camera_node.cpp:1165). CameraConfig.format / camera.format ("BGR8") are only used in the cache key (line 394) and never affect the pipeline. nvvidconv can output GRAY8 directly (hardware), which would remove both the videoconvert element and the cvtColor pass; when the color raw stream is needed for the web viewer, capturing BGRx and using cvtColor(BGRA2GRAY) still avoids the videoconvert stage. Also framerate=30/1 and 1280x720 sensor mode are hard-coded, ignoring camera.fps.

Evidence detail: top -H on pid 8013 showed 5 process-internal worker threads at 13-47% CPU each plus nvargus/argus threads; videoconvert runs inside those. Static: template at line 329 hard-codes BGRx+videoconvert and framerate 30/1; camera_config_.fps only appears in logs and the cache key.

Estimated gain: Removes one full-frame CPU color conversion per camera per frame (2x at 30 fps) and one cvtColor per frame

Fix sketch: Build the caps from config.format: for stereo use `nvvidconv ! video/x-raw,format=GRAY8,width=%d,height=%d ! appsink ...`; use config.fps in the framerate caps; drop videoconvert.

Verifier (confirmed, adjusted medium): camera_interface.hpp:329 is the sole template and hard-codes `format=BGRx ! videoconvert` plus `framerate=30/1`; grep shows config.fps and config.format are used only in the cache key at camera_interface.hpp:394, never in the pipeline string. stereo_camera_node.cpp:1165 and :1172 then run cv::cvtColor(BGR2GRAY) on every frame because frames arrive 3-channel, so the CPU videoconvert plus cvtColor pass per camera per frame exists as described. Note the node declares camera.format default "GRAY8" (stereo_camera_node.cpp:305) while the yaml sets "BGR8" (config/stereo_camera_config.yaml:13), neither of which has any effect. The CPU numbers were not re-measured in this session; the static evidence alone supports the finding.

### jetank_perception-14 — Two OpenCV runtimes (4.10 CUDA + 4.5.4d) loaded into one process via cv_bridge/image_transport

`src/jetank_perception/CMakeLists.txt:34` · package jetank_perception · lens footprint · severity medium (orig high) · evidence measured · effort M · verdict confirmed

Description: find_package(OpenCV) resolves to /usr/local OpenCV 4.10 (CUDA), but the apt cv_bridge and image_transport plugins pulled in by ament_target_dependencies (lines 116-117) were built against Ubuntu OpenCV 4.5.4d, so the executable maps libopencv_core/imgproc/imgcodecs twice (ABI mixing risk, duplicated code pages and heap). cv_bridge is used only for CvImage::toImageMsg / toCvCopy (stereo_camera_node.cpp:405,917,1105; single_camera.cpp:112), which is ~10 lines of manual Image filling; image_transport is never used at all (member at line 120 only).

Evidence detail: ldd install_sys/.../stereo_camera_node: 273 shared objects including libopencv_core.so.410, libopencv_imgproc.so.410, libopencv_imgcodecs.so.410 (/usr/local) AND libopencv_core.so.4.5d, libopencv_imgproc.so.4.5d, libopencv_imgcodecs.so.4.5d (/lib/aarch64-linux-gnu). profile_node RSS mean 441.9 MB.

Estimated gain: Drops a second OpenCV (~tens of MB mapped), pluginlib/class_loader, and an ABI hazard; faster start-up and link

Fix sketch: Remove image_transport and cv_bridge from CMake/package.xml; fill sensor_msgs::msg::Image directly (encoding, step=mat.step, data.assign(mat.datastart, mat.dataend)) and decode ROS-topic input with cv::Mat wrapping msg->data; or rebuild cv_bridge against OpenCV 4.10 in the workspace.

Verifier (confirmed, adjusted medium): Live stereo_camera_node (pid 8013) /proc maps confirm both runtimes loaded: libopencv_{core,imgproc,imgcodecs}.so.4.5.4d from /usr/lib plus the 4.10.0 set from /usr/local, and ldd of /opt/ros/humble/lib/libcv_bridge.so shows it pulls the 4.5d trio; CMakeLists.txt:24-25,116-117 link cv_bridge/image_transport while image_transport is only a never-used member (stereo_camera_node.cpp:120) and cv_bridge is 5 trivial toImageMsg/toCvCopy call sites (stereo_camera_node.cpp:917-918,1105,1213,1227; single_camera.cpp:112). However the memory gain is overstated: smaps PSS for the three 4.5.4d libs totals ~2.8 MB (1182+1044+537 kB) against 430 MB RSS, not "tens of MB", so the real value is the ABI-mixing hazard (cv::Mat crossing two libopencv_core symbol sets) and dependency trimming, which justifies medium rather than high.

### jetank_perception-21 — ~320 lines of calibration services duplicate camera_info_manager and produce a fake stereo calibration

`src/jetank_perception/src/stereo_camera_node.cpp:1432` · package jetank_perception · lens minimality · severity medium (orig high) · evidence measured · effort M · verdict confirmed

Description: set_left/right_camera_info (1360-1430), calibrate_stereo (1432-1470, which hard-codes R=I, T=-0.06 m with a comment 'In a real implementation...'), save_calibration_to_yaml (1584-1645), save_stereo_calibration (1647-1715), from_camera_info_msg (1782-1815), calibrate_stereo_from_individual (1914-1934), validate_camera_info (1334-1358) and setup_services (762-783) re-implement what camera_info_manager already provides (its own set_camera_info service persisting to the URL). Live graph shows /stereo_camera/set_camera_info advertised by the two CameraInfoManagers (both named for the same node, so they collide) next to the node's left/right services. Calibration is in practice done offline with the camera_calibration tool (config/calibration/ost.txt is its output).

Evidence detail: get_node_info lists services /stereo_camera/set_camera_info, /stereo_camera/left/set_camera_info, /stereo_camera/right/set_camera_info, /stereo_camera/calibrate_stereo. Line ranges counted in-session: 324 lines.

Estimated gain: ~320 lines, 3 services, filesystem/sstream/iomanip/fstream includes and the std::filesystem link removed

Fix sketch: Delete the three services and the save/convert helpers; rely on camera_info_manager (use the standard left/right sub-namespace so its set_camera_info services do not collide) and offline camera_calibration to write the YAML.

Verifier (confirmed, adjusted medium): Code facts hold: setup_services (src/stereo_camera_node.cpp:762-783) advertises left/right set_camera_info and calibrate_stereo; calibrate_stereo_from_individual (1914-1934) hard-codes R=I and T=(-0.06,0,0) under the comment "In a real implementation..." yet the service reports "Stereo calibration completed and saved" (1464); std::filesystem/ofstream/stringstream are used only inside save_calibration_to_yaml (1623-1633) and save_stereo_calibration (1693-1703), so the four includes and the stdc++fs link (CMakeLists.txt:165) really do go away with the deletion, and the only external references are three README rows (README.md:58-60). The live service-collision claim could not be re-verified: get_node_info for /stereo_camera returned "nonexistent node" (node not running this session). Gain is plausible as stated (~320 of 2019 lines, 16%), but this code sits behind idle services with zero hot-path CPU/latency cost on the Orin, so "high" overstates it; medium is honest for a LOC/dependency cleanup that also removes a misleading fake-success calibration path.

### jetank_perception-09 — StatisticalOutlierRemoval (k=30) on every frame is the heaviest filter in the chain

`src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:637` · package jetank_perception · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: apply_statistical_filter builds a KD-tree and runs a 30-NN query per point plus a second pass, O(N log N) with a large constant, every frame after voxelisation (config: enable=true, k_neighbors=30, stddev=2.0, leaf 5 mm which barely downsamples a 640x360 cloud within 2 m). This is typically the dominant CPU cost of the cloud stage on an Orin Nano and runs even when the consumer (Nav2 pointcloud->laserscan) tolerates outliers.

Evidence detail: Filter order at lines 603-615; live params pointcloud.statistical_filter.enable=true, k_neighbors=30, voxel leaf 0.005. Not profiled per-stage.

Estimated gain: Likely the largest single per-frame CPU saving in the cloud path if disabled or replaced

Fix sketch: Default statistical_filter.enable=false; if outlier removal is needed use pcl::RadiusOutlierRemoval with small radius or increase voxel leaf to 1-2 cm first; consider running SOR only for the nav consumer at its rate.

Verifier (confirmed, adjusted medium): The fix is a config-only default flip: `pointcloud.statistical_filter.enable` is read from a declared parameter in stereo_camera_node.cpp:375/511-512 and gated at stereo_processing_strategy.hpp:613-614, so disabling it (or raising `voxel_filter.leaf_size` at stereo_camera_config.yaml:68) touches no class, factory, or strategy interface. No other package consumes the stereo point cloud: jetank_navigation's Nav2 costmaps and slam_toolbox use `/scan` from the RPLidar (nav2_params.yaml:204-206, 235-237; slam_toolbox.yaml:15), and grep found no PointCloud2 subscriber in jetank_navigation/jetank_ros_main/jetank_detection. Only behavioral change is a noisier cloud in RViz. Not profiled per-stage this session, so the "dominant cost" claim remains static reasoning.

### jetank_perception-23 — Pipeline template/cache machinery for a single hard-coded template (~120 lines) and a static defined in a header

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:280` · package jetank_perception · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: PipelineTemplate struct (31-38, description/priority never used), static pipeline_cache_ declared and DEFINED in the header (70, 74 - an ODR violation if two TUs include it), build_gstreamer_pipeline/get_pipeline_templates/build_pipeline_from_template/test_pipeline_compatibility/get_fallback_pipeline/create_cache_key (280-395) implement a priority search over exactly one template with a fallback that is never a better option. The `else` branch in build_pipeline_from_template (348-354) is unreachable.

Evidence detail: get_pipeline_templates pushes one entry (326-334); name compare at 343 always true.

Estimated gain: ~120 lines removed, no header-defined static, one camera open per sensor

Fix sketch: Replace with a single `std::string make_pipeline(const CameraConfig&)` using snprintf/format; open once in initialize().

Verifier (confirmed, adjusted medium): All pipeline machinery (PipelineTemplate at camera_interface.hpp:32-38, pipeline_cache_ at :70/:74, build_gstreamer_pipeline..create_cache_key at :281-395) is private to JetsonCSICamera and referenced nowhere else in the workspace; the only consumers are jetank_perception/src/single_camera.cpp:46 and src/stereo_camera_node.cpp:610/624, which use only CameraFactory::create_camera and the public CameraInterface API, so a private `make_pipeline(const CameraConfig&)` replacement breaks no consumer and leaves the strategy/factory abstractions intact. The two TUs are separate executables (CMakeLists.txt:97,101), so the header-defined static at :74 does not link-fail today but will the moment two TUs in one target include the header. Behaviorally, get_pipeline_templates (:326-334) yields exactly one template so the loop at :300 runs once; the fallback at :384-388 (nvarguscamerasrc straight into videoconvert, no nvvidconv, no size caps) would not negotiate NVMM output and ignores the requested resolution, so dropping it plus the test-open (:359-382, a second full camera open + frame read per sensor at startup) changes only the failure path, which already fails at cap_.open (:104,129).

### jetank_perception-28 — stereo_camera.launch.py declares 8 arguments that are never applied; single/simple launches carry template placeholders

`src/jetank_perception/launch/stereo_camera.launch.py:126` · package jetank_perception · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: config_file (45-53) is declared but the Node uses a hard-coded PathJoinSubstitution (144-148); camera_width/height/fps (71-89), stereo_algorithm (92-98), left/right_camera_info_url (108-120) are declared and listed (236-242) but never read; build_parameter_overrides (126-132) returns {} and is never called; the LogInfo prints config_file that is not in effect. single_camera.launch.py includes an rviz node with '/path/to/your/rviz/config.rviz' (112) and an image_view node (124-133); simple_camera.launch.py duplicates single_camera.launch.py with fixed values. Every DeclareLaunchArgument adds parse time and misleads `ros2 launch --show-args`.

Evidence detail: grep in session: LaunchConfiguration('config_file') appears only in LogInfo (219); no LaunchConfiguration('camera_width'/'stereo_algorithm'/'left_camera_info_url') anywhere; build_parameter_overrides has no call site.

Estimated gain: ~80 lines across launch files; one launch file instead of two for camera_node

Fix sketch: Either wire the args into parameters (only when non-empty) or delete them; use LaunchConfiguration('config_file') for the params file; remove rviz/image_view template nodes; delete simple_camera.launch.py.

Verifier (confirmed, adjusted medium): stereo_camera.launch.py:45-53 declares config_file but the Node at 143-148 hard-codes the same PathJoinSubstitution and LaunchConfiguration('config_file') appears only in the LogInfo at 219; camera_width/height/fps (71-89), stereo_algorithm (92-98) and left/right_camera_info_url (108-120) have no LaunchConfiguration reads anywhere in the file (grep confirms), and build_parameter_overrides (126-132) returns {} with zero call sites in src/. single_camera.launch.py:112 still carries '/path/to/your/rviz/config.rviz' and line 77 has a 'Replace with your executable name' template comment; simple_camera.launch.py (43 lines) launches the same camera_node with fixed values, duplicating single_camera.launch.py. Severity stays medium: the runtime cost is small (launch parse only), but the dead args actively mislead `--show-args` users into thinking overrides work when they are silently ignored.

Duplicate folded in: **cross-17** — same dead launch arguments in stereo_camera.launch.py; cross-17 additionally proposes dropping the two default-off static_transform_publisher nodes (177-208) since the URDF provides those frames.

### jetank_perception-16 — CameraInfo message rebuilt from CameraInfoManager and copied twice every frame

`src/jetank_perception/src/stereo_camera_node.cpp:1119` · package jetank_perception · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: publish_camera_info() calls manager->getCameraInfo() (returns by value: copy 1), wraps it in make_shared (copy 2), then publish(*info_msg) (copy 3) at 30 fps for both cameras. The intrinsics never change at runtime except via the set_camera_info services. The shared_ptr<CameraInfoManager> and Publisher SharedPtr are also passed by value (atomic refcount churn).

Evidence detail: get_topic_hz /stereo_camera/left/camera_info: 34.0 Hz while subscribed; code path lines 1111-1123.

Estimated gain: Removes 3 small copies x2 per frame

Fix sketch: Cache left_info_msg_/right_info_msg_ once (update on set_camera_info), publish a unique_ptr copy with only the stamp changed; pass publishers/managers by const reference.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-17 — Up to four CPU JPEG encodes per frame (raw L/R + rect L/R) when compressed streams are subscribed

`src/jetank_perception/src/stereo_camera_node.cpp:1145` · package jetank_perception · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: With compression enabled for both raw and rectified streams (config lines 86-97), each subscribed stream costs a cv::imencode JPEG on the CPU per frame. The only in-repo consumer is jetank_web_control on left/image_raw/compressed; the rectified compressed streams and right raw exist without a consumer. Jetson has nvjpegenc which could do this in hardware inside the GStreamer pipeline.

Evidence detail: get_topic_bw left/image_raw/compressed 5 s: 3.96 MB/s (30 msgs); left/image_rect/compressed: 2.49 MB/s (20 msgs) while probed (tool byte counts look inflated per message; treat as relative). grep: only jetank_web_control subscribes to left/image_raw/compressed.

Estimated gain: Each avoided encode saves a few ms/frame of CPU on Orin Nano

Fix sketch: Default rectified_images.compression.enabled=false; keep subscription gating; consider a GStreamer tee with nvjpegenc for the web stream.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-33 — image_transport declared, found, and linked but never used

`src/jetank_perception/CMakeLists.txt:25` · package jetank_perception · lens footprint · severity low (orig medium) · evidence measured · effort S · verdict confirmed

Description: find_package(image_transport) (25), ament_target_dependencies(stereo_camera_node ... image_transport) (118), package.xml <depend>image_transport</depend> (28) and the two includes (stereo_camera_node.cpp:11-12) support a member (line 120) that is never instantiated. image_transport pulls pluginlib/class_loader and the compressed transport plugins (which are what drag in the second OpenCV in -14).

Evidence detail: grep: image_transport_ only at declaration; ldd shows libopencv_*.so.4.5d present alongside 4.10, consistent with the apt-built transport/cv_bridge stack.

Estimated gain: One fewer heavy runtime dependency; contributes to removing the duplicate OpenCV

Fix sketch: Remove from CMakeLists, package.xml and includes.

Verifier (confirmed, adjusted low): stereo_camera_node.cpp:120 declares `image_transport_` and it is never assigned or used; all image outputs use plain `create_publisher` (703-755), including manual `CompressedImage` publishers (719-733), so no behavior depends on image_transport. CMakeLists.txt has no `ament_export_dependencies` for it, and the only downstream consumer (jetank_ros_main/package.xml:27) is a launch-only package, so removing it from CMakeLists.txt:25/117, package.xml:28 and the includes at stereo_camera_node.cpp:11-12 (plus the member at :120, which the fix_sketch omits but is required to compile) breaks nothing and stays inside the perception package. Severity lowered: cv_bridge and camera_info_manager (CMakeLists.txt:24,26) remain and are the same apt-built OpenCV-4.5 stack, so this alone will not remove the duplicate OpenCV the finding credits it with.

### jetank_perception-36 — CMake hygiene: stdc++fs, INSTALL_INTERFACE on executables, double OpenCV linking, misnamed CUDA define, global include_directories

`src/jetank_perception/CMakeLists.txt:165` · package jetank_perception · lens footprint · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: stdc++fs (165) is unnecessary on GCC 11 (std::filesystem is in libstdc++); target_include_directories with INSTALL_INTERFACE on executables (154-156, 169-171) is meaningless; OpenCV is linked both via ament_target_dependencies(... OpenCV) (122) and ${OpenCV_LIBS} (161); the CUDA detection defines -DOPENCV_ENABLE_NONFREE (54) which is OpenCV's real macro for the nonfree/xfeatures2d modules, not CUDA, and the header keys all CUDA code on it (stereo_processing_strategy.hpp:18, 139, 154...); global include_directories/link_directories/add_definitions (46-63) leak into every target; the comment at 42 ('Add filesystem for C++17') precedes find_package(Threads). None affect runtime but they slow/obscure the build.

Evidence detail: gcc --version: 11.4.0; cv2.getBuildInformation confirms CUDA 12.6 so the define is currently set and exercised.

Estimated gain: Cleaner, faster configure; unambiguous CUDA gating

Fix sketch: Drop stdc++fs and INSTALL_INTERFACE; keep one OpenCV link path; rename to JETANK_OPENCV_CUDA and set via target_compile_definitions on the stereo target only; move include dirs to targets.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-16** — same stdc++fs link at CMakeLists.txt:165; footprint-16 notes the pixi toolchain is GCC 14.3 (libstdc++ 15) so the archive is unnecessary there too.

### jetank_perception-39 — yaml-cpp build/link dependency exists only for the redundant calibration loader/saver

`src/jetank_perception/CMakeLists.txt:40` · package jetank_perception · lens footprint · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: pkg_check_modules(YAML_CPP) (39-40), include (62), link (163) and package.xml libyaml-cpp-dev (36) serve StereoCalibration's YAML code (stereo_camera_node.cpp 1520-1715) which duplicates camera_info_manager's parser (-22) and the calibration-save services (-21). Removing those removes yaml-cpp entirely from the package and one shared object from the node.

Evidence detail: ldd shows libyaml-cpp.so.0.7 mapped by the running node; grep: YAML:: appears only in stereo_camera_node.cpp calibration functions.

Estimated gain: One fewer dependency and shared object; depends on -21/-22

Fix sketch: After -21/-22, delete the pkg_check_modules block, include/link entries and the package.xml depend.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-08 — All large messages published by copy (publish(*msg) / by value) instead of unique_ptr

`src/jetank_perception/src/stereo_camera_node.cpp:1108` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: publish_image builds a shared_ptr via toImageMsg() then publishes *msg (copy, line 1108); publish_camera_info make_shared copies then publishes *info_msg (1119-1122); disparity_msg (921 KB) published by value (1331); pc_msg by value (1288); diagnostics images (1216, 1230); sock server debug_pub_->publish(cloud_target) (sock_segmentation_server.cpp:527). Each costs a full serialization-size memcpy and prevents intra-process zero-copy. Only publish_compressed_image (1153) uses unique_ptr correctly.

Evidence detail: Lines cited; rclcpp Publisher::publish(const T&) copies into a new message before handing to rmw, whereas publish(std::unique_ptr<T>) moves.

Estimated gain: Saves ~1-2 MB of memcpy per frame across image/disparity/cloud topics

Fix sketch: Build messages as std::make_unique<T>() and publish(std::move(msg)); for cv_bridge use toImageMsg(*msg) into a unique_ptr-owned Image.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-10 — capture_loop clones every frame into latest_frame_ and sleeps 1 ms although camera_node only uses the async callback

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:406` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: In threaded/async mode (used by camera_node, single_camera.cpp:61,74) capture_loop clones each frame (921 KB memcpy at 30 fps) into latest_frame_ under a mutex, then invokes the callback with the original; camera_node never calls get_frame(), so the clone is wasted. The 1 ms sleep after a blocking cap>>frame adds jitter for no reason. camera_node also captures at full rate then discards frames in frame_callback (single_camera.cpp:106) instead of setting the pipeline framerate. camera_format and use_hardware_acceleration params (single_camera.cpp:17,19) are never applied.

Evidence detail: Lines 397-416 (capture_loop), single_camera.cpp 61-77, 104-108.

Estimated gain: One 921 KB memcpy per frame removed in camera_node; less jitter

Fix sketch: Only store latest_frame_ when no async callback is registered; remove the sleep; drive publish rate from the GStreamer framerate caps.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-11 — Each CSI camera is opened twice at startup (compatibility test then real open) plus test frames

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:305` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: build_gstreamer_pipeline() calls test_pipeline_compatibility() which opens the nvargus pipeline, reads a frame and releases it (lines 359-382), then initialize() opens the same pipeline again (104) and reads another test frame just for std::cout debug (117-125). Argus session setup is slow (hundreds of ms per open) and the two cameras are opened sequentially, so node start-up pays ~4 camera opens. The static pipeline_cache_ only helps within a process.

Evidence detail: Call chain lines 102-104, 284-320, 359-382; there is exactly one template so the fallback loop never has a second option.

Estimated gain: Roughly halves camera bring-up time; removes ~120 lines (see -23)

Fix sketch: Open the pipeline once in initialize(); treat isOpened()+first read as the compatibility check; log via RCLCPP not std::cout.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-12 — sock_segmentation_server scans the whole disparity image per goal only to log a pixel count, and logs INFO per detection stage

`src/jetank_perception/src/sock_segmentation_server.cpp:239` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 239-265 iterate all 230k disparity pixels every goal purely to produce an RCLCPP_INFO line; lines 221-225, 272-280, 360-367, 388-393, 408-412 and the drop messages log at INFO for every detection, including a get_parameter("use_sim_time") per goal (280). This is debugging output left in the hot path of an action that may be called repeatedly during a grasp loop.

Evidence detail: Code at cited lines; no measurement (no live sock_segmentation_server).

Estimated gain: Removes a 230k-pixel pass and ~8 log lines per goal

Fix sketch: Delete the diagnostic scan block; demote per-detection logs to RCLCPP_DEBUG; keep one INFO summary per goal.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-13 — Blob transform round-trips PCL->PointCloud2->tf->PCL and copies the result cloud twice

`src/jetank_perception/src/sock_segmentation_server.cpp:487` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The chosen blob is serialised with pcl::toROSMsg (488), transformed as PointCloud2 (493), then deserialised again with pcl::fromROSMsg (509) just to get min/max; SockCloud sock is built and then copied into result->sock (523) and cloud_target copied again into debug_pub_->publish (527). Two blocking lookupTransform calls with 0.2 s timeout each (688, 696) also run per goal. A single pcl::transformPointCloud with an Eigen::Affine3f from tf_to_target, then getMinMax3D/compute3DCentroid on the transformed PCL cloud and one toROSMsg into result->sock.cloud, removes 2 serialisations and 2 copies.

Evidence detail: Lines 485-528.

Estimated gain: Removes two (de)serialisation passes and two cloud copies per goal

Fix sketch: tf2::fromMsg(tf_to_target.transform, eigen); pcl::transformPointCloud(*chosen.cloud, out, eigen); compute centroid/min-max on out; pcl::toROSMsg(out, result->sock.cloud); std::move into result.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-18 — Quality metrics use per-pixel .at<>() loops and full-size value copies (three double vectors for cloud stats)

`src/jetank_perception/src/quality_monitor.cpp:100` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: analyze_disparity_quality collects every valid disparity into a std::vector<float> (reserve 230k) and loops with .at<float>(y,x) (100-115), then does 4 more passes for min/max/sum/sq_sum; analyze_pointcloud_quality copies x,y,z into three std::vector<double> (174-189) and re-scans; create_depth_uncertainty_map uses .at<> per pixel (302-312); analyze_image_quality runs Laplacian in CV_64F (43). All can be single-pass with row pointers and running sums. Disabled by default (quality_monitoring.enable=false) so cost is only when enabled.

Evidence detail: Lines cited; live quality_monitoring.enable=false.

Estimated gain: ~4x fewer passes and ~6 MB less allocation per analysed frame when quality monitoring is on

Fix sketch: Use row pointers, Welford/one-pass sums, min/max in the same loop; Laplacian CV_16S; drop the vectors.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-19 — GPU strategy uses pageable host Mats, synchronous stream, and reserves 2x64 MB buffer pool

`src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:204` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: compute_disparity uploads from pageable cv::Mat, computes on stream_, waitForCompletion()s, then downloads into a freshly allocated cv::Mat each frame (204-215). On Jetson unified memory, pinned cv::cuda::HostMem buffers (or zero-copy) avoid the staging copies and the per-frame output allocation. optimize_for_jetson() (268-279) re-creates stream_ and calls setBufferPoolConfig(64 MB, 2) which reserves 128 MB of device memory that this workload (two 230 KB inputs) never needs.

Evidence detail: Lines 199-230, 258-279; OpenCV 4.10 CUDA confirmed by cv2.getBuildInformation (CUDA 12.6, arch 87) so this path is active with stereo.algorithm=GPU_BM.

Estimated gain: Removes ~128 MB reserved GPU pool and 3 staging copies per frame

Fix sketch: Preallocate cv::cuda::HostMem(PAGE_LOCKED) for left/right/disparity and reuse; drop setBufferPoolConfig or size it to a few MB; keep a single stream.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-24 — Unused CameraInterface virtuals (set_parameter/get_parameter/set_buffer_size/get_config/get_camera_type/supports_hardware_acceleration)

`src/jetank_perception/include/jetank_perception/camera_interface.hpp:53` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The 'extended' and 'parameter' interface (53-63) and their JetsonCSICamera implementations (226-277) have no caller in the package: stereo node uses initialize/start/stop/get_frame; camera_node adds enable_threading/get_frame_async/is_running. set_buffer_size is also a no-op on the GStreamer backend (see -03). CameraFactory has a single enum value with an unreachable default branch (425-438).

Evidence detail: grep across src/ and test/ in session: no calls to these methods.

Estimated gain: ~55 lines removed; smaller vtable/interface to keep in sync

Fix sketch: Trim the interface to the six used methods; keep the factory (architecture) but drop the dead switch default.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-25 — Dead strategy code: concrete subclasses, update_config x3, get_optimal_strategy_for_platform, unused StereoConfig/PointCloudConfig fields

`src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:505` · package jetank_perception · lens minimality · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: CPUBlockMatchingStereo/GPUSGBMStereo/CPUSGBMStereo (505-535) are never created by the factory or node; only test_stereo_math.cpp instantiates them (145-152) to assert their names. update_config() in all three strategies (232-250, 343-359, 472-493) is never called (no parameter callback exists). get_optimal_strategy_for_platform (568-579) is unused. StereoConfig.use_gpu (41, loaded at node 502) and max_disparity (32, mirror of num_disparities at 493) and smaller_block_size (36; StereoBM::setSmallerBlockSize is a no-op in OpenCV) plus PointCloudConfig.max_threads/downsample_factor (54-55) are never read by any strategy.

Evidence detail: grep in session: create_strategy is the only constructor path; no update_config callers; fields only assigned.

Estimated gain: ~110 lines removed; three fewer test cases that only test dead code

Fix sketch: Delete the three subclasses, update_config overrides (or add a real parameter callback if runtime tuning is wanted), get_optimal_strategy_for_platform, and the unused config fields/params.

Verifier (confirmed, adjusted low): Evidence supports the finding: stereo_camera_node.cpp:675 is the only strategy construction path (via create_strategy, which never returns the subclasses at stereo_processing_strategy.hpp:505-535); grep finds no add_on_set_parameters_callback and no callers of update_config (declared at hpp:71, overrides at 232/343/472, also the PointCloud one at 622); get_optimal_strategy_for_platform (hpp:568) has no callers; StereoConfig.use_gpu, PointCloudConfig.max_threads/downsample_factor are only assigned (node:502/521/522), never read. One inaccuracy: max_disparity IS read at hpp:162 (cuda createStereoBM), though it is merely a mirror of num_disparities set at node:493. The ~110-LOC gain is plausible, but this is header-only dead code with zero hot-path or memory cost on the Orin, so "medium" overstates it; honest severity is low.

### jetank_perception-26 — QualityMonitoringConfig helpers and metric flags unused; visualization gating bypasses the master switch

`src/jetank_perception/include/jetank_perception/quality_monitoring.hpp:124` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: should_visualize() (129), any_expensive_metrics_enabled() (134), validate() (140-155) and the QualityMetricsConfig flags (81-88) are never used by the node - all metrics run whenever compute_metrics is on, and the visualization publishers are created/used on `quality_config_.visualization.enable` alone (stereo_camera_node.cpp:748, 1205, 1221), so the master `quality_monitoring.enable=false` does not disable visualization (live config has visualization.enable=false only by coincidence). Only the unit tests exercise these helpers. ComputeMetricsConfig.publish_to_topic and CalibrationValidationConfig (99-104) are never read.

Evidence detail: grep: should_visualize/validate/any_expensive_metrics_enabled appear only in the header and test/test_stereo_math.cpp.

Estimated gain: ~35 header lines and ~30 node lines removed; consistent gating

Fix sketch: Keep enable + compute_metrics.{enable,log_interval} + visualization.{disparity_colored,depth_uncertainty} + thresholds; gate visualization on should_visualize(); drop the rest.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-27 — Unused members in StereoCalibration/JetsonStereoNode (image_transport_, E_/F_, getters, base_frame_id_)

`src/jetank_perception/src/stereo_camera_node.cpp:120` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: image_transport_ (120) is declared and never assigned; E_/F_ (90) never written; get_right_camera_matrix/get_left_dist_coeffs/get_right_dist_coeffs (57-59) never called; base_frame_id_ (188) loaded at 485 and never used; camera_config_.fps/format/use_hardware_acceleration (474-477) only appear in log lines; processing_fps_ is std::atomic<double> (191) but read only on the writing thread; processing_mutex_ (143) guards a function that has a single producer per mode; the 2-arg process_stereo_frames overload (1003-1006) exists for one call site.

Evidence detail: grep in session for each symbol shows declaration/assignment only.

Estimated gain: ~20 lines and one header include (image_transport) removed

Fix sketch: Delete the members/getters/overload; make processing_fps_ a plain double; drop the mutex.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-17** — same unused image_transport_ member at stereo_camera_node.cpp:120; see also jetank_perception-33 (CMake/package.xml side); footprint-17 also lists pixi.toml:88 as a place to drop image_transport.

### jetank_perception-29 — Config YAML carries $(find-pkg-share) URLs that rcl never expands plus a stale header

`src/jetank_perception/config/stereo_camera_config.yaml:50` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: calibration.*_url values (50-52) use `$(find-pkg-share ...)` which rclcpp's YAML parameter loader does not substitute; they only work because stereo_camera.launch.py overrides them (154-156). Lines 1-2 ('Save this as:') are template residue. Together with the 24 unread keys (-20) roughly a third of the file has no effect.

Evidence detail: Live get_node_params shows the URLs as absolute file:///.../install_sys/... paths, i.e. the launch override, never the YAML value.

Estimated gain: ~50 YAML lines removed; no misleading keys

Fix sketch: Delete the URL keys from YAML (launch sets them) and the unread keys; keep the file to the parameters that are read.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-30 — Duplicated RANSAC plane setup in remove_ground_plane and remove_ground_height

`src/jetank_perception/src/sock_segmentation_server.cpp:548` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 548-557 and 584-593 are identical SACSegmentation configuration blocks; the two functions differ only in how inliers are used. A shared fit_ground_plane() returning coefficients/inliers removes the duplication; the 'ransac' legacy mode itself (543-570) is documented as producing empty results in practice and could be removed with its ground_filter parameter.

Evidence detail: Textual comparison of the two blocks in session.

Estimated gain: ~15-40 lines removed

Fix sketch: Extract the segmentation setup; consider dropping the ransac mode and ground_filter parameter.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-31 — Raw and rectified publish/compress blocks duplicated verbatim in process_stereo_frames

`src/jetank_perception/src/stereo_camera_node.cpp:1027` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 1027-1049 (raw) and 1061-1083 (rect) are the same 20-line block parameterised only by publishers, frames, encoding and CompressionConfig; the constructor also duplicates the compression log twice (265-287). A publish_pair(cfg, pubL, pubR, cpubL, cpubR, frameL, frameR, enc) helper halves it.

Evidence detail: Textual comparison in session.

Estimated gain: ~30 lines

Fix sketch: Introduce a small helper taking a struct {raw_pub_l, raw_pub_r, comp_pub_l, comp_pub_r, CompressionConfig}.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-32 — Unit tests mainly assert names/defaults of dead code and link full OpenCV+PCL

`src/jetank_perception/test/test_stereo_math.cpp:130` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: CreatesNonNullForEveryStrategyType, SgbmNameReflectsGpuFlag, ConcreteSubclassNames, StereoConfigDefaults and the FPS-math tests exercise get_strategy_name(), unused subclasses (-25) and unused config helpers (-26); building them requires linking ${OpenCV_LIBS} ${PCL_LIBRARIES} (CMakeLists.txt:81) for header-only code. test_reproject.cpp is a real test of reproject_roi and worth keeping.

Evidence detail: Test bodies at 105-161; CMake 76-93.

Estimated gain: Faster `colcon test` link; tests track real behaviour

Fix sketch: Drop the naming/subclass tests along with the dead code; keep QualityMonitoringConfig gating tests only if the helpers survive; link only pcl_common/opencv_core where still needed.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-34 — std_msgs, geometry_msgs linked into stereo_camera_node and launch_xml/launch_yaml exec_depends without use

`src/jetank_perception/CMakeLists.txt:111` · package jetank_perception · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: stereo_camera_node uses std_msgs::msg::Header only through sensor_msgs and includes no geometry_msgs header, yet both are in its ament_target_dependencies (111, 114); package.xml exec_depends launch_xml and launch_yaml (46-47) although all launch files are Python. tf2 is found separately (29) although only tf2_ros/tf2_geometry_msgs are needed by the sock server (tf2::durationFromSec comes via tf2_ros). Each unused find_package costs configure time and pulls transitive packages into the install.

Evidence detail: Includes at stereo_camera_node.cpp 1-34 contain no geometry_msgs/std_msgs headers; launch/ contains only .py.

Estimated gain: Smaller dependency closure / rosdep set; faster configure

Fix sketch: Remove std_msgs and geometry_msgs from the stereo target (keep geometry_msgs for sock_segmentation_server), drop launch_xml/launch_yaml, keep tf2 only if a tf2 symbol is used directly.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-37 — Headers installed with no export, and config install ships unused ost.txt

`src/jetank_perception/CMakeLists.txt:195` · package jetank_perception · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: install(DIRECTORY include/) (195-198) installs five headers but the package neither ament_export_include_directories nor has any downstream includer (grep across src/ found none outside the package). install(DIRECTORY config) (201-202) ships config/calibration/ost.txt, a camera_calibration dump not read by any code, and stereo_calibration.yaml which becomes redundant with -22. scripts/check_quality.sh is neither installed nor referenced.

Evidence detail: grep for 'jetank_perception/' includes outside the package returned only in-package files; ost.txt has no reader in code.

Estimated gain: Smaller install tree; no misleading exported API

Fix sketch: Remove the include/ install rule (or export properly if another package will consume the headers); exclude ost.txt via PATTERN; delete or install the script.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-18** — same include/ install without export; footprint-18 adds that jetank_motor_control/CMakeLists.txt:72-73 installs the un-namespaced motor.hpp the same way (see jetank_motor_control-32).

### jetank_perception-38 — ament_lint_common pulls every linter into colcon test though copyright/cpplint are already disabled

`src/jetank_perception/package.xml:50` · package jetank_perception · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: test_depend ament_lint_common plus ament_lint_auto_find_test_dependencies (CMakeLists.txt:66-74) run cppcheck, uncrustify, flake8, pep257, xmllint and lint_cmake on every `colcon test`, while the copyright and cpplint checks are already switched off (69, 73). On the Orin this adds tens of seconds per test run and a dozen packages to the dev dependency set.

Evidence detail: package.xml 49-51; CMake 65-74.

Estimated gain: Faster `pixi run test`; fewer test deps

Fix sketch: Replace ament_lint_common with the specific linters wanted (e.g. ament_cmake_uncrustify, ament_cmake_cppcheck) or drop lint_auto entirely and keep the two gtests.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-40 — ros_topics input path uses toCvCopy for both frames where toCvShare suffices

`src/jetank_perception/src/stereo_camera_node.cpp:917` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: stereo_image_callback converts the incoming Image messages with cv_bridge::toCvCopy (917-918), copying both frames even when the encoding already matches (mono8/bgr8 from Gazebo). toCvShare (or wrapping msg->data in a cv::Mat header after -14) avoids two full-frame copies per synchronized pair. Sim-only path; no live measurement.

Evidence detail: Lines 906-934.

Estimated gain: Two frame copies per pair removed in simulation mode

Fix sketch: Use toCvShare(msg, target_encoding) and keep the ConstSharedPtr alive for the call; or a zero-copy cv::Mat header over msg->data when encodings match.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-41 — sock_segmentation_server spawns a detached thread per goal and blocks up to 0.4 s on TF lookups

`src/jetank_perception/src/sock_segmentation_server.cpp:196` · package jetank_perception · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: handle_accepted starts a new detached std::thread per goal (196-197); execute() then performs two lookupTransform calls each with a 0.2 s timeout and a retry at latest (686-701), so a goal can block 0.4-0.8 s waiting for TF while holding copies of a ~0.9 MB disparity, camera_info and detections. Using the node's executor (callback group + rclcpp::executors::MultiThreadedExecutor) or a single worker avoids thread creation per goal; using canTransform with the disparity stamp before copying inputs shortens the critical section.

Evidence detail: Lines 194-198, 449-460, 680-702.

Estimated gain: No per-goal thread creation; bounded latency

Fix sketch: Run execute via a reentrant callback group on a multithreaded executor, or a single std::thread worker with a queue; drop the second TF retry or shorten timeouts.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_perception-06 — Point cloud path allocates and copies the full frame 4-5 times per frame before filtering

`src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:86` · package jetank_perception · lens runtime · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: generate_pointcloud(): cv::reprojectImageTo3D allocates a 640x360x3 float temp (2.7 MB), then an organized pcl cloud of 230k PointXYZ (3.7 MB, 16 B/pt) is filled including NaNs (lines 89-110). PointCloudFilter then runs PassThrough (allocates/copies again, line 634), VoxelGrid, StatisticalOutlierRemoval (KD-tree + k-NN, line 643), then pcl::toROSMsg copies to PointCloud2 (stereo_camera_node.cpp:1284) and publish(pc_msg) copies once more (1288). The range gate and NaN removal could be applied in the reprojection loop directly into an unorganized reserved cloud, eliminating reprojectImageTo3D's temp, the NaN points, and the PassThrough pass.

Evidence detail: Code path lines 81-118 (strategy header) and 628-651 (filters), node 1241-1288. Measured points latency p50 141 ms (see -03) is consistent with heavy per-frame work but was not attributed per stage.

Estimated gain: Removes ~6 MB of per-frame allocation/copies and one full filter pass; several ms per frame on Orin Nano

Fix sketch: Loop over disparity rows once, compute Z=Q[2][3]/(Q[3][2]*d+Q[3][3]) etc. inline (or from f,cx,cy,baseline), skip d<=0 and Z outside [min,max], emplace_back into a reserved unorganized cloud (is_dense=true); drop apply_range_filter; publish via std::make_unique<PointCloud2> and toROSMsg into it.

Verifier (confirmed, adjusted low): The code matches the finding: stereo_processing_strategy.hpp:86 allocates the reprojectImageTo3D temp, :89-110 fills a full 640x360 organized cloud including NaNs, PointCloudFilter::filter (:601-612) then runs PassThrough, VoxelGrid and SOR with all three enabled in stereo_camera_config.yaml:67-72, and stereo_camera_node.cpp:1283-1288 does toROSMsg plus a by-value publish. The gain is plausible but modest: the passes eliminated are linear over ~230k points (roughly 1-3 ms each on an Orin Nano CPU), while the dominant cost is likely the KD-tree StatisticalOutlierRemoval with k=30 (:637-644), which the fix does not touch, and the path only runs when a subscriber exists (:1240). "Several ms" against a 141 ms p50 justifies low rather than medium severity.

## jetank_manipulation

Coverage: 25 files read; 3 measurements run. Notes: All source, config, build, launch, action, test and doc files in src/jetank_manipulation were read in full (excluded per constraints: .pytest_cache/ contents and __pycache__). Binary/log/build dirs not present in the package. No node of this package was live, so the read-only measurement tools could only be run against /move_group (external MoveIt binary); every finding is therefore evidence='static'. Cross-package references (jetank_ros_main launch files, jetank_web_control object_hint use, jetank_moveit_config SRDF, jetank_detection interfaces) were checked via grep/partial reads only to confirm which nodes/params are actually used in deployment.

Measurements:
- mcp__ros2-mcp__get_node_list: live nodes include /move_group (x2 entries), /move_group_private_*, /transform_listener_impl_*, /web_control_node, /robot_controller, controller spawners; NO jetank_manipulation node (grasp_server, base_approach_node, mobile_grasp_coordinator, grasp_pose_node) is running, so no runtime claim about this package's own code could be measured — all package findings are static.
- mcp__ros2-mcp__profile_node /move_group 10 s: cpu mean 2.66% / p95 9.9% / peak 39.3%, RSS mean 67.3 MB, 21 threads, 16 fds, 0 ctx switches sampled. /move_group is the jetank_moveit_config binary (the MoveGroup action target of grasp_server), not code from this package; recorded for context only.
- mcp__ros2-mcp__get_node_info /move_group: /move_action and /execute_trajectory action servers present; gripper/arm controller action feedback subscriptions present (controllers inactive per task brief). Confirms /grasp_object and /approach_target action servers are NOT advertised (no package node up).

Findings: 33 (high 0, medium 1, low 32).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_manipulation-01 | minimality | Pose-targeted grasp path is unreachable with default params (t_approach='' -> ValueError -> abort) | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:782` | medium | static | Either ~200 LOC removable, or a 2-line fix (skip approach when empty, as the preset path does at line 671) makes the code reachable | S | confirmed |
| jetank_manipulation-02 | runtime | spin_until_complete busy-polls at 50 Hz for up to 60 s per action call | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:71` | low | static | Removes ~3k wakeups/60 s per waited future; zero CPU while idle | S | **UNVERIFIED** |
| jetank_manipulation-03 | runtime | time.sleep() inside async execute callback blocks an executor thread | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:686` | low | static | Frees executor threads during dwell; enables smaller executor pool (see -07) | S | **UNVERIFIED** |
| jetank_manipulation-04 | runtime | Coordinator always builds+TFs the world grasp pose even in default 'preset' mode where it is never used | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:179` | low (orig medium) | static | Removes PCA + TF lookup (up to 0.5 s block) per trigger in the default path; removes a spurious failure mode | S | confirmed |
| jetank_manipulation-05 | runtime | TransformListener with spin_thread=True spawns an extra thread and hidden node in the coordinator | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:105` | low | static | -1 thread, -1 rcl node, -1 duplicate /tf subscription per coordinator process | S | **UNVERIFIED** |
| jetank_manipulation-06 | runtime | wait_for_server timeouts equal the result timeouts -> missing server blocks a Trigger service for up to 60 s | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:325` | low | static | Fail-fast in ~1-2 s instead of 10-60 s when a server is absent | S | **UNVERIFIED** |
| jetank_manipulation-07 | runtime | MultiThreadedExecutor() defaults to cpu_count threads per node (6 on Orin Nano) for at most 2-3 concurrent callbacks | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/node_runner.py:28` | low | static | ~3-4 fewer threads per node (~8 MB virtual stack each), less contention | S | **UNVERIFIED** |
| jetank_manipulation-09 | minimality | top_down_quaternion + _quat_mul reduce to a closed form (cos(yaw/2), sin(yaw/2), 0, 0) | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:129` | low | static | -30 LOC, ~20 fewer float ops per call | S | **UNVERIFIED** |
| jetank_manipulation-10 | minimality | Orientation-constraint branch and helpers are dead-by-design on the 4-DOF arm | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:271` | low | static | ~-60 LOC in grasp_server, ~-70 LOC tests, -1 parameter | S | **UNVERIFIED** |
| jetank_manipulation-11 | runtime | _moveit_error_name reflects over vars(MoveItErrorCodes) on every call | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:91` | low | static | O(1) dict lookup instead of O(n) reflection per move | S | **UNVERIFIED** |
| jetank_manipulation-12 | runtime | Debug f-string for the pose request is formatted eagerly even when debug logging is off | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:301` | low | static | Removes 1 string build per pose move | S | **UNVERIFIED** |
| jetank_manipulation-13 | minimality | _abort_and_retreat re-implements _publish_stage inline | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:719` | low | static | -3 LOC | S | **UNVERIFIED** |
| jetank_manipulation-14 | minimality | 'ready' SRDF state is dead under the shipped sequence | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:327` | low | static | -6 LOC + test row | S | **UNVERIFIED** |
| jetank_manipulation-15 | minimality | GraspObject.object_hint goal field is never read by grasp_server | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/action/GraspObject.action:2` | low | static | -1 field, -1 line, smaller generated typesupport | S | **UNVERIFIED** |
| jetank_manipulation-16 | minimality | compute_grasp_fields 'dimensions' parameter is documented as unused; callers build tuples just to pass it | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:69` | low | static | -4 LOC, simpler signature | S | **UNVERIFIED** |
| jetank_manipulation-17 | minimality | make_segment_goal publish_debug parameter never passed by any caller | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:51` | low | static | -2 LOC | S | **UNVERIFIED** |
| jetank_manipulation-18 | runtime | cloud_to_xyz makes 3-4 intermediate copies to convert a structured array to (N,3) | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:114` | low | static | ~3 fewer array allocations per cloud, -8 LOC | S | **UNVERIFIED** |
| jetank_manipulation-19 | runtime | pca_long_axis_yaw allocates a per-point norm array only for a degeneracy test, then np.cov recentres again | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:52` | low | static | -1 N-length allocation and one pass over the cloud per call | S | **UNVERIFIED** |
| jetank_manipulation-20 | runtime | Control loop re-imports time and re-reads a parameter every tick; hypot/atan2 computed twice | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/base_approach_node.py:397` | low | static | Fewer per-tick lookups; -3 LOC | S | **UNVERIFIED** |
| jetank_manipulation-21 | minimality | _stop() is called twice on every exit path (explicit + finally), publishing a duplicate zero Twist | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/base_approach_node.py:299` | low | static | -4 LOC, one fewer cmd_vel publish per goal end | S | **UNVERIFIED** |
| jetank_manipulation-22 | minimality | _to_frame catches three subclasses of TransformException plus TransformException itself | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:384` | low | static | -4 LOC | S | **UNVERIFIED** |
| jetank_manipulation-23 | minimality | Latched ~/grasp_pose publisher in coordinator is only used in non-default 'pose' mode | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:130` | low | static | -1 DDS publisher in default mode; -4 LOC if removed | S | **UNVERIFIED** |
| jetank_manipulation-25 | footprint | ament_lint_common / ament_copyright test_depends pull the full C++ linter set into a Python-only package | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:34` | low (orig medium) | static | ~6 fewer test targets per colcon test; fewer resolved test deps in the pixi env | S | confirmed |
| jetank_manipulation-26 | footprint | package.xml declares sensor_msgs, action_msgs, builtin_interfaces, visualization_msgs that the code does not directly use; shape_msgs and numpy are missing | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:18` | low | static | -3/4 dependency edges; correct rosdep set | S | **UNVERIFIED** |
| jetank_manipulation-27 | footprint | Runtime-only Python deps declared as <depend> (build+exec), serialising the colcon build after moveit_msgs/control_msgs/tf2 etc. | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:14` | low | static | Better colcon parallelism; -4 CMake lines | S | **UNVERIFIED** |
| jetank_manipulation-28 | footprint | Node entry-point files are installed twice (module copy + renamed PROGRAMS copy) | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/CMakeLists.txt:31` | low | static | -4 duplicate installed files, ~-20 CMake LOC | S | **UNVERIFIED** |
| jetank_manipulation-29 | footprint | -Wall -Wextra -Wpedantic compile options in a package with no C++ sources apply only to rosidl-generated typesupport | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/CMakeLists.txt:4` | low | static | -3 LOC, quieter generated-code build | S | **UNVERIFIED** |
| jetank_manipulation-30 | minimality | grasp_poses.yaml forces use_sim_time: true under the /** wildcard | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/config/grasp_poses.yaml:47` | low | static | -2 LOC, no hidden sim-time override | S | **UNVERIFIED** |
| jetank_manipulation-31 | minimality | README claims grasp_poses.yaml still carries pre-tuning values; the YAML already has the tuned values | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/README.md:171` | low | static | -6 misleading LOC | S | **UNVERIFIED** |
| jetank_manipulation-32 | runtime | wait_for_server re-queried on every arm move and gripper command within one goal | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:481` | low | static | -4..6 discovery calls per grasp; earlier rejection when move_group is absent | S | **UNVERIFIED** |
| jetank_manipulation-33 | minimality | arm_base_xy list parameter with defensive length fallbacks instead of two scalar params | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:153` | low | static | -3 LOC, one source of truth for the default | S | **UNVERIFIED** |
| jetank_manipulation-08 | minimality | grasp_pose_node is a debug tool not launched anywhere; superseded by mobile_grasp_coordinator | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_pose_node.py:68` | low (orig medium) | static | -243 LOC node, -1 executable, -1 package dependency (visualization_msgs), ~-90 LOC test stubs | M | confirmed |
| jetank_manipulation-24 | minimality | Three test files each carry ~60-95 LOC of duplicated ROS stub scaffolding | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/test/test_import.py:27` | low | static | ~-150 LOC test code | M | **UNVERIFIED** |

### jetank_manipulation-01 — Pose-targeted grasp path is unreachable with default params (t_approach='' -> ValueError -> abort)

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:782` · package jetank_manipulation · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: _run_pose_grasp step 1 calls _move_to_named(p.t_approach, p) but arm_targets.approach defaults to "" (line 406, and config/grasp_poses.yaml line 8). _named_target_request does _SRDF_STATES.get("") -> None -> raises ValueError (line 148) -> _move_to_named returns False -> _abort_and_retreat. So every pose-targeted GraspObject goal aborts before planning under the shipped defaults, and mobile_grasp_coordinator defaults to grasp_mode=preset anyway. The ~200 LOC pose path (lines 164-315, 732-824) plus the pose-request tests are effectively dead in the deployed configuration.

Evidence detail: Read grasp_server.py lines 146-148, 406, 782; grasp_poses.yaml line 8; coordinator line 90. No grasp_server node was live to reproduce.

Estimated gain: Either ~200 LOC removable, or a 2-line fix (skip approach when empty, as the preset path does at line 671) makes the code reachable

Fix sketch: Mirror the preset path: `if p.t_approach and not await self._move_to_named(...)`. If the pose path is not wanted on the 4-DOF arm, delete _pose_target_request/_move_to_pose/_run_pose_grasp/_offset_pose_z/_resolve_approach_height and the pose tests.

Verifier (confirmed, adjusted medium): grasp_server.py:782 calls `_move_to_named(p.t_approach, p)` unconditionally while the preset path guards it with `if p.t_approach:` (line 671); the default is "" (line 406) and config/grasp_poses.yaml:8 also ships `approach: ""`. `_named_target_request` raises ValueError for an unknown state (lines 146-148), `_move_to_named` catches it and returns False (lines 632-634), so line 782 falls into `_abort_and_retreat` for every pose-targeted goal (dispatched at lines 529-538) under shipped defaults. mobile_grasp_coordinator.py:90 defaults `grasp_mode` to "preset", so the pose path is not exercised by the deployed pipeline either. Severity stays medium: it is a real latent bug/dead path, but a one-line guard fixes it.

### jetank_manipulation-02 — spin_until_complete busy-polls at 50 Hz for up to 60 s per action call

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:71` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The wait loop sleeps 20 ms and calls node.get_clock().now() twice per iteration (each a C-extension call with lock). With grasp_timeout_s=60 that is up to 3000 wakeups and 6000 clock calls per step, on an executor thread that is held for the whole duration. Used by grasp_pose_node and mobile_grasp_coordinator for segment/approach/grasp (up to 3 x 2 futures per trigger).

Evidence detail: Lines 70-76; callers at grasp_pose_node.py:143, mobile_grasp_coordinator.py:280/308/336. No coordinator node was live to measure.

Estimated gain: Removes ~3k wakeups/60 s per waited future; zero CPU while idle

Fix sketch: ev = threading.Event(); future.add_done_callback(lambda f: ev.set()); return ev.wait(timeout_s) and future.done() — no polling, wall-clock timeout.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-03 — time.sleep() inside async execute callback blocks an executor thread

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:686` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _execute_cb is a coroutine but calls time.sleep(p.dwell_open)/time.sleep(p.dwell_close) (lines 686, 697, 795, 806) and the blocking wait_for_server (lines 481, 593). Each blocks one MultiThreadedExecutor thread for 0.2-0.8 s (dwell) or up to 10 s (wait_for_server) per move, defeating the point of the coroutine and reducing threads available for cancel/feedback handling.

Evidence detail: Lines 481, 593, 686, 697, 795, 806 read this session.

Estimated gain: Frees executor threads during dwell; enables smaller executor pool (see -07)

Fix sketch: Make the dwell an awaited one-shot timer future (create_timer + Future) or move the sleep into the gripper result wait; check wait_for_server once per goal instead of per move.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-04 — Coordinator always builds+TFs the world grasp pose even in default 'preset' mode where it is never used

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:179` · package jetank_manipulation · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: SEGMENT (lines 179-193) runs cloud_to_xyz + PCA + quaternion + a 0.5 s-timeout tf2 transform into odom and FAILS THE WHOLE RUN if TF is unavailable, but _grasp_pose_world is only consumed in the grasp_mode=='pose' branch (line 229). With the default grasp_mode='preset' this is wasted CPU/latency and an unnecessary failure mode (a missing odom->base_link TF aborts a preset grasp that does not need it).

Evidence detail: Lines 90, 179-193, 216-242 read; grasp_mode default 'preset' (line 90) and no launch file overrides it (jetank_ros_main mobile_grasp*.launch.py pass only use_sim_time).

Estimated gain: Removes PCA + TF lookup (up to 0.5 s block) per trigger in the default path; removes a spurious failure mode

Fix sketch: Read grasp_mode first; only call _make_grasp_pose/_to_frame when grasp_mode=='pose'.

Verifier (confirmed, adjusted low): Fix is safe and in-scope: `_grasp_pose_world` (set at mobile_grasp_coordinator.py:187) is only read at :229 inside the `grasp_mode != "preset"` branch, and `_pose_pub` (:130) publishes only at :241 in that same branch, so gating :179-193 on grasp_mode changes nothing observable in preset mode except dropping one info log and the TF-unavailable abort at :181-186. No other package reads `_grasp_pose_world`, `~/grasp_pose` from this node, or the `grasp_mode` param (workspace grep: only README/plan docs mention the coordinator by service name); the change is intra-file in jetank_manipulation and touches no perception strategy/factory. Severity adjusted to low: the PCA on a single sock cloud is milliseconds and the 0.5 s tf2 block (:381) only occurs when odom->base_link TF is actually missing, which on a nav-enabled robot is uncommon; the spurious failure mode is the real (small) win.

### jetank_manipulation-05 — TransformListener with spin_thread=True spawns an extra thread and hidden node in the coordinator

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:105` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: tf2_ros.TransformListener(self._tf_buffer, self) uses the default spin_thread=True, which creates a dedicated executor thread plus a private 'transform_listener_impl_*' node subscribing /tf and /tf_static. The node already spins on a MultiThreadedExecutor (node_runner), so base_approach_node correctly passes spin_thread=False (line 166). The coordinator only needs TF once per trigger (and, per -04, not at all in preset mode).

Evidence detail: Coordinator line 105 vs base_approach_node.py line 164-167.

Estimated gain: -1 thread, -1 rcl node, -1 duplicate /tf subscription per coordinator process

Fix sketch: TransformListener(self._tf_buffer, self, spin_thread=False).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-06 — wait_for_server timeouts equal the result timeouts -> missing server blocks a Trigger service for up to 60 s

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:325` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _segment/_approach/_grasp call wait_for_server(timeout_sec=timeout_s) with 10/30/60 s, then send_goal_and_wait uses the same timeout twice (goal acceptance + result). A missing grasp server therefore holds the service callback (and an executor thread) for 60 s before returning success=false; grasp_pose_node.py:127 has the same pattern (10 s). Server presence is a discovery check that resolves in well under a second.

Evidence detail: Coordinator lines 267, 291, 325; action_utils.py lines 87-99; grasp_pose_node.py line 127.

Estimated gain: Fail-fast in ~1-2 s instead of 10-60 s when a server is absent

Fix sketch: Use a short dedicated server_wait_s (e.g. 2.0) for wait_for_server; keep timeout_s for the result only.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-07 — MultiThreadedExecutor() defaults to cpu_count threads per node (6 on Orin Nano) for at most 2-3 concurrent callbacks

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/node_runner.py:28` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: All four executables share spin_node, which builds MultiThreadedExecutor() with no num_threads -> os.cpu_count() worker threads each. The nodes need one thread for the long-running action/service callback plus one to service action-client futures/TF; the remaining threads idle but cost stack memory and scheduler entries. Three nodes launched together -> ~18 idle-capable threads.

Evidence detail: node_runner.py line 28; rclpy MultiThreadedExecutor default num_threads=None -> cpu_count. No package node live to count threads.

Estimated gain: ~3-4 fewer threads per node (~8 MB virtual stack each), less contention

Fix sketch: MultiThreadedExecutor(num_threads=3) in spin_node (or a per-node argument).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-09 — top_down_quaternion + _quat_mul reduce to a closed form (cos(yaw/2), sin(yaw/2), 0, 0)

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:129` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: q_roll is the constant (1,0,0,0) (lines 145-146 recompute cos/sin(pi/2) every call). The Hamilton product q_yaw*(1,0,0,0) is analytically (cos(yaw/2), sin(yaw/2), 0, 0). The 34-line function pair (129-162) can be 3 lines with identical results (tests at test_grasp_pose.py 249-276 still pass).

Evidence detail: Derived from _quat_mul lines 157-161 with q2=(1,0,0,0).

Estimated gain: -30 LOC, ~20 fewer float ops per call

Fix sketch: def top_down_quaternion(yaw): h=yaw/2; return (math.cos(h), math.sin(h), 0.0, 0.0)

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-10 — Orientation-constraint branch and helpers are dead-by-design on the 4-DOF arm

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:271` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 60-76 (_ORI_TOL_LOOSE constants), 99-115 (_normalized_quaternion), 271-299 (OrientationConstraint block), the ori_tol_x/y/z/ee_link/position_box_size keyword params (171-176, never passed by any caller), the pose_grasp.pose_use_orientation parameter (420) and ~70 LOC of tests only execute when pose_use_orientation=True, which config (yaml lines 39-44) and code comments say must stay False because KDL IK cannot solve it. Combined with -01 this is dead code guarding dead code.

Evidence detail: Callers of _pose_target_request: grasp_server.py 646-655 and test_import.py 371-377; neither passes ee_link/position_box_size/ori_tol_*.

Estimated gain: ~-60 LOC in grasp_server, ~-70 LOC tests, -1 parameter

Fix sketch: Remove include_orientation and the orientation block; hard-code position-only. Drop unused keyword params and the _ORI_TOL_LOOSE constant.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-11 — _moveit_error_name reflects over vars(MoveItErrorCodes) on every call

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:91` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Each move result iterates all attributes of the generated message class (dozens of entries incl. slots/methods) with isupper()/isinstance checks. Called once per arm move (5-7 per grasp). Cheap in absolute terms but trivially cacheable.

Evidence detail: Lines 90-96.

Estimated gain: O(1) dict lookup instead of O(n) reflection per move

Fix sketch: _ERR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)} at module import; return _ERR_NAMES.get(val, 'UNKNOWN').

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-12 — Debug f-string for the pose request is formatted eagerly even when debug logging is off

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:301` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 301-312 build a multi-part f-string (7 float formats) and call logger.debug on every pose request; rclpy evaluates the argument before checking the severity threshold, so at the default INFO level this is pure waste. Minor because the pose path runs at most 3 times per grasp.

Evidence detail: Lines 301-312.

Estimated gain: Removes 1 string build per pose move

Fix sketch: Guard with `if logger.is_enabled_for(LoggingSeverity.DEBUG)` or drop the dump.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-13 — _abort_and_retreat re-implements _publish_stage inline

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:719` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 719-721 build GraspObject.Feedback and publish it manually; _publish_stage (578-583) does exactly this plus logging. Duplicate of a 3-line helper defined 140 lines earlier.

Evidence detail: Lines 578-583 vs 719-721.

Estimated gain: -3 LOC

Fix sketch: self._publish_stage(goal_handle, 'aborting_retreat').

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-14 — 'ready' SRDF state is dead under the shipped sequence

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:327` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _SRDF_STATES['ready'] is only reachable if arm_targets.approach/retreat is set to 'ready'; defaults (406, 409) and grasp_poses.yaml (8, 11) are empty, and the speed-optimisation plan explicitly removed it. The entry plus README/test rows (test_import.py 160) are maintained for a state the code no longer uses.

Evidence detail: grep '"ready"' in package sources: only the table entry and comments.

Estimated gain: -6 LOC + test row

Fix sketch: Remove 'ready' from _SRDF_STATES and EXPECTED_STATES (or keep only if a launch config actually sets it).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-15 — GraspObject.object_hint goal field is never read by grasp_server

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/action/GraspObject.action:2` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: object_hint is documented as 'unused in Phase 1'; the pose-targeted path (Phase 7) landed without using it. jetank_web_control sets it (web_control_node.py 885) but grasp_server never reads goal_handle.request.object_hint. Unused interface field serialized on every goal and a coupling for clients.

Evidence detail: grep object_hint in grasp_server.py: no matches.

Estimated gain: -1 field, -1 line, smaller generated typesupport

Fix sketch: Remove the field from GraspObject.action and the assignment in jetank_web_control (coordinate the interface change).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-16 — compute_grasp_fields 'dimensions' parameter is documented as unused; callers build tuples just to pass it

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:69` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Docstring lines 86-88 admit dimensions is unused. grasp_pose_node.py 165 and mobile_grasp_coordinator.py 357 each construct a dims tuple solely to satisfy the signature. Speculative-generality API surface.

Evidence detail: grasp_math.py 66-126: 'dimensions' never referenced in the body.

Estimated gain: -4 LOC, simpler signature

Fix sketch: Drop the parameter; add it back when a heuristic actually needs it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-17 — make_segment_goal publish_debug parameter never passed by any caller

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:51` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Both callers (grasp_pose_node.py 136, coordinator 274-278) omit publish_debug, so the parameter and its bool() cast are unused surface.

Evidence detail: grep publish_debug in package: only the helper definition.

Estimated gain: -2 LOC

Fix sketch: Remove the parameter (goal.publish_debug defaults to False in the message).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-18 — cloud_to_xyz makes 3-4 intermediate copies to convert a structured array to (N,3)

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:114` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: np.asarray(structured) then three np.asarray(arr[field], dtype=float64) casts and a column_stack produce four temporaries. numpy.lib.recfunctions.structured_to_unstructured does the same in one pass. Sock blobs are small (hundreds-thousands of points) so the absolute cost is low, but it runs on every segmentation result.

Evidence detail: Lines 105-126.

Estimated gain: ~3 fewer array allocations per cloud, -8 LOC

Fix sketch: from numpy.lib.recfunctions import structured_to_unstructured; return structured_to_unstructured(arr[['x','y','z']], dtype=np.float64).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-19 — pca_long_axis_yaw allocates a per-point norm array only for a degeneracy test, then np.cov recentres again

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:52` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 52-54 compute centered and an N-vector of norms just to check any spread > 1e-9; np.cov (line 57) then subtracts the mean again internally. A ptp-based check (np.ptp(pts, axis=0).max() > 1e-9) or checking the covariance trace avoids the extra N-length allocation and the double centering.

Evidence detail: Lines 48-63.

Estimated gain: -1 N-length allocation and one pass over the cloud per call

Fix sketch: cov = np.cov(pts, rowvar=False); if not np.isfinite(cov).all() or np.trace(cov) < 1e-18: return None.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-20 — Control loop re-imports time and re-reads a parameter every tick; hypot/atan2 computed twice

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/base_approach_node.py:397` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _sleep_period does `import time` inside the function (sys.modules lookup each 10 Hz tick, line 397); _stop() (383-385) calls get_parameter('base_frame') on every TF-gap tick and every exit; approach_control already computes hypot/atan2 (113-114) and the caller recomputes both (274-275). All small, all in the 10 Hz loop.

Evidence detail: Lines 274-275, 383-385, 397-399.

Estimated gain: Fewer per-tick lookups; -3 LOC

Fix sketch: Move `import time` to module top; pass base_frame into _stop; have approach_control return dist/heading or compute once and pass in.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-21 — _stop() is called twice on every exit path (explicit + finally), publishing a duplicate zero Twist

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/base_approach_node.py:299` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 237, 249, 283 call self._stop() before returning, and the finally block (299) calls it again; line 302 after the loop calls it a third time before the finally runs. Every goal end therefore publishes two zero TwistStamped messages and the explicit calls are redundant LOC.

Evidence detail: Lines 233-306.

Estimated gain: -4 LOC, one fewer cmd_vel publish per goal end

Fix sketch: Keep only the finally: self._stop(); delete the inline calls.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-22 — _to_frame catches three subclasses of TransformException plus TransformException itself

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:384` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: LookupException, ConnectivityException and ExtrapolationException all derive from tf2_ros.TransformException, so the 4-entry tuple is equivalent to `except TransformException` (as base_approach_node.py 342 already does).

Evidence detail: Lines 384-389.

Estimated gain: -4 LOC

Fix sketch: except tf2_ros.TransformException as exc:

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-23 — Latched ~/grasp_pose publisher in coordinator is only used in non-default 'pose' mode

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:130` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _pose_pub (130-132) is created on every start but published only at line 241 inside the grasp_mode=='pose' branch. In the default preset mode it is an unused transient-local publisher (DDS endpoint + discovery traffic).

Evidence detail: Lines 130-132, 241.

Estimated gain: -1 DDS publisher in default mode; -4 LOC if removed

Fix sketch: Create lazily in the pose branch, or remove together with -08's grasp_pose_node if RViz debug output is not needed.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-25 — ament_lint_common / ament_copyright test_depends pull the full C++ linter set into a Python-only package

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:34` · package jetank_manipulation · lens footprint · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: test_depend on ament_lint_common (transitively ament_cppcheck, ament_cpplint, ament_uncrustify, ament_lint_cmake, ament_xmllint, ament_flake8, ament_pep257) and ament_copyright, with CMakeLists 80-81 then hacking *_FOUND=TRUE to suppress copyright/xmllint. ament_lint_auto_find_test_dependencies (line 82) still registers cppcheck/cpplint/uncrustify/lint_cmake/flake8/pep257 tests that run on every `colcon test` for a package with zero C++ sources.

Evidence detail: package.xml lines 34-40; CMakeLists.txt lines 78-88.

Estimated gain: ~6 fewer test targets per colcon test; fewer resolved test deps in the pixi env

Fix sketch: Drop ament_copyright/ament_lint_auto/ament_lint_common test_depends and the ament_lint_auto block; keep ament_cmake_pytest + python3-pytest (optionally ament_flake8 as a single explicit ament_flake8() call).

Verifier (confirmed, adjusted low): package.xml:34-40 declares ament_copyright, ament_lint_auto and ament_lint_common as test_depends, and CMakeLists.txt:79-82 runs ament_lint_auto_find_test_dependencies() after forcing copyright/xmllint _FOUND=TRUE; the package contains no C++ sources (find for .cpp/.hpp/.h/.c returned nothing, only .py files), so cppcheck/cpplint/uncrustify tests are registered for nothing. However, the pixi-env dependency-weight claim is overstated: all 9 workspace packages test_depend on ament_lint_common (grep), so dropping it here removes no resolved packages; the real gain is only the ~3-4 pointless test targets (lint_cmake is still legitimately useful), hence low severity.

### jetank_manipulation-26 — package.xml declares sensor_msgs, action_msgs, builtin_interfaces, visualization_msgs that the code does not directly use; shape_msgs and numpy are missing

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:18` · package jetank_manipulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: sensor_msgs (18) is never imported (only sensor_msgs_py, which already depends on it); action_msgs (26) is never imported (rclpy.action brings it); builtin_interfaces (25) is only needed transitively via geometry_msgs' Header; visualization_msgs (21) is used only by the unlaunched grasp_pose_node (-08). Conversely grasp_server.py line 42 imports shape_msgs and grasp_math/action_utils import numpy, neither declared (python3-numpy exec_depend, shape_msgs exec_depend). rosdep/colcon build ordering is wider than needed and the missing deps would break a clean rosdep install.

Evidence detail: grep of imports across jetank_manipulation/*.py (this session) vs package.xml lines 14-30.

Estimated gain: -3/4 dependency edges; correct rosdep set

Fix sketch: Remove sensor_msgs, action_msgs, builtin_interfaces (keep it only if the rosidl DEPENDENCIES list keeps it), visualization_msgs (after -08); add <exec_depend>shape_msgs</exec_depend> and <exec_depend>python3-numpy</exec_depend>.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-29** — same shape_msgs/numpy undeclared, sensor_msgs/action_msgs redundant.

### jetank_manipulation-27 — Runtime-only Python deps declared as <depend> (build+exec), serialising the colcon build after moveit_msgs/control_msgs/tf2 etc.

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/package.xml:14` · package jetank_manipulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: rclpy, std_srvs, control_msgs, moveit_msgs, sensor_msgs_py, visualization_msgs, tf2_ros, tf2_geometry_msgs, jetank_detection are only imported at runtime by Python; nothing at CMake time needs them except geometry_msgs (rosidl DEPENDENCIES). Declaring them as <depend> forces colcon to build this package after all of them, reducing build parallelism. Likewise CMakeLists line 16 find_package(jetank_detection) is redundant: package.xml already orders the build, and the find_package only adds configure-time cost.

Evidence detail: package.xml lines 14-26; CMakeLists.txt lines 13-16; only geometry_msgs/builtin_interfaces are referenced by rosidl_generate_interfaces (23-27).

Estimated gain: Better colcon parallelism; -4 CMake lines

Fix sketch: Change the runtime-only entries to <exec_depend>; keep <depend>geometry_msgs</depend> and buildtool deps; remove find_package(jetank_detection).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-28 — Node entry-point files are installed twice (module copy + renamed PROGRAMS copy)

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/CMakeLists.txt:31` · package jetank_manipulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: install(DIRECTORY jetank_manipulation/ ... *.py) copies grasp_server.py, grasp_pose_node.py, base_approach_node.py and mobile_grasp_coordinator.py into site-packages, and four separate install(PROGRAMS ... RENAME) blocks (38-67) copy the same files again into lib/jetank_manipulation. Four duplicated ~10-30 KB files and 30 LOC of near-identical CMake. A tiny launcher stub per executable (or one foreach) would avoid the duplication.

Evidence detail: CMakeLists.txt lines 29-67.

Estimated gain: -4 duplicate installed files, ~-20 CMake LOC

Fix sketch: foreach(_exe grasp_server grasp_pose_node base_approach_node mobile_grasp_coordinator) install(PROGRAMS jetank_manipulation/${_exe}.py DESTINATION lib/${PROJECT_NAME} RENAME ${_exe}) endforeach(), and exclude those files from the DIRECTORY install (or install thin wrappers).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-29 — -Wall -Wextra -Wpedantic compile options in a package with no C++ sources apply only to rosidl-generated typesupport

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/CMakeLists.txt:4` · package jetank_manipulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The package has zero hand-written C/C++; add_compile_options is inherited by the rosidl-generated C/C++ typesupport targets, adding warning noise (generated code is not fixable here) with no benefit. Template boilerplate.

Evidence detail: find in package: no .cpp/.hpp files; CMakeLists lines 4-6.

Estimated gain: -3 LOC, quieter generated-code build

Fix sketch: Delete the add_compile_options block.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-30 — grasp_poses.yaml forces use_sim_time: true under the /** wildcard

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/config/grasp_poses.yaml:47` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The /** namespace means any node that loads this file gets use_sim_time=true; grasp.launch.py (39) and mobile_grasp_hw.launch.py (138) both then override it with a launch argument, so the YAML value is either redundant or a hardware foot-gun depending on parameter order. Over-general config.

Evidence detail: yaml lines 1, 46-47; grasp.launch.py 37-40; jetank_ros_main/launch/mobile_grasp_hw.launch.py 137-139 (grep output).

Estimated gain: -2 LOC, no hidden sim-time override

Fix sketch: Remove use_sim_time from the YAML; let launch own it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-31 — README claims grasp_poses.yaml still carries pre-tuning values; the YAML already has the tuned values

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/README.md:171` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: README 171-176 says the YAML has approach/retreat: grasp_pre, scaling 0.3, planning 5.0 s x 3 attempts and dwell 0.5 s, and tells users to pass overrides. grasp_poses.yaml lines 8-33 already contain the speed-optimised values (empty approach/retreat, 0.6/0.5, 1.5 s, 1 attempt, 0.2 s). Stale guidance that would make users add needless overrides.

Evidence detail: README lines 171-176 vs grasp_poses.yaml lines 7-33.

Estimated gain: -6 misleading LOC

Fix sketch: Delete the blockquote.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-32 — wait_for_server re-queried on every arm move and gripper command within one goal

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:481` · package jetank_manipulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _execute_move_request (481) and _command_gripper (593) each call wait_for_server before every send, so a preset grasp performs 5-7 graph-discovery checks per goal, and the init log at 453-455 says it is waiting for /move_action while nothing waits. One check at goal start (or in the goal_callback to REJECT early) is sufficient.

Evidence detail: Lines 453-455, 481, 593; preset sequence 662-715 issues 3 arm moves + 2 gripper commands minimum.

Estimated gain: -4..6 discovery calls per grasp; earlier rejection when move_group is absent

Fix sketch: Check both servers once in _goal_cb (return GoalResponse.REJECT if absent) and drop the per-call waits.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-33 — arm_base_xy list parameter with defensive length fallbacks instead of two scalar params

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:153` · package jetank_manipulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 153-155 copy the list, then guard len()>0 / len()>1 with hard-coded fallbacks (0.06, 0.0) duplicating the declared default at line 85. Two float parameters (arm_base_x, arm_base_y) remove the list copy and the duplicated defaults.

Evidence detail: Lines 85, 153-155.

Estimated gain: -3 LOC, one source of truth for the default

Fix sketch: declare arm_base_x / arm_base_y floats and read them directly.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_manipulation-08 — grasp_pose_node is a debug tool not launched anywhere; superseded by mobile_grasp_coordinator

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_pose_node.py:68` · package jetank_manipulation · lens minimality · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: No launch file in the workspace starts grasp_pose_node (jetank_ros_main mobile_grasp.launch.py, mobile_grasp_hw.launch.py, sim_demo.launch.py launch only grasp_server/base_approach_node/coordinator). README line 28 calls it 'also packaged'. Its grasp logic is already in grasp_math/action_utils; the node adds 243 LOC, a CMake install rule (CMakeLists 45-51), the visualization_msgs dependency, and the re-export shim (lines 53-60) that exists only so test_grasp_pose.py can reach grasp_math through it.

Evidence detail: grep for grasp_pose_node/plan_sock_grasp outside the package found only README/tests; launch files read this session.

Estimated gain: -243 LOC node, -1 executable, -1 package dependency (visualization_msgs), ~-90 LOC test stubs

Fix sketch: Delete grasp_pose_node.py + its install rule; point test_grasp_pose.py at grasp_math directly (pure numpy, no stubs needed); drop visualization_msgs from package.xml. If a debug pose publisher is wanted, publish the latched ~/grasp_pose from the coordinator (it already has _pose_pub).

Verifier (confirmed, adjusted low): Evidence supports the finding: no launch file starts grasp_pose_node (jetank_manipulation/launch/grasp.launch.py:34 and jetank_ros_main/launch/mobile_grasp*.launch.py:101-146 only launch grasp_server/base_approach_node/mobile_grasp_coordinator), the file is 243 LOC (wc), visualization_msgs Marker is used only there (package.xml:21 depend would become dead), and test/test_grasp_pose.py:79-139 loads the node solely to reach grasp_math re-exports (grasp_pose_node.py:53-60), needing ~20 stub lines for rclpy/visualization_msgs/jetank_detection. The LOC/dependency gain is plausible; runtime gain on the Jetson is zero since the node never runs, so this is a build/maintenance-weight finding only, which justifies low rather than medium severity.

### jetank_manipulation-24 — Three test files each carry ~60-95 LOC of duplicated ROS stub scaffolding

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/test/test_import.py:27` · package jetank_manipulation · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: test_import.py 27-121, test_grasp_pose.py 27-124 and test_base_control.py 25-94 each define _make_stub, rclpy/geometry_msgs/jetank_manipulation.action stubs and file-path module loading. A single conftest.py fixture would replace ~250 LOC with ~80. grasp_math and approach_control are pure functions and need no stubbing at all if imported directly.

Evidence detail: All three test files read in full this session.

Estimated gain: ~-150 LOC test code

Fix sketch: Add test/conftest.py with one _install_stubs + load_source(name) helper; import grasp_math/approach_control directly where the module is ROS-free.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_web_control

Coverage: 24 files read; 14 measurements run. Notes: All source, config, build, launch, static web and test files of jetank_web_control were read in full in this session (package has no CMakeLists.txt, YAML config, or xacro; it is ament_python). The .pytest_cache/v/cache/nodeids file was only previewed (first 600 bytes) since it is a generated cache, and lastfailed/stepwise are empty. build/, install/, log/ and .pixi/ were not read per constraints (only an ls of the install share/static layout to confirm symlink-install and that the running executable is the installed copy). The rclpy executor source was not consulted (it lives under .pixi/), so the per-wakeup executor cost is attributed empirically via the isolated scratch-node benchmark rather than by reading rclpy internals. cmd_vel_bridge is sim-only and was not running, so its findings are static. The MCP hz/bw tools materially undercount the 30 fps camera topic; the stream capture from the node itself was used as ground truth and this discrepancy is recorded in measurements. Two cross-package files (jetank_ros_main/launch/mobile_grasp_hw.launch.py excerpt, jetank_navigation slam_toolbox.yaml grep) were consulted only to support findings 03 and 05.

Measurements:
- mcp__ros2-mcp__get_node_list: /web_control_node present among 16 nodes (ROS_DOMAIN_ID=42, rmw_fastrtps_cpp, ROS_DISTRO=humble, rclpy 3.3.21)
- mcp__ros2-mcp__get_node_info(/web_control_node): pubs /cmd_vel,/initialpose; subs /amcl_pose,/detections/socks,/map,/stereo_camera/left/image_raw/compressed + action feedback/status subs for /grasp_object and /navigate_to_pose
- mcp__ros2-mcp__get_node_params(/web_control_node): image_compressed=true, sim=false, cmd_vel_topic=/cmd_vel, web_port=8080 (defaults otherwise)
- mcp__ros2-mcp__profile_node(/web_control_node,10s): cpu mean 7.5% p95 9.9% peak 19.6%; RSS 96.0 MB; 12 threads; 20 fds; 0 HTTP clients connected (ss -tn showed only LISTEN)
- /proc per-thread sampling (PID 8258): spin thread 8397 = 6.2% CPU (31 ticks/5s), 84 voluntary ctx-switches/s; asyncio main thread 8258 = 0 ticks idle; DDS threads 8390/8393 = 69/39 wakeups/s
- mcp__ros2-mcp__get_topic_hz(/stereo_camera/left/image_raw/compressed): 0.69 Hz (5s), 3.79-3.88 Hz (10s x3) -- UNDERCOUNTS: a 10 s curl of /stream.mjpg delivered 294 distinct parts of ~144 KB (29.4 fps, 42.6 MB), so the true topic rate is ~30 Hz / ~4.2 MB/s; get_topic_bw reported 1.32-2.45 MB/s with an implausible 647 KB/msg -- treat MCP hz/bw for this topic as unreliable
- mcp__ros2-mcp__measure_topic_perf(image compressed,10s): count 37, jitter 11 ms, drop_estimate 0, header-stamp latency p50 324 ms (subject to the same undercount)
- mcp__ros2-mcp__get_topic_hz(/cmd_vel,5s): 0.0 Hz (no browser connected -> watchdog silent as designed); get_topic_hz(/detections/socks): 0.0 Hz (no detector running)
- ros2 topic info -v (pixi, read-only): image topic pub/sub both RELIABLE/VOLATILE; /map has 0 publishers now; /cmd_vel: web_control_node -> robot_controller RELIABLE
- Isolated scratch rclpy node (own process, no publishing) on the live graph, spin-thread CPU / wakeups per 8 s window: idle 0.00%/0; timer(10Hz) 0.87%/11.2; 4 subs 0.00%/0; timer+subs 1.25%/17.9; timer+subs+2 action clients 1.50%/22.2; +image sub 7.12%/115.5; image sub only 4.87-5.12%/69-78; image raw=True 3.87%/56.9; image best-effort 5.25%/73.7; image raw+best-effort 3.62%/60.7
- Live read-only HTTP load (curl, GET only): browser poll set (/map_meta,/map.png(404),/nav_status,/robot_pose) at 1 Hz for 10 s = 8 main-thread ticks (0.8% of a core, ~2 ms/request); /captures+/detections/latest x20 each = 8 ticks; one MJPEG client 10 s = 22 main-thread ticks (2.2%), spin thread 66 ticks (6.6%) in the same window
- Offline micro-benchmarks (pixi env, Orin Nano): bytes(array 660KB) 80 us; map PNG 400x400 RGB optimize=True 221.5 ms/57.9 KB, optimize=False 29.6 ms/60.7 KB, mode L optimize=True 132.5 ms/46.4 KB, mode L optimize=False 15.5 ms/47.5 KB; PIL JPEG 1280x720 q80 16.0 ms, from bgr[:,:,::-1] 21.5 ms, cv2.imencode 13.9 ms; import times numpy 207 ms, PIL.Image 95 ms, cv2 610 ms, aiohttp 417 ms; maxrss deltas for numpy/PIL/aiohttp imports ~1 MB each
- CDR layout check: serialize_message(CompressedImage frame_id='camera_left', format='jpeg') hex dump shows data length field at byte 40 and payload at 44 (4-byte encapsulation, 8-byte stamp, 4-aligned strings) -- basis for the raw=True slice in finding 02; my offline raw_jpeg assertion failed only because it compared bytes to array.array
- FAILED/UNAVAILABLE: py-spy not installed (no stack sampling); /proc/<pid>/task/<tid>/sched absent on this Tegra kernel (used status ctxt_switches instead); no /map publisher, no detector and no browser client were live, so map-encode, detection and browser-side claims rely on offline benchmarks or static reading

Findings: 33 (high 1, medium 2, low 30).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_web_control-01 | runtime | Camera subscription is always active: node deserializes ~30 fps of JPEG frames with zero viewers | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:493` | high | measured | ~5-6% of a core saved whenever no browser is streaming (the common case on the robot); fewer DDS reader buffers held | M | confirmed |
| jetank_web_control-03 | runtime | Map PNG encoded with optimize=True in the ROS spin thread stalls teleop for ~200 ms per map update | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:587` | medium | measured | 14x faster map render (221 ms -> ~15 ms); removes 0.2-1 s executor stalls every 5 s during mapping; smaller PNG too | S | confirmed |
| jetank_web_control-05 | runtime | Browser send loop streams zero-velocity commands at 10 Hz forever, defeating the node's silent-when-idle watchdog | `/home/koen/workspaces/ros2_ws/static/app.js:211` | medium | static | Removes 10 msg/s WS + 10 publishes/s per open tab while idle; restores the intended idle-silence on /cmd_vel | S | confirmed |
| jetank_web_control-06 | runtime | MJPEG handler polls at a fixed 30 Hz per client instead of waking on new frames | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1412` | low | measured | Removes up to 30 empty wakeups/s per client when the camera is slower than 30 fps; 1 fewer syscall per frame | S | **UNVERIFIED** |
| jetank_web_control-07 | runtime | 10 Hz watchdog timer runs permanently even when idle and silent | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:502` | low | measured | ~0.9% of a core whenever no client is driving | S | **UNVERIFIED** |
| jetank_web_control-09 | runtime | Desktop page polls four endpoints every second forever, even with no nav stack and no map | `/home/koen/workspaces/ros2_ws/static/app.js:372` | low | measured | ~0.8% of a core per idle desktop tab; 4 fewer HTTP round-trips/s | S | **UNVERIFIED** |
| jetank_web_control-11 | runtime | Open MJPEG stream keeps running while the tab is hidden or the Annotate panel is open | `/home/koen/workspaces/ros2_ws/static/index.html:26` | low | measured | 2.2% of a core + 4.2 MB/s per hidden/idle tab | S | **UNVERIFIED** |
| jetank_web_control-34 | footprint | numpy and PIL are imported eagerly at module import though only the map and sim-image paths use them | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:71` | low | measured | ~300 ms faster startup on the robot; ~2-3 MB RSS | S | **UNVERIFIED** |
| jetank_web_control-02 | runtime | _on_image fully deserializes each CompressedImage then copies it; a raw=True subscription with a CDR slice avoids both | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:539` | low (orig medium) | measured | ~1% of a core at 30 fps while streaming; removes one 144 KB copy per frame | M | confirmed |
| jetank_web_control-04 | runtime | Sim raw-Image path JPEG-encodes every frame with PIL in the spin thread, with an extra channel-flip copy for bgr8 | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:570` | low (orig medium) | measured | ~50% of a core in sim while nobody is watching; 25-35% less per frame when watching (cv2, no flip) | M | confirmed |
| jetank_web_control-08 | runtime | Two always-on ActionClients add 10 wait-set entities that are polled on every executor iteration | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:425` | low | measured | ~0.25% of a core idle; smaller wait set | M | **UNVERIFIED** |
| jetank_web_control-10 | runtime | Map PNG re-downloaded every second with a cache-busting query even when the map has not changed | `/home/koen/workspaces/ros2_ws/static/app.js:502` | low | static | ~80% fewer map transfers during mapping, 100% fewer in AMCL mode | S | **UNVERIFIED** |
| jetank_web_control-12 | runtime | GET /captures opens and parses every label sidecar in the dataset on each request | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:675` | low | static | Avoids N file reads + parses per open; keeps the loop responsive with large datasets | S | **UNVERIFIED** |
| jetank_web_control-13 | runtime | Labeller redraw performs 4 layout reads per box per mouse-move | `/home/koen/workspaces/ros2_ws/static/app.js:938` | low | static | Constant 2 layout reads per redraw instead of 4N; smoother dragging with many boxes | S | **UNVERIFIED** |
| jetank_web_control-14 | runtime | _launch_nav forks 11 sequential pkill processes and then sleeps a fixed 1 s before every nav start | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1024` | low | static | ~1 s + 10 process spawns removed from every nav start | S | **UNVERIFIED** |
| jetank_web_control-15 | runtime | cmd_vel_bridge allocates a new TwistStamped and Time object on every 20 Hz tick | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/cmd_vel_bridge.py:184` | low | static | Fewer per-tick allocations; negligible CPU but trivially cheaper | S | **UNVERIFIED** |
| jetank_web_control-16 | runtime | nav_status performs blocking stat + waitpid syscalls on the asyncio loop each poll | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1003` | low | static | Removes 2 syscalls per poll per client; keeps the loop free of blocking I/O | S | **UNVERIFIED** |
| jetank_web_control-17 | minimality | navigate_to_pixel duplicates _pixel_to_world_locked line-for-line | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1132` | low | static | -7 LOC, one code path for pixel->world | S | **UNVERIFIED** |
| jetank_web_control-18 | minimality | Dead pre-checks in _safe_capture_name are fully covered by the regex; one branch is unreachable | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:106` | low | static | -4 LOC | S | **UNVERIFIED** |
| jetank_web_control-19 | minimality | get_frame duplicates get_frame_and_seq | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:615` | low | static | -4 LOC | S | **UNVERIFIED** |
| jetank_web_control-20 | minimality | Grasp and mission action callbacks repeat identical 4-line state-reset blocks seven times | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:902` | low | static | ~-20 LOC; one place to keep the terminal-state invariant | S | **UNVERIFIED** |
| jetank_web_control-21 | minimality | handle_get_labels reaches into private node state and takes _classes_lock a second time | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1608` | low | static | -3 LOC, one lock acquisition fewer per request | S | **UNVERIFIED** |
| jetank_web_control-22 | minimality | rough_boxes_from_bgr stores a temporary '_frac' key only to sort then delete it | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:286` | low | static | -4 LOC | S | **UNVERIFIED** |
| jetank_web_control-23 | minimality | Sidebar 'Mapping Mode' toggle duplicates the map-footer Start/Stop buttons and keeps a second state flag | `/home/koen/workspaces/ros2_ws/static/app.js:348` | low | static | ~-30 LOC across html/js/css; one source of truth for nav state | S | **UNVERIFIED** |
| jetank_web_control-24 | minimality | Broken \u escapes in mode labels render garbage characters | `/home/koen/workspaces/ros2_ws/static/app.js:18` | low | static | Correct label; -2 stray characters | S | **UNVERIFIED** |
| jetank_web_control-25 | minimality | Client re-implements is_terminal_mission_status instead of receiving the verdict from the server | `/home/koen/workspaces/ros2_ws/static/app.js:735` | low | static | -6 LOC JS, no duplicated rule | S | **UNVERIFIED** |
| jetank_web_control-26 | minimality | Module docstring endpoint table is a stale second copy of the router table | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:9` | low | static | -25 LOC of drifting documentation | S | **UNVERIFIED** |
| jetank_web_control-27 | minimality | Redundant availability flags for the optional action imports | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:66` | low | static | -6 LOC and simpler guards (client is None) | S | **UNVERIFIED** |
| jetank_web_control-28 | minimality | cmd_vel_bridge _nav_subscribed flag is redundant with _nav being None | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/cmd_vel_bridge.py:157` | low | static | -3 LOC, one fewer invariant | S | **UNVERIFIED** |
| jetank_web_control-29 | minimality | 80 lines of ROS/aiohttp stub scaffolding in conftest.py for a bare-interpreter test run that this pixi workspace never uses | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/test/conftest.py:67` | low | static | ~-100 LOC of test scaffolding | S | **UNVERIFIED** |
| jetank_web_control-30 | footprint | Hard runtime dependencies aiohttp, numpy, Pillow and ament_index_python are not declared in package.xml | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:12` | low | static | Reproducible install via rosdep/pixi; no manual pip step | S | **UNVERIFIED** |
| jetank_web_control-31 | footprint | Unused ament linter test dependencies (copyright/flake8/pep257) pull three packages for tests that do not exist | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:24` | low | static | 3 fewer packages in rosdep/pixi resolution for this package | S | **UNVERIFIED** |
| jetank_web_control-33 | footprint | Message packages declared with <depend> in a pure ament_python package | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:13` | low | static | Tighter manifest; no build-time edges for a Python package | S | **UNVERIFIED** |

### jetank_web_control-01 — Camera subscription is always active: node deserializes ~30 fps of JPEG frames with zero viewers

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:493` · package jetank_web_control · lens runtime · severity high · evidence measured · effort M · verdict confirmed

Description: The CompressedImage subscription is created unconditionally in __init__, so the rclpy spin thread takes and deserializes every camera frame (~144 KB, ~30 fps as proven by the MJPEG capture) whether or not any browser is streaming. With 0 HTTP clients connected (ss showed only the LISTEN socket) the live node's spin thread (TID 8397) burns a steady ~6.2% of an Orin Nano core and 84 wakeups/s, 24/7, purely to discard frames.

Evidence detail: profile_node(/web_control_node,10s): cpu mean 7.5%, p95 9.9%, peak 19.6%, RSS 96 MB, 12 threads, 0 clients. Per-thread /proc stat: spin thread 8397 = 31 ticks/5s (6.2%), 84 ctx-switches/s; asyncio main thread 8258 = 0 ticks. Isolated scratch node reproducing only the CompressedImage subscription: 4.87-5.12% CPU, 69-78 wakeups/s; same node with timer+subs+actions but no image: 1.5%. 10 s MJPEG capture: 294 distinct parts of ~144 KB (=29.4 fps), so the topic is ~30 Hz/4.2 MB/s (the MCP hz/bw tools undercount it at 3.85 Hz).

Estimated gain: ~5-6% of a core saved whenever no browser is streaming (the common case on the robot); fewer DDS reader buffers held

Fix sketch: Keep a stream-client refcount in handle_mjpeg (increment after prepare, decrement in finally). On 0->1 call node.create_subscription(...) (rclpy allows creation from the aiohttp thread; the executor guard picks it up), on 1->0 node.destroy_subscription(). Same gate for the sim raw-Image path. Also gate /capture on the same mechanism (create a one-shot subscription when no stream is open).

Verifier (confirmed, adjusted high): web_control_node.py:492-495 creates the CompressedImage/Image subscription unconditionally in __init__, and grep finds no refcount/viewer gating or destroy_subscription anywhere in the file; _on_image (line 539) copies every frame into _latest_jpeg regardless of consumers, and handle_mjpeg (line 1384) only reads the cache. Live re-measurement this session: profile_node(/web_control_node, 5 s) cpu mean 7.46%, p95 9.9%, RSS 91 MB, with zero established HTTP connections (ss showed none), and per-thread /proc stat showed the spin thread TID 8397 consuming 35 ticks/5 s (~7% of a core) while all other threads totaled 6 ticks. Severity high stands: this is continuous idle CPU burn on the robot; not a finding-refuting factor that the launch defaults (image_compressed = not sim) always enable it.

### jetank_web_control-03 — Map PNG encoded with optimize=True in the ROS spin thread stalls teleop for ~200 ms per map update

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:587` · package jetank_web_control · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: _on_map runs in the rclpy executor thread and calls PIL img.save(format='PNG', optimize=True) on an RGB image. optimize=True tries multiple zlib strategies and is 7x slower than default for a ~5% size gain; RGB is 3x more data than a grayscale 'L' image for a 3-value map. During mapping slam_toolbox republishes /map every map_update_interval (5.0 s in jetank_navigation/config/slam/slam_toolbox.yaml), so every 5 s the watchdog timer, /cmd_vel publish path and image callbacks are blocked for the encode duration while the user is actively driving to build the map.

Evidence detail: Offline PIL benchmark in the pixi env on a 400x400 synthetic occupancy grid: RGB optimize=True 221.5 ms (57.9 KB); RGB optimize=False 29.6 ms (60.7 KB); mode 'L' optimize=True 132.5 ms (46.4 KB); mode 'L' optimize=False 15.5 ms (47.5 KB). Live measurement not possible (no /map publisher running now).

Estimated gain: 14x faster map render (221 ms -> ~15 ms); removes 0.2-1 s executor stalls every 5 s during mapping; smaller PNG too

Fix sketch: Build a uint8 grayscale array (128 unknown, 220 free, 20 occupied) and save with Image.fromarray(g,'L').save(buf,'PNG',optimize=False,compress_level=1). Optionally move the encode to the GET handler (asyncio.to_thread) keyed on a map seq so it runs off the ROS thread and only when a client asks.

Verifier (confirmed, adjusted medium): Re-measured on this Jetson in the pixi env (400x400 synthetic grid, median of 5): RGB optimize=True 158 ms / 63 KB vs 'L' optimize=False 14.4 ms / 52 KB (~11x) and 'L' compress_level=1 4.1 ms / 91 KB (~38x), so the claimed ~14x gain is plausible. The threading claim holds: web_control_node.py:1746 runs a single rclpy.spin thread, and the /map subscription (line 496), watchdog timer at 0.1 s (line 502) and image callbacks (lines 493-495) all share that executor, so _on_map's encode at line 587 does stall them. Severity medium is honest: it only fires every ~5 s during mapping (slam_toolbox.yaml:36) and /cmd_vel publishes from the web handlers are not executor-gated, but the "0.2-1 s" upper bound overstates a 400x400 map (~160 ms here).

### jetank_web_control-05 — Browser send loop streams zero-velocity commands at 10 Hz forever, defeating the node's silent-when-idle watchdog

`/home/koen/workspaces/ros2_ws/static/app.js:211` · package jetank_web_control · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: tick() runs every 100 ms whenever the WebSocket is open and always ws.send()s, including linear_x=0/angular_z=0. Server-side apply_cmd publishes a Twist for every message, so any open browser tab produces a permanent 10 Hz zero Twist stream on /cmd_vel plus 10 JSON parses/s. The node's watchdog (web_control_node.py:408-414) was deliberately made silent after a 0.3 s burst so base_approach can own the topic during a grasp APPROACH; on hardware the web node publishes Twist directly on /cmd_vel (live node info: publisher /cmd_vel; mobile_grasp_hw.launch.py bridges /cmd_vel_manip -> /cmd_vel), so the browser's zeros re-introduce exactly the interleaving that comment describes.

Evidence detail: Code path: app.js:203-215 tick -> ws.send unconditionally; web_control_node.py:1426-1433 handle_websocket -> apply_cmd -> _publish_twist every message. Live /cmd_vel hz was 0.0 only because no browser was connected during the measurement window.

Estimated gain: Removes 10 msg/s WS + 10 publishes/s per open tab while idle; restores the intended idle-silence on /cmd_vel

Fix sketch: In tick(): if l==0 && a==0 and the previous send was also zero, skip sending (the server watchdog already brakes after cmd_timeout_sec); optionally send a 1 Hz keepalive with a 'keepalive' flag the server does not publish. Server-side: in apply_cmd, if both are 0 and _stop_burst==0, skip _publish_twist.

Verifier (confirmed, adjusted medium): Mechanism verified: static/app.js:203-213 ws.send()s every 100 ms unconditionally, and web_control_node.py:1426-1433 -> apply_cmd (1357-1362) -> _publish_twist on every message, so an idle tab defeats the deliberate idle-silence at 408-414/601-611. The client-side fix is safe and in-scope: no consumer outside jetank_web_control reads the WS format or calls apply_cmd (grep), aiohttp's heartbeat=5.0 (1421) keeps the socket alive without app messages, the watchdog still brakes after cmd_timeout_sec 0.5 s, and cmd_vel_bridge.py:12-13/155 already ignores zero teleop, so nothing depends on the zero stream. The optional server-side variant needs care: test_cmd_vel_bridge.py:452-461 _FakeNode has no _stop_burst attribute, so an unguarded `self._stop_burst` check in apply_cmd would break TestApplyCmdMath unless ordered after the both-zero test, and it would also turn the disconnect-safety apply_cmd(0,0) at 1441 into a no-op once the burst is spent (harmless since the base is already stopped). No package merging or perception abstraction is touched.

### jetank_web_control-06 — MJPEG handler polls at a fixed 30 Hz per client instead of waking on new frames

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1412` · package jetank_web_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: handle_mjpeg loops with asyncio.sleep(0.033) and re-reads get_frame_and_seq() under the lock 30 times/s per client, regardless of camera rate. With a slow camera (sim, or a throttled stream) most wakeups are empty; with N clients the loop wakes 30N times/s. Each frame is also written as three chunked-transfer writes (header, body, CRLF) = 3 syscalls/frame.

Evidence detail: Live: one curl MJPEG client for 10 s cost the asyncio main thread 22 ticks (2.2% of a core) at 29.4 fps / 4.2 MB/s over loopback (idle main thread = 0 ticks). The polling share cannot be separated from write cost without modifying the node; the 30 Hz timer itself is ~300 wakeups/s per client.

Estimated gain: Removes up to 30 empty wakeups/s per client when the camera is slower than 30 fps; 1 fewer syscall per frame

Fix sketch: Keep an asyncio.Event on the app; in _on_image call loop.call_soon_threadsafe(event.set). In handle_mjpeg: await event.wait(); event.clear(); write. Prepend the trailing CRLF to the next part's header so each frame is 2 writes.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-07 — 10 Hz watchdog timer runs permanently even when idle and silent

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:502` · package jetank_web_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: create_timer(0.1, _watchdog_cb) fires 10 times/s for the node's whole lifetime; once _stop_burst reaches 0 the callback is a pure early-return, but every fire still costs a full rclpy wait-set rebuild + Python dispatch in the executor.

Evidence detail: Isolated scratch node with only a 10 Hz no-op timer: 0.87% CPU, 11.2 wakeups/s on the spin thread (vs 0.00% / 0 idle).

Estimated gain: ~0.9% of a core whenever no client is driving

Fix sketch: Cancel the timer (self._wd.cancel()) when _stop_burst hits 0; in apply_cmd call self._wd.reset() if cancelled. rclpy Timer supports cancel()/reset().

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-09 — Desktop page polls four endpoints every second forever, even with no nav stack and no map

`/home/koen/workspaces/ros2_ws/static/app.js:372` · package jetank_web_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: startMapRefresh runs at page load on every desktop client and every 1 s issues GET /map_meta, GET /map.png (404 until a map exists), GET /nav_status and GET /robot_pose, indefinitely. Each aiohttp request costs ~2 ms of Python on the Orin; with the map panel idle this is pure background load and WiFi chatter, per open tab.

Evidence detail: Live: replaying the exact 4-GET set at 1 Hz for 10 s cost the main thread 8 ticks = 0.8% of a core (40 requests -> ~2 ms each; /map.png returned 404 every time). /captures + /detections/latest x20 each = 8 ticks, same ~2 ms/request.

Estimated gain: ~0.8% of a core per idle desktop tab; 4 fewer HTTP round-trips/s

Fix sketch: Merge into one GET /status returning nav_status + map_meta(+seq) + robot_pose; poll at 1 Hz only while nav_status.running is set, otherwise every 5 s; stop polling when document.hidden.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-11 — Open MJPEG stream keeps running while the tab is hidden or the Annotate panel is open

`/home/koen/workspaces/ros2_ws/static/index.html:26` · package jetank_web_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: The <img src="/stream.mjpg"> connection starts at page load and is never paused. A backgrounded tab or a user annotating in the label panel still pulls ~4.2 MB/s (30 fps x 144 KB) over WiFi and costs the server ~2.2% of a core per connection.

Evidence detail: Live: 10 s curl of /stream.mjpg = 42.6 MB, 294 parts (29.4 fps, ~144 KB each); main-thread cost 22 ticks/10 s (2.2%) per client.

Estimated gain: 2.2% of a core + 4.2 MB/s per hidden/idle tab

Fix sketch: On document visibilitychange (hidden) and when the label panel opens, set camImg.src=''; restore '/stream.mjpg?'+Date.now() when visible/closed. Combine with the refcounted lazy subscription (finding 01) so the ROS side also goes idle.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-34 — numpy and PIL are imported eagerly at module import though only the map and sim-image paths use them

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:71` · package jetank_web_control · lens footprint · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: On the hardware path (image_compressed=true) numpy/PIL are needed only when a /map message arrives; importing them at startup adds ~300 ms to node start and loads their shared objects into the process for the whole lifetime. cv2 is already imported lazily (255, 738) for the same reason.

Evidence detail: python -X importtime in the pixi env: numpy 207 ms, PIL.Image 95 ms, cv2 610 ms, aiohttp 417 ms. Live process maps show numpy (8 mappings) and PIL (4) loaded with no /map publisher present. RSS delta from the imports was ~2-3 MB in a maxrss test.

Estimated gain: ~300 ms faster startup on the robot; ~2-3 MB RSS

Fix sketch: Move 'import numpy as np' / 'from PIL import Image' into _on_map and _on_raw_image (module-level cache after first import) mirroring the existing lazy cv2 pattern.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-02 — _on_image fully deserializes each CompressedImage then copies it; a raw=True subscription with a CDR slice avoids both

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:539` · package jetank_web_control · lens runtime · severity low (orig medium) · evidence measured · effort M · verdict confirmed

Description: Each frame is converted by rclpy into a Python CompressedImage (array.array for data) and then copied again with bytes(msg.data). The node only needs the JPEG bytes. Subscribing with raw=True yields the serialized CDR buffer; the JPEG payload is a fixed-format slice (4-byte encapsulation, 8-byte stamp, frame_id string, format string, uint32 length, data), so no per-field conversion is needed.

Evidence detail: Isolated scratch node on the live topic (8 s windows, same conditions): default subscription 4.87% CPU / 78.5 wakeups/s vs raw=True 3.87% / 56.9 wakeups/s (-20% CPU). Best-effort QoS gave no gain (5.25% / 3.62% raw). CDR layout verified from serialize_message hex dump: frame_id len at byte 12, data length field at 40 for an 11-char frame_id, data starts at 44 (4-byte aligned strings).

Estimated gain: ~1% of a core at 30 fps while streaming; removes one 144 KB copy per frame

Fix sketch: create_subscription(CompressedImage, topic, self._on_image_raw, qos, raw=True); in the callback parse: off=12; n=u32(off); off=align4(off+4+n); n=u32(off); off=align4(off+4+n); n=u32(off); jpeg=memoryview(buf)[off+4:off+4+n]; store bytes(jpeg) (single copy) or the memoryview itself for aiohttp write.

Verifier (confirmed, adjusted low): The fix is in-scope and safe: `_on_image` (web_control_node.py:539-542) is the only writer of `_latest_jpeg`, and every consumer (`get_frame` :616, `get_frame_and_seq` :621, `save_capture` :624, and the `np.frombuffer` decode at ~:740) only needs a bytes-like JPEG payload, so storing `bytes(memoryview_slice)` is behaviour-preserving; a workspace grep found no other package touching `_on_image`/`_latest_jpeg` (only launch/setup/topics.yaml reference the node by name), and `rclpy.node.Node.create_subscription` in the pixi env accepts `raw` (verified via inspect). The test suite stubs `sensor_msgs.msg.CompressedImage` (test/conftest.py:124) and never calls `create_subscription`, so tests are unaffected. The only residual risk is the hand-rolled CDR offset parsing (should check the 2-byte encapsulation header for endianness/XCDR2 rather than assuming CDR_LE), which is a robustness concern of the fix, not a scope or consumer break. Severity lowered because the absolute gain is ~1% of a core for M effort plus added parsing fragility.

### jetank_web_control-04 — Sim raw-Image path JPEG-encodes every frame with PIL in the spin thread, with an extra channel-flip copy for bgr8

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:570` · package jetank_web_control · lens runtime · severity low (orig medium) · evidence measured · effort M · verdict confirmed

Description: _on_raw_image encodes each incoming sensor_msgs/Image to JPEG regardless of whether any viewer exists, on the executor thread. For bgr8 it does arr[:,:,::-1] which PIL.fromarray must materialise as a full copy. At a 30 fps Gazebo camera this is 50-65% of one core continuously, and it delays every other callback (watchdog, /map, /amcl_pose).

Evidence detail: Offline benchmark (pixi env, 1280x720 random RGB): PIL JPEG q80 16.0 ms; PIL from bgr[:,:,::-1] 21.5 ms; cv2.imencode q80 13.9 ms (accepts BGR natively). At 30 fps: 480-645 ms CPU per second. Not measured live (hardware path is active now, image_compressed=true).

Estimated gain: ~50% of a core in sim while nobody is watching; 25-35% less per frame when watching (cv2, no flip)

Fix sketch: Gate the subscription on stream clients (finding 01); when active, throttle to the stream cap (skip frames arriving faster than the last send) and encode with cv2.imencode('.jpg', arr, [IMWRITE_JPEG_QUALITY,80]) when cv2 is importable (it already is an optional dependency for autolabel), falling back to PIL.

Verifier (confirmed, adjusted low): The mechanism is real: web_control_node.py:495 subscribes unconditionally and _on_raw_image (lines 544-573) JPEG-encodes every frame via PIL on the single rclpy.spin thread (line 1746) with no viewer/stream-client gating; the bgr8 branch at line 561 does a negative-stride slice that PIL must copy. The magnitude is overstated though: the sim camera in jetank_description/urdf/components/camera.xacro:89-95 is 640x360 RGB_INT8 at 30 Hz, not 1280x720, and Gazebo emits rgb8 so the bgr flip path is not hit in sim. Measured in the pixi env this session: 640x360 PIL rgb 3.8 ms/frame (~11% of a core at 30 fps, vs the claimed 50-65%); 1280x720 PIL rgb 14.6 ms, bgr-flip 21.5 ms, cv2 12.7 ms confirms the relative gains for the fix. Wasted work while nobody watches is confirmed, but at low severity for the actual sim config.

### jetank_web_control-08 — Two always-on ActionClients add 10 wait-set entities that are polled on every executor iteration

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:425` · package jetank_web_control · lens runtime · severity low · evidence measured · effort M · verdict **UNVERIFIED**

Description: NavigateToPose (line 425) and GraspObject (line 482) action clients are created at startup; each adds 3 service clients + 2 subscriptions (feedback/status) to the wait set and a status subscription that discovers/matches with every nav/grasp server. They are used only after a POST; until then they only make each spin iteration more expensive.

Evidence detail: Isolated scratch node: timer+4 subs = 1.25% CPU / 17.9 wakeups/s; timer+4 subs+2 action clients = 1.50% / 22.2 wakeups/s (+0.25%, +4 wakeups/s) with no action server running.

Estimated gain: ~0.25% of a core idle; smaller wait set

Fix sketch: Create the ActionClient lazily on first POST (/navigate, /grab) inside asyncio.to_thread with wait_for_server(timeout_sec=2.0), keep it afterwards. Alternatively keep as-is if the non-blocking server_is_ready() discovery is valued more than the 0.25%.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-10 — Map PNG re-downloaded every second with a cache-busting query even when the map has not changed

`/home/koen/workspaces/ros2_ws/static/app.js:502` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: refreshMap sets map-img.src='/map.png?t='+Date.now() every tick, forcing a full PNG transfer (tens of KB) each second while slam_toolbox only updates /map every 5 s (map_update_interval) and AMCL-mode maps never change. The server has no change token, so the browser cannot avoid it.

Evidence detail: app.js:489-505 reloads unconditionally; web_control_node.py:593-599 _map_meta carries no sequence/ETag; _on_map is the only writer so a counter is trivial.

Estimated gain: ~80% fewer map transfers during mapping, 100% fewer in AMCL mode

Fix sketch: Increment self._map_seq in _on_map, expose it in /map_meta, and only reassign img.src when meta.seq changed; or set an ETag header on /map.png and honour If-None-Match with 304.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-12 — GET /captures opens and parses every label sidecar in the dataset on each request

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:675` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: list_captures does os.listdir + sorted, then for every .jpg opens the .txt sidecar, reads it and runs _yolo_parse just to derive labelled/n_boxes. The README states captures accumulate without bound, so this is O(N) file opens + parses per labeller open, executed synchronously on the asyncio loop (handle_list_captures is not offloaded).

Evidence detail: Lines 662-689 run on the event loop (handle_list_captures at 1587-1589 calls it directly). Current dataset is tiny (1 file, 8 KB) so the cost is not measurable today; it scales linearly with dataset size.

Estimated gain: Avoids N file reads + parses per open; keeps the loop responsive with large datasets

Fix sketch: Use os.scandir once, build a set of .txt names; labelled = sidecar exists and stat().st_size>0; n_boxes = count of non-empty lines (or drop n_boxes from the list and fetch it per image). Run via asyncio.to_thread.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-13 — Labeller redraw performs 4 layout reads per box per mouse-move

`/home/koen/workspaces/ros2_ws/static/app.js:938` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: lblRedraw calls lblNormToCanvas twice per box; each call runs containRect (getBoundingClientRect on the img) plus a second getBoundingClientRect on the canvas. During a drag this runs on every mousemove, so N boxes cost 4N forced layout reads per event.

Evidence detail: app.js:906-914 lblNormToCanvas recomputes containRect+cv.getBoundingClientRect each call; app.js:937-948 calls it 2x per box; app.js:977-990 mousemove -> lblRedraw.

Estimated gain: Constant 2 layout reads per redraw instead of 4N; smoother dragging with many boxes

Fix sketch: Compute rect=containRect(img) and cr=cv.getBoundingClientRect() once at the top of lblRedraw and pass an inline mapping closure to the box loop; same for lblCanvasToNorm in the mouse handlers.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-14 — _launch_nav forks 11 sequential pkill processes and then sleeps a fixed 1 s before every nav start

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1024` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Each Start Mapping / Navigate call first runs stop_nav (waits up to 8 s), then spawns pkill -9 -f once per pattern (11 fork+exec of a shell utility), then unconditionally time.sleep(1.0). The loose patterns (e.g. '-f amcl', '-f map_server') also match any unrelated process whose command line contains the substring. All of this is latency added to a user-facing button.

Evidence detail: Lines 1021-1025; patterns at 1014-1018. nav lock (app['nav_lock']) is held for the whole sequence so concurrent nav requests queue behind the sleep.

Estimated gain: ~1 s + 10 process spawns removed from every nav start

Fix sketch: Single subprocess.run(['pkill','-9','-f', '|'.join(patterns)]) and then poll pgrep in a short loop (max ~1 s) exiting as soon as nothing matches; anchor patterns to executable names (e.g. '(^|/)amcl( |$)').

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-15 — cmd_vel_bridge allocates a new TwistStamped and Time object on every 20 Hz tick

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/cmd_vel_bridge.py:184` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _tick constructs TwistStamped() plus get_clock().now().to_msg() each tick, and _now() creates an rclpy Time object per callback and per tick (lines 127-128). At rate_hz=20 this is ~40-60 short-lived message/Time objects per second in the sim hot path.

Evidence detail: Lines 169-190 and 127-141. Sim-only node (not running live now), so unmeasured.

Estimated gain: Fewer per-tick allocations; negligible CPU but trivially cheaper

Fix sketch: Preallocate self._out = TwistStamped() with frame_id set once; per tick assign header.stamp and twist. Use time.monotonic() for staleness bookkeeping (timeouts are relative).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-16 — nav_status performs blocking stat + waitpid syscalls on the asyncio loop each poll

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1003` · package jetank_web_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: nav_status() (called from handle_nav_status directly, not via to_thread) calls proc.poll() and has_saved_map() -> os.path.isfile on every 1 Hz poll from every desktop tab. Cheap today, but it is blocking filesystem/process I/O inside an async handler.

Evidence detail: Lines 1000-1009 and handler at 1507-1509; app.js:373 polls it at 1 Hz.

Estimated gain: Removes 2 syscalls per poll per client; keeps the loop free of blocking I/O

Fix sketch: Cache has_saved_map result and refresh it only in save_map()/start_navigation(); read the last known proc state updated by the nav lifecycle methods instead of polling.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-17 — navigate_to_pixel duplicates _pixel_to_world_locked line-for-line

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1132` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 1132-1138 (lock, copy meta/origin, 'no map yet', map_pixel_to_world) are the same as the helper defined at 1175-1181, which the two mission methods already use.

Evidence detail: Compared lines 1132-1138 with 1175-1181 in this session.

Estimated gain: -7 LOC, one code path for pixel->world

Fix sketch: world = self._pixel_to_world_locked(ix, iy); if world is None: return False, 'no map yet'; wx, wy = world.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-18 — Dead pre-checks in _safe_capture_name are fully covered by the regex; one branch is unreachable

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:106` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _SAFE_NAME_RE (^[A-Za-z0-9._-]+\.[Jj][Pp][Gg]$) already rejects empty strings, '/', '\\' and the bare name '..'. The check '..' in name.split('.') (line 108) can never be true because splitting on '.' cannot produce a '..' element.

Evidence detail: Regex at line 97; checks at 106-109; test_labels.py cases (traversal, backslash, empty, '..', 'foo..bar.jpg') all pass through the regex alone.

Estimated gain: -4 LOC

Fix sketch: return name if name and _SAFE_NAME_RE.match(name) else None

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-19 — get_frame duplicates get_frame_and_seq

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:615` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Two accessors take the same lock and return the same frame; get_frame is used only by save_capture (line 631).

Evidence detail: grep in this session: get_frame( appears at 615 (def) and 631 (use); get_frame_and_seq at 619 and 1400.

Estimated gain: -4 LOC

Fix sketch: frame, _ = self.get_frame_and_seq() in save_capture; delete get_frame.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-20 — Grasp and mission action callbacks repeat identical 4-line state-reset blocks seven times

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:902` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The block 'running=False; stage=''; last_success=X; last_message=Y' under _grasp_lock appears at 902-906, 910-914, 933-937 and 939-943; the mission equivalent (active=False; goal_handle=None; status=...) at 1246-1249, 1253-1256, 1269-1272.

Evidence detail: Read lines 897-967 and 1241-1287 in this session.

Estimated gain: ~-20 LOC; one place to keep the terminal-state invariant

Fix sketch: def _grasp_finish(self, success, message, stage=''): with self._grasp_lock: ... ; def _mission_finish(self, status): with self._mission_lock: ... ; call from each branch.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-21 — handle_get_labels reaches into private node state and takes _classes_lock a second time

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1608` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: read_labels already acquires _classes_lock (line 718) to get n_classes; the handler then re-acquires it to copy the class list. Returning classes from read_labels keeps the lock discipline inside the node and removes the private access.

Evidence detail: Lines 703-720 and 1602-1610.

Estimated gain: -3 LOC, one lock acquisition fewer per request

Fix sketch: Have read_labels return (boxes, classes) copying both under one lock; handler unpacks.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-22 — rough_boxes_from_bgr stores a temporary '_frac' key only to sort then delete it

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:286` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Boxes carry '_frac' solely as a sort key, followed by a loop that deletes it. Sorting on w*h (already normalised area = frac) removes the key and the cleanup loop.

Evidence detail: Lines 281-292.

Estimated gain: -4 LOC

Fix sketch: boxes.sort(key=lambda b: b['w']*b['h'], reverse=True); return boxes[:max_boxes]

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-23 — Sidebar 'Mapping Mode' toggle duplicates the map-footer Start/Stop buttons and keeps a second state flag

`/home/koen/workspaces/ros2_ws/static/app.js:348` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: toggleMappingMode (app.js:343-361) plus the sidebar button (index.html:117-122) and .sidebar-footer CSS (style.css:321-325) only call startMapping()/stopNav(), which the map footer already exposes (index.html:64,67). Its local mappingMode flag can desync from navRunning (e.g. mapping stopped via the footer leaves the sidebar button saying 'Stop Mapping'). stopMapRefresh (app.js:484-487) is never called and drawRobotArrow (479-482) is a 2-line wrapper used once.

Evidence detail: grep in this session: toggleMappingMode referenced once (index.html), stopMapRefresh defined but 0 call sites, drawRobotArrow 1 call site.

Estimated gain: ~-30 LOC across html/js/css; one source of truth for nav state

Fix sketch: Delete toggleMappingMode/mappingMode, the sidebar Navigation section and .sidebar-footer rules, stopMapRefresh; inline lastPose=p; redrawOverlay() at the drawRobotArrow call site.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-24 — Broken \u escapes in mode labels render garbage characters

`/home/koen/workspaces/ros2_ws/static/app.js:18` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: '὏1 Touch mode' and '὚5 Desktop mode' use 4-hex-digit \u escapes, so they decode as U+1F4F followed by a literal '1' / U+1F5A followed by '5'. The glyphs add nothing; the text alone is enough.

Evidence detail: app.js lines 18 and 20; JavaScript \uXXXX takes exactly four hex digits (astral code points need \u{...}).

Estimated gain: Correct label; -2 stray characters

Fix sketch: Use 'Touch mode' / 'Desktop mode', or '\u{1F4F1}' / '\u{1F5A5}'.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-25 — Client re-implements is_terminal_mission_status instead of receiving the verdict from the server

`/home/koen/workspaces/ros2_ws/static/app.js:735` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: isTerminalStatus duplicates the Python helper (web_control_node.py:221-235) and must be 'kept in sync' by comment. /mission/status already has the data to return a boolean.

Evidence detail: app.js:734-739 vs web_control_node.py:218-235; mission_status() at 1288-1292 returns only status/active.

Estimated gain: -6 LOC JS, no duplicated rule

Fix sketch: mission_status() returns {'status','active','terminal': is_terminal_mission_status(status)}; missionPoll uses s.terminal && !s.active.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-26 — Module docstring endpoint table is a stale second copy of the router table

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:9` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 9-33 list endpoints by hand; build_app (1690-1720) is the real table and already differs (/detections/latest, /robot_pose, /navigate, /capture are missing from the docstring).

Evidence detail: Compared docstring 9-33 with router registrations 1690-1720.

Estimated gain: -25 LOC of drifting documentation

Fix sketch: Replace the list with one line pointing at build_app(); README already documents the HTTP API.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-27 — Redundant availability flags for the optional action imports

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:66` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _MISSION_AVAILABLE/_GRASP_AVAILABLE (66-83) duplicate '_RunMissionAction is not None' / '_GraspObjectAction is not None', and are copied again into self._mission_available / self._grasp_available (430, 474) alongside self._mission_client / self._grasp_client which are None in the same cases; guards then test both (874, 1210).

Evidence detail: Lines 64-83, 430, 474, 874, 1210.

Estimated gain: -6 LOC and simpler guards (client is None)

Fix sketch: Drop the module flags and instance copies; guard on self._mission_client is None / self._grasp_client is None; grasp_status 'available' = self._grasp_client is not None.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-28 — cmd_vel_bridge _nav_subscribed flag is redundant with _nav being None

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/cmd_vel_bridge.py:157` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: When nav_topic is '' no subscription is created, so self._nav stays None and the 'self._nav is not None' check already excludes nav; the extra _nav_subscribed flag (89, 113, 157) only adds state (one unit test asserts on it).

Evidence detail: Lines 89, 111-113, 157-159; test_cmd_vel_bridge.py TestEmptyTopicGuard depends on the flag.

Estimated gain: -3 LOC, one fewer invariant

Fix sketch: Remove _nav_subscribed; adjust the two tests to pass nav=None instead.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-29 — 80 lines of ROS/aiohttp stub scaffolding in conftest.py for a bare-interpreter test run that this pixi workspace never uses

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/test/conftest.py:67` · package jetank_web_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _install_stubs fabricates rclpy, geometry_msgs, nav_msgs, sensor_msgs, nav2_msgs, vision_msgs, std_msgs, jetank_* and aiohttp modules when absent. The README's documented test command runs inside pixi where all of these are real packages, so the stub path is dead in practice and must be maintained (e.g. the qos stub list) every time an import is added.

Evidence detail: conftest.py:21-149; README.md:449-460 runs tests via 'pixi run -- ... pytest'.

Estimated gain: ~-100 LOC of test scaffolding

Fix sketch: Keep only the sys.path insert; let pytest.importorskip('rclpy') skip the suite outside the ROS env.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-30 — Hard runtime dependencies aiohttp, numpy, Pillow and ament_index_python are not declared in package.xml

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:12` · package jetank_web_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: web_control_node.py raises SystemExit without aiohttp (85-91), uses numpy/PIL for the map and sim image paths (71-76) and ament_index_python (312), and cv2 optionally (255, 738); none appear in package.xml, so rosdep cannot resolve them and the README falls back to 'pip3 install aiohttp'.

Evidence detail: package.xml lines 12-18 list only rclpy + message packages; grep for aiohttp/numpy/pillow/opencv/ament_index in package.xml and setup.py returned nothing.

Estimated gain: Reproducible install via rosdep/pixi; no manual pip step

Fix sketch: Add <exec_depend>python3-aiohttp</exec_depend>, python3-numpy, python3-pil, ament_index_python; python3-opencv as an optional (documented) extra.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-31 — Unused ament linter test dependencies (copyright/flake8/pep257) pull three packages for tests that do not exist

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:24` · package jetank_web_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: test_depend on ament_copyright, ament_flake8 and ament_pep257 is only meaningful with test_copyright.py / test_flake8.py / test_pep257.py; test/ contains only conftest.py, test_cmd_vel_bridge.py, test_labels.py, test_mission.py.

Evidence detail: ls test/ in this session; package.xml lines 24-26.

Estimated gain: 3 fewer packages in rosdep/pixi resolution for this package

Fix sketch: Delete the three test_depend lines (keep python3-pytest).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_web_control-33 — Message packages declared with <depend> in a pure ament_python package

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:13` · package jetank_web_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: <depend> expands to build, build_export and exec dependencies. A setup.py package has no build step that needs sensor_msgs/geometry_msgs/nav_msgs/nav2_msgs/vision_msgs/std_msgs; <exec_depend> is the accurate (and lighter for dependency-graph tooling) declaration.

Evidence detail: package.xml lines 12-18; build_type ament_python at line 30.

Estimated gain: Tighter manifest; no build-time edges for a Python package

Fix sketch: Change <depend> to <exec_depend> for rclpy and the six message packages.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_ros_main

Coverage: 58 files read; 16 measurements run. Notes: All source, config, launch, test, RViz, build and doc files in the package were read in full except workspace_template/pixi.lock (33,711 lines, generated lock file: only the header was read; its contents were checked by diff against the root lock, not line by line) and .pytest_cache internals (generated, gitignored; only nodeids was read). docs/images/jetank_real.jpg was inspected with `file` only. The package has no C++/CMakeLists.txt/xacro/JS files; the '~1878 lines of C++/Python' in the task corresponds to the Python launch files, topics.py and the two scripts. Live measurement caveats: in the running stack ros2_control_node, rplidar_node and the pixi-launched stereo_camera_node had all crashed at start (see measurements), so /joint_states came only from joint_state_publisher and no /scan existed; findings -02 and -06 describe the conflict that arises when those nodes are healthy as well as the observed degraded state. No node was launched, killed, parameterised or actuated; ps/log reads were read-only. Findings for sibling packages that surfaced incidentally (web_control_node idle CPU, ros2_control serial crash, jetank_detection lifecycle launch style) are noted inside the relevant jetank_ros_main findings but not filed separately since they are out of this package's scope.

Measurements:
- mcp__ros2-mcp__get_node_list: 16 nodes incl. /joint_state_publisher, /world_to_base_footprint_tf, /robot_controller, /web_control_node, /move_group, 3 spawners; NO /controller_manager, NO rplidar node
- mcp__ros2-mcp__get_topic_list: no /scan topic; /tf, /tf_static, /joint_states, /odom present
- mcp__ros2-mcp__profile_node /joint_state_publisher 10 s: cpu mean 1.56% p95 9.41% peak 9.9%, RSS 68.2 MB, 11 threads, 16 fds
- mcp__ros2-mcp__profile_node /world_to_base_footprint_tf 10 s: cpu mean 0.08%, RSS 28.85 MB, 11 threads
- mcp__ros2-mcp__profile_node /robot_controller 10 s: cpu mean 1.28% p95 9.9%, RSS 29.6 MB
- mcp__ros2-mcp__profile_node /robot_state_publisher 5 s: cpu mean 0.39%, RSS 30.3 MB
- mcp__ros2-mcp__profile_node /web_control_node 5 s: cpu mean 7.71% p95 9.9%, RSS 95.96 MB, 12 threads (no browser connected)
- mcp__ros2-mcp__get_node_info /joint_state_publisher: publishes /joint_states; /robot_state_publisher: publishes /tf,/tf_static, subscribes /joint_states
- mcp__ros2-mcp__get_node_params /robot_controller: left_motor=0 right_motor=1, alpha 1.0/beta 0.0, track_width 0.11, max_linear 1.0, max_angular 2.0, odom_rate 30, publish_odom true, base_frame base_footprint; no *_channel params
- mcp__ros2-mcp__get_node_params /joint_state_publisher: rate=10, publish_default_positions=true
- mcp__ros2-mcp__get_topic_hz /joint_states = 10.0 Hz (50 msgs/5 s); /tf = 40.3 Hz; /odom = 30.3 Hz
- mcp__ros2-mcp__read_topic /tf_static: world->base_footprint identity from static publisher + URDF fixed frames; /tf: odom->base_footprint from robot_controller at ~30 Hz plus arm/wheel joints from RSP; /joint_states: 10 joints all 0.0, frame_id ''
- ps -eo (read-only): unified launch pid 8228; spawners 8286/8294/8296 RSS 64-66 MB alive 438 s; joint_state_publisher RSS 66.6 MB; separate system-ROS stereo_camera launch pid 7948 started by hand
- grep ~/.ros/log/2026-09-26-09-15-05-559512-ubuntu-8228/launch.log: rplidar_node died exit 255 at +0.4 s; ros2_control_node died exit -6 at +1.3 s; pixi stereo_camera_node died exit 1 at +2 s; spawners log 'Could not contact service /controller_manager/list_controllers' every 10 s
- ls /dev/ttyUSB*: none present (lidar unplugged); ls build/jetank_perception/build.ninja: absent (Makefile generator)
- diff root pixi.toml/pixi.lock/LICENSE vs workspace_template copies: identical

Findings: 33 (high 1, medium 5, low 27).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_ros_main-01 | runtime | Static world->base_footprint TF gives base_footprint two parents (odom + world) | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:203` | high | measured | Removes 1 process (~29 MB RSS), eliminates duplicate-parent TF churn in all tf2 consumers; fixes MoveIt+odom/Nav2 frame tree conflict | S | confirmed |
| jetank_ros_main-02 | runtime | joint_state_publisher always started; duplicates joint_state_broadcaster with all-zero joint states when MoveIt is enabled | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/urdf.launch.py:50` | medium (orig high) | measured | ~68 MB RSS and ~1.5-2% CPU saved; removes conflicting /joint_states publisher when ros2_control is up | S | confirmed |
| jetank_ros_main-04 | runtime | RPLidar include is unconditional; node dies at start when no /dev/ttyUSB0, no enable arg or respawn | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:279` | medium | measured | Avoids a failed process per bringup; enables lidar-less sessions; clearer failure signalling | S | confirmed |
| jetank_ros_main-06 | runtime | ros2_control_node crash leaves three spawner processes polling forever; no OnProcessExit handling in the MoveIt layer include | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:298` | medium | measured | ~195 MB RSS and 3 idle Python processes reclaimed after a controller_manager failure; faster failure visibility | M | confirmed |
| jetank_ros_main-10 | footprint | package.xml declares all seven sibling packages (including jetank_simulation) as build+exec <depend> for a package that compiles nothing | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:26` | medium | static | Hardware target can build/install without Gazebo stack; correct dependency graph | S | confirmed |
| jetank_ros_main-31 | footprint | Workspace pixi.toml installs ros-humble-desktop, the full Gazebo stack and unused packages on the aarch64 robot target | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:80` | medium | static | Multiple GB smaller env on the Jetson; faster pixi install; fewer packages to resolve | M | confirmed |
| jetank_ros_main-05 | runtime | Stereo camera include always launches the pixi stereo_camera_node, which cannot open CSI under pixi and dies; a second copy must be run by hand | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:256` | low (orig medium) | measured | One fewer crashed process per bringup; single source of truth for camera launch args (frame ids actually applied) | S | confirmed |
| jetank_ros_main-07 | runtime | Web control enabled by default on hardware bringup: ~8% CPU and 96 MB RSS with no browser connected | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:114` | low | measured | ~8% of one core and ~96 MB RSS when the browser UI is not needed | S | **UNVERIFIED** |
| jetank_ros_main-08 | runtime | topics.py re-opens and re-validates topics.yaml on every accessor call | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/topics.py:27` | low | static | 3-5x fewer YAML parses/ament lookups per launch; ~10 LOC simpler | S | **UNVERIFIED** |
| jetank_ros_main-09 | minimality | Unused helpers contract_path() and node_topic_params() in topics.py | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/topics.py:49` | low | static | -8 LOC, smaller public surface | S | **UNVERIFIED** |
| jetank_ros_main-11 | footprint | Unused exec_depends: launch_xml, launch_yaml, urdf, xacro | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:22` | low | static | -4 dependency lines; cleaner rosdep set | S | **UNVERIFIED** |
| jetank_ros_main-12 | footprint | Dependencies actually used are undeclared: tf2_ros, rviz2, python3-yaml, ament_index_python, jetank_manipulation, jetank_detection is exec but sensor/nav/geometry msgs for scripts missing | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:12` | low | static | Accurate dependency closure for install.sh/rosdep; rclpy can go if scripts are removed (see -18) | S | **UNVERIFIED** |
| jetank_ros_main-13 | footprint | ament_lint_auto / ament_lint_common test_depends have no effect in an ament_python package | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:43` | low | static | Fewer test deps resolved by rosdep/CI | S | **UNVERIFIED** |
| jetank_ros_main-14 | minimality | test_copyright.py is permanently skipped: dead test file plus ament_copyright dependency | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/test/test_copyright.py:20` | low | static | -25 LOC, -1 test dependency | S | **UNVERIFIED** |
| jetank_ros_main-15 | minimality | test_pep257.py is documented as always failing (D213 vs D212) — burns colcon test time with zero signal | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/test/test_pep257.py:22` | low | static | One fewer always-red test; faster colcon test | S | **UNVERIFIED** |
| jetank_ros_main-16 | runtime | Diagnostic scripts use rclpy Rate.sleep() in the same thread as spin_once (blocking/deadlock-prone loop) | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:45` | low | static | Deterministic 10 Hz command loop; removes hang risk | S | **UNVERIFIED** |
| jetank_ros_main-17 | minimality | Duplicated forward/backward test bodies and left/right FPS blocks in the diagnostic scripts | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:73` | low | static | -40 LOC | S | **UNVERIFIED** |
| jetank_ros_main-18 | minimality | test_drive / test_cameras console scripts are sim-only, hardcode topics that don't match the sim, and are referenced by nothing | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/setup.py:31` | low | static | -420 LOC, -2 entry points, allows dropping rclpy/geometry_msgs/nav_msgs/sensor_msgs deps | S | **UNVERIFIED** |
| jetank_ros_main-19 | minimality | stereo_camera.launch.py is a pass-through wrapper that also diverges from unified's camera args | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/stereo_camera.launch.py:11` | low | static | -20 LOC, one launch file fewer, consistent frame ids across entrypoints | S | **UNVERIFIED** |
| jetank_ros_main-20 | minimality | main.launch.py is a 45-line wrapper whose only effect is enable_web_control:=false | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/main.launch.py:28` | low | static | -45 LOC, -1 launch file | S | **UNVERIFIED** |
| jetank_ros_main-22 | minimality | Optional-layer package paths resolved eagerly at parse time (get_package_share_directory) — hard failure even when the layer is disabled | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:218` | low | static | Optional layers become truly optional; consistent style, ~5 LOC | S | **UNVERIFIED** |
| jetank_ros_main-26 | minimality | Redundant use_sim_time=False parameters on hardware nodes | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:77` | low | static | -6 LOC, clearer intent | S | **UNVERIFIED** |
| jetank_ros_main-27 | minimality | Model path defaults hardcoded to /home/koen in two launch files, expanduser('~') in the third | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:159` | low | static | Portable defaults, one definition | S | **UNVERIFIED** |
| jetank_ros_main-28 | minimality | gazebo_sim.launch.py declares five conditional includes of the same launch file (one per world name) | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/gazebo_sim.launch.py:57` | low | static | -12 LOC, 4 fewer launch actions, unknown world name rejected at parse time via choices | S | **UNVERIFIED** |
| jetank_ros_main-29 | minimality | use_sim_time default declared twice in urdf.launch.py | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/urdf.launch.py:26` | low | static | -1 redundant default | S | **UNVERIFIED** |
| jetank_ros_main-30 | minimality | use_sim_time not forwarded to the motor and lidar includes while it is to every other include | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:247` | low | static | Consistent, smaller arg surface | S | **UNVERIFIED** |
| jetank_ros_main-32 | footprint | ninja is installed but colcon builds with Unix Makefiles | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:22` | low | static | Faster incremental/parallel builds of the C++ packages; or -1 dependency if ninja is removed | S | **UNVERIFIED** |
| jetank_ros_main-33 | footprint | 1.2 MB pixi.lock and LICENSE duplicated inside the package (workspace_template) and prone to drift | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.lock:1` | low | static | Smaller repo; no drift between the shipped and the live lock | S | **UNVERIFIED** |
| jetank_ros_main-34 | minimality | Stale docs contradict launch defaults (SIM_CONTROL.md) and ~2,200 lines of historical plans ship in the package | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/SIM_CONTROL.md:9` | low | static | Correct docs; ~2 k lines and 100 KB out of the seed repo | S | **UNVERIFIED** |
| jetank_ros_main-35 | runtime | unified.rviz enables both compressed camera streams, the point cloud and Nav2/AMCL displays by default | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/rviz/unified.rviz:66` | low | static | Avoids 2x JPEG encoding + point-cloud streaming on the Jetson unless requested | S | **UNVERIFIED** |
| jetank_ros_main-36 | minimality | Ten verbose DeclareLaunchArgument blocks + ten add_action lines could be a single list | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:108` | low | static | -50 to -60 LOC in the most-read launch file | S | **UNVERIFIED** |
| jetank_ros_main-24 | runtime | Fixed worst-case TimerAction staggers (up to 46 s sim, 28 s hardware) instead of readiness events | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp.launch.py:77` | low | static | Tens of seconds off each bring-up; no more timing races | M | **UNVERIFIED** |
| jetank_ros_main-25 | minimality | Detector include + SetParameter + lifecycle-activation block copy-pasted across three launch files | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/sim_demo.launch.py:188` | low (orig medium) | static | -60 to -80 LOC; one place to change detector/web wiring | M | confirmed |

### jetank_ros_main-01 — Static world->base_footprint TF gives base_footprint two parents (odom + world)

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:203` · package jetank_ros_main · lens runtime · severity high · evidence measured · effort S · verdict confirmed

Description: When enable_moveit:=true, unified.launch.py spawns a static_transform_publisher process publishing world->base_footprint, while robot_controller (launched by the same file via motor_controller.launch.py) publishes odom->base_footprint on /tf at ~30 Hz. A TF frame may only have one parent; tf2 buffers in every consumer (move_group, Nav2, RViz, base_approach_node) flip between the two parents, produce lookup errors/warnings, and any map->odom->base_footprint chain from Nav2/SLAM is broken while MoveIt is enabled. It also costs a whole extra process (28.8 MB RSS measured) for an identity transform. On the Jetson this is wasted memory plus TF churn in every node that holds a tf2 buffer.

Evidence detail: read_topic /tf_static returned world->base_footprint (identity) from node /world_to_base_footprint_tf; read_topic /tf returned odom->base_footprint messages from robot_controller at ~30 Hz (get_topic_hz /tf = 40.3 Hz, /odom = 30.3 Hz). profile_node /world_to_base_footprint_tf: RSS 28.85 MB, 11 threads. SRDF (jetank_moveit_config) declares virtual_joint parent_frame="world" child_link="base_footprint".

Estimated gain: Removes 1 process (~29 MB RSS), eliminates duplicate-parent TF churn in all tf2 consumers; fixes MoveIt+odom/Nav2 frame tree conflict

Fix sketch: Delete the world_to_base_tf Node (lines 203-212) and its add_action (387). Either change the SRDF virtual joint parent to 'odom' (planning frame = odom, which robot_controller already publishes) or, if a 'world' root is required, publish world->odom (not world->base_footprint) so base_footprint keeps its single odom parent.

Verifier (confirmed, adjusted high): unified.launch.py:203-212 spawns static_transform_publisher world->base_footprint when enable_moveit is true (added at :387), and motor_launch is included unconditionally at :249/:390. robot_controller.cpp:180-185 broadcasts odom->base_footprint whenever publish_odom is true (default true at :34), so base_footprint gets two parents with MoveIt enabled; jetank.srdf:120 declares the virtual joint parent as 'world', confirming why the static TF was added. Nothing in the code gates or reconciles the two publishers, so the duplicate-parent conflict and the extra process are real.

### jetank_ros_main-02 — joint_state_publisher always started; duplicates joint_state_broadcaster with all-zero joint states when MoveIt is enabled

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/urdf.launch.py:50` · package jetank_ros_main · lens runtime · severity medium (orig high) · evidence measured · effort S · verdict confirmed

Description: urdf.launch.py unconditionally starts joint_state_publisher (JSP). unified.launch.py includes urdf.launch.py and, with enable_moveit:=true, also spawns joint_state_broadcaster which publishes the real servo positions on /joint_states. Two publishers on /joint_states means robot_state_publisher alternately receives real positions and JSP's constant zeros for S1..S5/gripper/wheels, so the arm TF chain flickers between the commanded pose and zero; MoveIt's planning scene monitor sees the same conflict. JSP is a Python process costing 68 MB RSS and ~1.6% CPU mean (peaks 9.9%) on the Jetson for a 10 Hz stream of zeros.

Evidence detail: get_node_list shows /joint_state_publisher and /joint_state_broadcaster_spawner both alive. get_node_info /joint_state_publisher: publishes /joint_states. get_topic_hz /joint_states = 10.0 Hz; read_topic /joint_states: 10 joints all position 0.0, frame_id ''. profile_node /joint_state_publisher over 10 s: cpu mean 1.56%, p95 9.41%, RSS 68.2 MB, 11 threads. get_node_params: rate=10, publish_default_positions=true. (In this session ros2_control_node had crashed so only JSP was publishing; with a healthy controller_manager both publish.)

Estimated gain: ~68 MB RSS and ~1.5-2% CPU saved; removes conflicting /joint_states publisher when ros2_control is up

Fix sketch: Add a 'use_jsp' launch arg to urdf.launch.py (default true) and set condition=UnlessCondition/IfCondition on the JSP Node; in unified.launch.py pass use_jsp = NOT enable_moveit (PythonExpression) so JSP only runs when no joint_state_broadcaster exists. Alternatively drop JSP entirely and rely on ros2_control in every hardware bringup.

Verifier (confirmed, adjusted medium): The fix is safe and in-scope: urdf.launch.py:50-55 starts JSP unconditionally, and its only other consumers (jetank_navigation/launch/navigation_full.launch.py:71-76 and pixi.toml:49 `urdf` task) include it without arguments, so a `use_jsp` arg defaulting to true leaves them unchanged; only unified.launch.py:193-200 with enable_moveit:=true would change, where moveit_bringup.launch.py:132 already spawns joint_state_broadcaster as the real /joint_states source. One side effect the fix_sketch omits: the ros2_control block covers only arm/gripper joints (components/arm.xacro, gripper.xacro; wheels.xacro:24 continuous joints have none), so dropping JSP removes wheel joint states and robot_state_publisher stops emitting the four *_wheel_link TFs — grep found no consumer of those frames outside jetank_description (test_urdf.py only checks the URDF), so this is a cosmetic RViz change, not a breakage. No package merging or perception abstraction is touched. Severity lowered to medium: the measured cost (68 MB, ~1.6% CPU) is real but the flicker only occurs in the enable_moveit path, and the "always drop JSP" alternative would break the non-MoveIt/nav path where no other joint_state source exists.

Duplicate folded in: **cross-10** — same JSP/JSB double-publisher issue; cross-10 was independently confirmed medium by the verifier.

### jetank_ros_main-04 — RPLidar include is unconditional; node dies at start when no /dev/ttyUSB0, no enable arg or respawn

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:279` · package jetank_ros_main · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: lidar.launch.py is included with no condition and no use_sim_time pass-through. In the live bringup the rplidar_node died 0.4 s after start (exit 255, serial port absent) and nothing restarts or reports it; the LogInfo banner (line 356) still claims 'LiDAR: RPLidar C1M1 (hardware)'. Every unified bringup without the lidar pays a process spawn + failure, and there is no way to run a base-only/arm-only session without the driver attempting the port.

Evidence detail: Launch log ~/.ros/log/2026-09-26-09-15-05-559512-ubuntu-8228/launch.log: '[ERROR] [rplidar_node-7]: process has died [pid 8242, exit code 255 ...]'. ls /dev/ttyUSB* -> no such file; get_topic_list shows no /scan topic; get_node_list shows no rplidar node.

Estimated gain: Avoids a failed process per bringup; enables lidar-less sessions; clearer failure signalling

Fix sketch: Add DeclareLaunchArgument('enable_lidar', default 'true') and condition=IfCondition(enable_lidar) on laser_scan_launch; optionally a RegisterEventHandler(OnProcessExit) that logs a clear warning. Same treatment for the IMU include (271-276).

Verifier (confirmed, adjusted medium): unified.launch.py:279-283 includes lidar.launch.py with no condition and no launch_arguments, and the only DeclareLaunchArguments (lines 108-179) contain no enable_lidar/enable_imu; imu_launch (271-276) is likewise unconditional. jetank_navigation/launch/lidar.launch.py has no respawn or OnProcessExit handler, and the LogInfo at line 356 hardcodes 'LiDAR: RPLidar C1M1 (hardware)'. The cited log (~/.ros/log/2026-09-26-09-15-05-559512-ubuntu-8228/launch.log line 41) shows rplidar_node-7 died with exit 255 ~0.4 s after start, and /dev/ttyUSB* is currently absent, matching the finding.

### jetank_ros_main-06 — ros2_control_node crash leaves three spawner processes polling forever; no OnProcessExit handling in the MoveIt layer include

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:298` · package jetank_ros_main · lens runtime · severity medium · evidence measured · effort M · verdict confirmed

Description: With hardware:=serial the included moveit_bringup started ros2_control_node, which aborted (exit -6) 1.3 s later. The three controller spawners (Python, ~64-66 MB RSS each) keep retrying /controller_manager/list_controllers every 10 s indefinitely, and move_group stays up with no controllers. unified.launch.py provides no event handler to detect the crash, so ~200 MB RSS of zombie spawners plus log spam persist for the whole session on the Jetson.

Evidence detail: Launch log: '[ERROR] [ros2_control_node-9]: process has died [pid 8266, exit code -6 ...]'; repeated '[WARN] ... Could not contact service /controller_manager/list_controllers' every 10 s from spawner-10/11/12 for >400 s. ps: spawner PIDs 8286/8294/8296 RSS 66400/64252/64460 KB, etimes 438 s. get_node_list has no /controller_manager.

Estimated gain: ~195 MB RSS and 3 idle Python processes reclaimed after a controller_manager failure; faster failure visibility

Fix sketch: In unified.launch.py (or moveit_bringup) add RegisterEventHandler(OnProcessExit(target_action=<ros2_control_node>, on_exit=[LogInfo, EmitEvent(Shutdown)])) or pass '--controller-manager-timeout <n>' to the spawners so they exit; surface the JetankSerial open failure as a fatal launch error.

Verifier (confirmed, adjusted medium): Live ps (this session) shows the three spawners (PIDs 8286/8294/8296) still alive at etimes 24773 s with RSS 59604/57180/57356 KB (~174 MB total) plus an idle move_group, and no ros2_control_node process exists. moveit_bringup.launch.py:96-133 defines ros2_control_node and the spawners with no RegisterEventHandler/OnProcessExit/Shutdown and no --controller-manager-timeout (grep returned nothing), and unified.launch.py:298-309 just includes it with no handler. The ~195 MB gain is plausible (measured ~174 MB now, RSS drifts) and a Jetson-relevant leak, but it is a failure-path issue rather than a hot path, so medium is honest.

### jetank_ros_main-10 — package.xml declares all seven sibling packages (including jetank_simulation) as build+exec <depend> for a package that compiles nothing

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:26` · package jetank_ros_main · lens footprint · severity medium · evidence static · effort S · verdict confirmed

Description: <depend> on jetank_motor_control, jetank_perception, jetank_navigation, jetank_description, jetank_simulation, jetank_moveit_config and jetank_web_control makes colcon order this ament_python package after every one of them, and makes jetank_simulation (ros_gz, ign_ros2_control, Gazebo worlds) a hard dependency of the hardware seed package on the Jetson. Only launch-time (exec) dependencies exist here; jetank_simulation is needed only by the sim launch files. This also blocks 'colcon build --packages-up-to jetank_ros_main' from being a light hardware build.

Evidence detail: package.xml lines 26-32 use <depend>; setup.py has no build step needing these packages; jetank_simulation is referenced only by gazebo_sim.launch.py:13 (get_package_share_directory) and sim launches.

Estimated gain: Hardware target can build/install without Gazebo stack; correct dependency graph

Fix sketch: Change lines 26-32 to <exec_depend>; keep jetank_simulation as exec_depend only (or drop it and let the sim launch files fail fast when the package is absent). Consider a condition="$SIM" attribute if the sim stack should be excluded from robot installs.

Verifier (confirmed, adjusted medium): package.xml:26-32 declares all seven sibling packages with <depend>, while the package is ament_python with no build step (setup.py:7-36 only installs launch/config/rviz files and two console scripts), so only exec-time dependencies exist. jetank_simulation is referenced solely by launch/gazebo_sim.launch.py:13,60,64 (and sim_demo/urdf comments), and the package already uses the correct pattern for jetank_detection at line 35 (<exec_depend>), confirming the inconsistency. The comment on line 25 ("ensures all packages are built together") shows the <depend> is deliberately forcing build ordering, which is exactly the dependency-graph weight the finding describes.

Duplicate folded in: **cross-14** — same <depend> on all siblings; cross-14 adds that jetank_manipulation (nodes started by sim_demo/mobile_grasp*.launch.py) is not declared at all and should be added as exec_depend.

Duplicate folded in: **footprint-31** — same <depend> issue; footprint-31 suggests a condition="$JETANK_SIM == 1" attribute for jetank_simulation/jetank_moveit_config and declaring geometry_msgs, nav_msgs, sensor_msgs, python3-yaml used by topics.py and the scripts.

### jetank_ros_main-31 — Workspace pixi.toml installs ros-humble-desktop, the full Gazebo stack and unused packages on the aarch64 robot target

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:80` · package jetank_ros_main · lens footprint · severity medium · evidence static · effort M · verdict confirmed

Description: The single default environment (platforms linux-aarch64 + linux-64) pulls ros-humble-desktop (RViz2, Qt, rqt, demos, tutorials) plus ros-gz-sim/ros-gz-bridge/ros-gz-image/ign-ros2-control (126-129) onto the headless Jetson, contributing to the ~6 GB env (CLAUDE.md) and 33,711-line lock. Several entries have no consumer anywhere in the workspace: ros-humble-moveit-servo (123; grep 0 hits), ros-humble-ros-gz-image (128; 0 hits), ros-humble-joint-state-publisher-gui (106; only jetank_motor_control/launch/test_urdf.launch.py). Disk, pixi install time and library load paths on the robot all grow with this.

Evidence detail: pixi.toml lines 80, 106, 123, 126-129; grep across src/ for moveit_servo/servo_node, ros_gz_image/image_bridge returned no launch/config references; joint_state_publisher_gui referenced only by jetank_motor_control/launch/test_urdf.launch.py. Root pixi.toml is byte-identical to the template (diff).

Estimated gain: Multiple GB smaller env on the Jetson; faster pixi install; fewer packages to resolve

Fix sketch: Split into pixi features: [feature.robot] (ros-base, ros2_control, nav2, slam, rplidar, moveit, cv-bridge...) and [feature.dev] (desktop/rviz2, ros-gz-*, ign-ros2-control, jsp-gui) with environments 'robot' (aarch64) and 'dev' (linux-64); drop moveit-servo and ros-gz-image; keep one lock.

Verifier (confirmed, adjusted medium): Measured this session: .pixi/envs/default is 5.9 GB on this aarch64 host (du -sh) and pixi.lock is 33,711 lines; root pixi.toml is byte-identical to the template, has no [feature]/[environments] sections, and pulls ros-humble-desktop (line 80), moveit-servo (123) and ros-gz-sim/bridge/image/ign-ros2-control (126-129) into the single environment used on the Jetson. moveit-servo has zero consumers outside pixi.toml/pixi.lock (grep confirms), so dropping it is free; ros-gz-image is NOT unused as claimed — jetank_simulation/package.xml:15 declares exec_depend ros_gz_image and plans/sock-pointcloud-plan.md:13 documents it bridging sim camera topics — it is sim-only, which still supports moving it to a dev feature but not deleting it. The "multiple GB" gain is plausible (RViz/Qt, rqt, Gazebo Fortress/ogre2 are the heaviest non-robot chunks) but unmeasured; the cost is disk and pixi install time only, not runtime CPU/latency, so medium is an honest ceiling for the footprint lens.

### jetank_ros_main-05 — Stereo camera include always launches the pixi stereo_camera_node, which cannot open CSI under pixi and dies; a second copy must be run by hand

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:256` · package jetank_ros_main · lens runtime · severity low (orig medium) · evidence measured · effort S · verdict confirmed

Description: camera_launch includes jetank_perception/stereo_camera.launch.py unconditionally. Under the pixi env the node exits (OpenCV without GStreamer backend — documented in mobile_grasp_hw.launch.py:29-32), so every unified hardware bringup starts a doomed process and the user separately runs the same launch under /opt/ros/humble. The optical frame_id / namespace overrides carefully passed here (lines 260-267) never reach the node that actually runs.

Evidence detail: Launch log: '[ERROR] [stereo_camera_node-5]: process has died [pid 8238, exit code 1 ...]' 2 s after start. ps shows a separate '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch jetank_perception stereo_camera.launch.py namespace:=stereo_camera publish_camera_transforms:=false' (pid 7948) started by hand, without left_frame_id/right_frame_id overrides. get_node_list shows exactly one /stereo_camera/stereo_camera_node.

Estimated gain: One fewer crashed process per bringup; single source of truth for camera launch args (frame ids actually applied)

Fix sketch: Add 'enable_camera' launch arg (default false on hardware, or auto-detect via an env var) so unified skips the include when the camera is run under system ROS; document the exact system-ROS command including left_frame_id/right_frame_id/namespace so both paths stay identical.

Verifier (confirmed, adjusted low): The fix is additive and in-scope: gating the include at unified.launch.py:256/391 with an IfCondition on a new `enable_camera` arg touches only jetank_ros_main and does not merge packages or alter jetank_perception's strategy/factory code. The only consumers of unified.launch.py are main.launch.py:28-40 and mobile_grasp_hw.launch.py:83-92, neither of which pass camera args, so a default of 'true' preserves their behavior while 'false' simply drops the process that already dies (mobile_grasp_hw.launch.py:29-32 documents the system-ROS split). One caveat weakens the finding's impact claim: stereo_camera.launch.py:33-42 already defaults left/right_frame_id to the optical frames, so the hand-run system-ROS copy gets identical frame ids and only the namespace/publish_camera_transforms args are at risk of drift; the runtime cost is a single 2 s crashed process, so severity is low.

### jetank_ros_main-07 — Web control enabled by default on hardware bringup: ~8% CPU and 96 MB RSS with no browser connected

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:114` · package jetank_ros_main · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: enable_web_control defaults to 'true', so every unified hardware bringup (and mobile_grasp_hw when the user flips it) runs web_control_node. Measured with no client connected it consumes 7.7% CPU mean and 96 MB RSS — the single largest CPU consumer among the nodes this package starts, while idle. On the Jetson this competes with the stereo/disparity and detector pipelines.

Evidence detail: profile_node /web_control_node over 5 s: cpu mean 7.71%, p95 9.9%, RSS 95.96 MB, 12 threads; no browser session open during measurement (ctx_switches 0/0). Launch was started with enable_web_control:=true (the default).

Estimated gain: ~8% of one core and ~96 MB RSS when the browser UI is not needed

Fix sketch: Default enable_web_control to 'false' in unified.launch.py (mobile_grasp_hw already assumes false) and let main.launch.py drop its explicit override; report the idle CPU usage to jetank_web_control as a separate finding (likely the MJPEG/image subscription is active without viewers).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-08 — topics.py re-opens and re-validates topics.yaml on every accessor call

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/topics.py:27` · package jetank_ros_main · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _load() does a get_package_share_directory lookup, file open, yaml.safe_load and three consistency checks each time any accessor is called. unified.launch.py calls three accessors, sim_demo.launch.py five (camera_left_raw twice, detections_socks twice, detections_socks_debug), mobile_grasp* four each. Each launch-description parse therefore parses the same 45-line YAML 3-5 times and runs the ament index lookup each time. Small, but it is pure repeated I/O at launch time on a slow eMMC/SD.

Evidence detail: Code reading: every public function calls _load(); no memoisation. grep counts of accessor calls per launch file listed above (unified.launch.py:229,236,263; sim_demo.launch.py:165,172,191,192,198).

Estimated gain: 3-5x fewer YAML parses/ament lookups per launch; ~10 LOC simpler

Fix sketch: Decorate _load with functools.lru_cache(maxsize=1) (or load once into a module-level dict). Optionally expose the three names as module constants computed once.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-09 — Unused helpers contract_path() and node_topic_params() in topics.py

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/topics.py:49` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: node_topic_params (lines 49-51) has no caller anywhere in the workspace; contract_path (21-24) is only called by _load. Both are public API surface that nothing uses.

Evidence detail: grep -rl node_topic_params / contract_path across src/ returned only topics.py itself.

Estimated gain: -8 LOC, smaller public surface

Fix sketch: Delete node_topic_params; inline contract_path into _load (or keep it private).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-11 — Unused exec_depends: launch_xml, launch_yaml, urdf, xacro

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:22` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: All launch files in the package are Python (launch_xml/launch_yaml frontends are never used). 'urdf' and 'xacro' are not invoked by this package — jetank_description/launch/robot_description.launch.py runs xacro and already declares it. These declarations pull packages into rosdep/exec resolution for no reason.

Evidence detail: glob launch/*.launch.py = 11 Python files, no .xml/.yaml launch; grep 'xacro' in jetank_ros_main returns only doc comments; urdf.launch.py includes jetank_description's launch instead of running xacro.

Estimated gain: -4 dependency lines; cleaner rosdep set

Fix sketch: Remove <exec_depend>launch_xml</exec_depend>, launch_yaml, urdf, xacro.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-12 — Dependencies actually used are undeclared: tf2_ros, rviz2, python3-yaml, ament_index_python, jetank_manipulation, jetank_detection is exec but sensor/nav/geometry msgs for scripts missing

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:12` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: unified.launch.py runs tf2_ros/static_transform_publisher, rviz.launch.py runs rviz2, topics.py imports yaml and ament_index_python, mobile_grasp*.launch.py run jetank_manipulation nodes, and the two console scripts import geometry_msgs/nav_msgs/sensor_msgs. None are declared, while rclpy (line 12) is declared as a full <depend> although only the diagnostic scripts use it. rosdep/colcon cannot verify the real runtime closure, which matters for the one-clone install this package advertises.

Evidence detail: unified.launch.py:204 package='tf2_ros'; rviz.launch.py:63 package='rviz2'; topics.py:16-18 imports; mobile_grasp.launch.py:101-106 package='jetank_manipulation'; test_drive.py:10-11 and test_cameras.py:10 imports; package.xml has none of these.

Estimated gain: Accurate dependency closure for install.sh/rosdep; rclpy can go if scripts are removed (see -18)

Fix sketch: Add <exec_depend> for tf2_ros, rviz2, python3-yaml, ament_index_python, jetank_manipulation, geometry_msgs, nav_msgs, sensor_msgs; downgrade rclpy to exec_depend (or drop with the scripts).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-13 — ament_lint_auto / ament_lint_common test_depends have no effect in an ament_python package

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/package.xml:43` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_lint_auto and ament_lint_common are CMake-driven (ament_lint_auto_find_test_dependencies); with build_type ament_python they are never invoked, but they still pull the whole ament_lint_common bundle (cppcheck, cpplint, uncrustify, xmllint...) into the test dependency set.

Evidence detail: package.xml:47 <build_type>ament_python</build_type>; no CMakeLists.txt in package; lines 43-44 declare the CMake-only lint packages.

Estimated gain: Fewer test deps resolved by rosdep/CI

Fix sketch: Delete lines 43-44.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-14 — test_copyright.py is permanently skipped: dead test file plus ament_copyright dependency

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/test/test_copyright.py:20` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The test is decorated @pytest.mark.skip and the README table (README.md:322) documents it as skipped. It contributes a 25-line file, an import of ament_copyright and a test_depend (package.xml:14) while never asserting anything.

Evidence detail: test_copyright.py:20 '@pytest.mark.skip(reason=...)'; .pytest_cache nodeids show it collected on every run.

Estimated gain: -25 LOC, -1 test dependency

Fix sketch: Delete test/test_copyright.py and <test_depend>ament_copyright</test_depend>, or add the headers and unskip.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-15 — test_pep257.py is documented as always failing (D213 vs D212) — burns colcon test time with zero signal

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/test/test_pep257.py:22` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: workspace_template/PIXI.md:63-65 states ament_pep257 D213 fails on every run because the repo uses the mutually exclusive D212 style. A test known to always fail is dead weight: it runs pydocstyle over the package every 'pixi run test' and its red result trains people to ignore failures.

Evidence detail: PIXI.md lines 62-65 describe the expected permanent failure; test_pep257.py:22 runs main(argv=['.', 'test']) with no --ignore.

Estimated gain: One fewer always-red test; faster colcon test

Fix sketch: Pass argv=['.', 'test', '--ignore', 'D213'] (ament_pep257 supports --ignore) or drop the test; the same applies to sibling packages.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-16 — Diagnostic scripts use rclpy Rate.sleep() in the same thread as spin_once (blocking/deadlock-prone loop)

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:45` · package jetank_ros_main · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: publish_velocity() creates a Rate via node.create_rate(10) and calls rate.sleep() after rclpy.spin_once(); rclpy's Rate is timer-backed and only wakes when an executor spins the node, so in a single-threaded loop sleep() blocks until the *next* spin_once processes the timer — the loop stalls or hangs, and cmd_vel cadence is not 10 Hz. test_cameras.py:83-87 has the identical pattern. Additionally publish_velocity loops on wall time while the intended target is Gazebo sim time.

Evidence detail: test_drive.py:45-50 and test_cameras.py:83-87: create_rate + spin_once + rate.sleep in one thread; no MultiThreadedExecutor/spin thread exists. Known rclpy pitfall (Rate requires concurrent spinning).

Estimated gain: Deterministic 10 Hz command loop; removes hang risk

Fix sketch: Drop create_rate; use `rclpy.spin_once(self, timeout_sec=0.1)` as the pacing (it already sleeps up to 0.1 s), or `time.sleep(0.1)` after spin_once(timeout_sec=0).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-17 — Duplicated forward/backward test bodies and left/right FPS blocks in the diagnostic scripts

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:73` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: test_forward (73-87) and test_backward (89-103) are identical except sign and label; test_rotation_left/right (105-123) likewise; test_cameras.py:105-117 repeats the interval/fps computation for left and right. Roughly 40 lines can collapse into two parametrised helpers.

Evidence detail: Side-by-side reading of the named line ranges; only the literal velocity and log string differ.

Estimated gain: -40 LOC

Fix sketch: def _drive_and_measure(self, lin, ang, dur, label) computing distance from odom; def _fps(ts) used for both cameras.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-18 — test_drive / test_cameras console scripts are sim-only, hardcode topics that don't match the sim, and are referenced by nothing

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/setup.py:31` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Both scripts' docstrings say 'for JeTank Simulation', yet test_drive publishes Twist on /cmd_vel while the sim controller consumes TwistStamped on /diff_drive_controller/cmd_vel and publishes odom on /diff_drive_controller/odom (SIM_CONTROL.md:57-62), so the script cannot pass in the sim it targets; test_cameras hardcodes /stereo_camera/* topics (lines 21-43) instead of using the topics.py contract this package advertises, and asserts 640x360 (line 139). No launch file, pixi task, plan or README command invokes either entry point. 420 LOC + two console_scripts + rclpy/msg deps are carried for unusable diagnostics.

Evidence detail: grep for 'test_drive|test_cameras' across src/ finds only setup.py and one README sentence; docstrings test_drive.py:3 and test_cameras.py:3; topic mismatch documented in SIM_CONTROL.md:57-62.

Estimated gain: -420 LOC, -2 entry points, allows dropping rclpy/geometry_msgs/nav_msgs/sensor_msgs deps

Fix sketch: Delete jetank_ros_main/scripts/ and the console_scripts entries (or move to jetank_simulation/test as real pytest launch tests that use topics.py and the correct sim topics).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-19 — stereo_camera.launch.py is a pass-through wrapper that also diverges from unified's camera args

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/stereo_camera.launch.py:11` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The file only includes jetank_perception/stereo_camera.launch.py with no arguments, so 'pixi run stereo-camera' (pixi.toml:53) is equivalent to launching the perception package directly — but WITHOUT the namespace/publish_camera_transforms/left_frame_id/right_frame_id overrides that unified.launch.py:260-267 applies. The two paths thus publish different frame_ids (camera_left_link vs camera_left_optical_frame), which docs/hardware-bringup-fetch-sock.md:154-169 warns breaks reprojection.

Evidence detail: stereo_camera.launch.py has no launch_arguments; unified.launch.py:260-267 passes four; pixi.toml task stereo-camera points at the wrapper.

Estimated gain: -20 LOC, one launch file fewer, consistent frame ids across entrypoints

Fix sketch: Delete the wrapper and point the pixi task at 'ros2 launch jetank_perception stereo_camera.launch.py namespace:=stereo_camera publish_camera_transforms:=false left_frame_id:=camera_left_optical_frame right_frame_id:=camera_right_optical_frame', or make the wrapper pass the same args unified does (single helper).

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **cross-18** — same argument-less stereo_camera.launch.py wrapper (and main.launch.py, see jetank_ros_main-20); cross-18 notes the pixi stereo-camera task therefore differs from unified.

### jetank_ros_main-20 — main.launch.py is a 45-line wrapper whose only effect is enable_web_control:=false

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/main.launch.py:28` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Kept for backward compatibility only; it re-declares use_sim_time and includes unified.launch.py with one pinned arg. Two entry points for the same bringup double the documentation surface (README.md:242) and the number of files to keep in sync. If -07 flips unified's default, this wrapper becomes fully redundant.

Evidence detail: main.launch.py:36-39 launch_arguments = {use_sim_time, enable_web_control:'false'}; nothing else.

Estimated gain: -45 LOC, -1 launch file

Fix sketch: Remove main.launch.py and update README/pixi docs to 'unified.launch.py enable_web_control:=false' (or make that the default).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-22 — Optional-layer package paths resolved eagerly at parse time (get_package_share_directory) — hard failure even when the layer is disabled

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:218` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: pkg_web_control (218) and pkg_jetank_moveit (297) are looked up with get_package_share_directory while generate_launch_description runs, so a hardware install without jetank_web_control or jetank_moveit_config cannot launch unified at all, even with enable_web_control:=false / enable_moveit:=false. sim_demo.launch.py already uses the lazy FindPackageShare substitution for the same includes; unified mixes both styles (88-90, 218, 297).

Evidence detail: unified.launch.py:88-90, 218, 297 vs sim_demo.launch.py:104-105, 168-169 (FindPackageShare).

Estimated gain: Optional layers become truly optional; consistent style, ~5 LOC

Fix sketch: Replace os.path.join(get_package_share_directory(pkg), 'launch', f) with PathJoinSubstitution([FindPackageShare(pkg), 'launch', f]) for the conditional includes.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-26 — Redundant use_sim_time=False parameters on hardware nodes

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:77` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: not_sim = {'use_sim_time': False} is injected into four pipeline nodes (136-148) and cmd_vel_bridge (108); False is the rclcpp/rclpy default, so these five parameter entries change nothing. unified.launch.py also passes 'use_sim_time': 'false' explicitly (86) where it is the include's default.

Evidence detail: mobile_grasp_hw.launch.py:77, 86, 108, 136, 138, 144, 147; ROS 2 default use_sim_time=false.

Estimated gain: -6 LOC, clearer intent

Fix sketch: Remove not_sim and the explicit use_sim_time:false entries; keep the docstring note that hardware has no /clock.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-27 — Model path defaults hardcoded to /home/koen in two launch files, expanduser('~') in the third

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:159` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: mobile_grasp_hw.launch.py:159 and mobile_grasp.launch.py:117 default model paths to /home/koen/models/*.pt; sim_demo.launch.py:96 uses os.path.expanduser('~/models/sock_sim.pt'). The same default is thus written three times in two styles, and the hardcoded form breaks for any other user/CI account (the one-clone install this package sells).

Evidence detail: Cited lines.

Estimated gain: Portable defaults, one definition

Fix sketch: Define MODEL_DIR = os.path.expanduser('~/models') in launch_helpers (see -25) or topics.py and reference it from all three launch files.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-28 — gazebo_sim.launch.py declares five conditional includes of the same launch file (one per world name)

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/gazebo_sim.launch.py:57` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: A loop adds five IncludeLaunchDescription actions guarded by LaunchConfigurationEquals('world', name), each carrying identical launch_arguments except the world path. One include with the path built by a substitution (or a 'world' arg with choices=[...] and PathJoinSubstitution([sim_share,'worlds', [world,'.sdf']])) does the same with ~12 fewer lines and fewer launch actions to evaluate. The file also mixes get_package_share_directory (13) and FindPackageShare (60) for the same package.

Evidence detail: gazebo_sim.launch.py:39-45 dict, 57-69 loop; jetank_simulation/launch/gazebo.launch.py accepts 'world' as a path.

Estimated gain: -12 LOC, 4 fewer launch actions, unknown world name rejected at parse time via choices

Fix sketch: DeclareLaunchArgument('world', choices=list(world_files)); world_path = PathJoinSubstitution([FindPackageShare('jetank_simulation'),'worlds', PythonExpression(["{...}['", world, "']"])]) or rename the sdf files so the name maps 1:1 and use [world, '.sdf'].

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-29 — use_sim_time default declared twice in urdf.launch.py

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/urdf.launch.py:26` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: LaunchConfiguration('use_sim_time', default='false') at line 26 and DeclareLaunchArgument('use_sim_time', default_value='false') at 41-45 both provide the default; the LaunchConfiguration default is dead once the argument is declared.

Evidence detail: urdf.launch.py:26 and 41-45.

Estimated gain: -1 redundant default

Fix sketch: LaunchConfiguration('use_sim_time') only.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-30 — use_sim_time not forwarded to the motor and lidar includes while it is to every other include

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:247` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: urdf (198), imu (275), moveit (303), slam (324), nav2 (338) receive use_sim_time, but motor_launch (247-251) and laser_scan_launch (279-283) do not (motor_controller.launch.py declares no such arg either). The use_sim_time arg therefore only half-applies, which is either dead configurability (these are hardware-only nodes, so the arg should not exist for them) or an omission.

Evidence detail: Cited line ranges; motor_controller.launch.py has no DeclareLaunchArgument.

Estimated gain: Consistent, smaller arg surface

Fix sketch: Either document unified.launch.py as hardware-only and drop use_sim_time entirely (sim uses sim_demo), or forward it uniformly.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-32 — ninja is installed but colcon builds with Unix Makefiles

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:22` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The build tasks pass only -DCMAKE_BUILD_TYPE; without -G Ninja CMake defaults to Makefiles, so the 'ninja' dependency (line 74) is dead weight and the C++ packages (perception, motor_control, navigation, detection...) compile with slower make-based incremental builds on the 6-core Orin.

Evidence detail: pixi.toml:22-26 build commands; build/jetank_perception contains no build.ninja (checked ls), confirming the Makefile generator is in use; ninja listed at pixi.toml:74.

Estimated gain: Faster incremental/parallel builds of the C++ packages; or -1 dependency if ninja is removed

Fix sketch: build = "colcon build --symlink-install --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release" (same for build-debug/build-*), or drop ninja from [dependencies].

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-33 — 1.2 MB pixi.lock and LICENSE duplicated inside the package (workspace_template) and prone to drift

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.lock:1` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: workspace_template/pixi.lock (33,711 lines, 1.24 MB) and workspace_template/LICENSE are byte-identical copies of the workspace-root files. install.sh copies them out once (cp -an), after which every 'pixi update' at the root silently diverges from the template unless someone re-copies it back. The lock is the largest file in the repo by far and is not installed by setup.py.

Evidence detail: diff -q root pixi.lock vs template: identical; diff LICENSE: identical; ls -la shows 1,235,882 bytes dated May 30 while root lock may change with updates.

Estimated gain: Smaller repo; no drift between the shipped and the live lock

Fix sketch: Add a pixi task 'sync-template' (cp pixi.toml pixi.lock src/jetank_ros_main/workspace_template/) and a CI check, or make install.sh generate the lock (pixi install) instead of shipping it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-32** — same vendored workspace_template/pixi.lock; footprint-32 adds plans/*.md (180 KB) and docs/images/jetank_real.jpg (125 KB) to the cloned-but-uninstalled weight (see jetank_ros_main-34).

### jetank_ros_main-34 — Stale docs contradict launch defaults (SIM_CONTROL.md) and ~2,200 lines of historical plans ship in the package

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/SIM_CONTROL.md:9` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: SIM_CONTROL.md:9-13 documents sim_demo defaults as world=house, arm=false, web=false, while sim_demo.launch.py:77-94 defaults to sock_arena/true/true. plans/ holds 10 planning documents (~1,680 lines) for work marked done, plus docs/hardware-bringup-fetch-sock.md (490 lines) and a 125 KB 1200x1600 JPEG displayed at 300 px in the README. None are installed, but they are cloned by install.sh on every robot and are the first thing a reader trusts.

Evidence detail: SIM_CONTROL.md lines 9-13 vs sim_demo.launch.py lines 77-94; wc -l plans/*.md = 1,679; docs/images/jetank_real.jpg 125,582 bytes 1200x1600 (file).

Estimated gain: Correct docs; ~2 k lines and 100 KB out of the seed repo

Fix sketch: Fix the SIM_CONTROL.md table; move plans/ to the second-brain vault or an archive branch; downscale the README image to ~600 px.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-35 — unified.rviz enables both compressed camera streams, the point cloud and Nav2/AMCL displays by default

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/rviz/unified.rviz:66` · package jetank_ros_main · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Left Camera (66-69) and Right Camera (71-74) compressed Image displays and the PointCloud2 display (83-89) are Enabled: true. As soon as a remote RViz loads this config, image_transport's lazy publishers on the Jetson start JPEG-encoding both 30 Hz streams and the full point cloud is shipped over Wi-Fi, whether or not the operator looks at them. ParticleCloud (137-143), global/local footprints (167-179) and the Navigation 2 panel are also on, producing TF/topic warnings in SLAM-only sessions. Frame Rate: 30 (213) at the same time.

Evidence detail: unified.rviz lines cited; docs/hardware-bringup says RViz runs on the laptop against the robot (rviz.launch.py docstring 16-18).

Estimated gain: Avoids 2x JPEG encoding + point-cloud streaming on the Jetson unless requested

Fix sketch: Set Enabled: false for Right Camera, Point Cloud, Particle Cloud and the costmap footprints; keep Left Camera off by default too and let the operator toggle; consider Frame Rate: 15.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-36 — Ten verbose DeclareLaunchArgument blocks + ten add_action lines could be a single list

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:108` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 108-186 declare ten arguments in separate 5-15 line blocks (with multi-paragraph descriptions that duplicate the module docstring 14-32) and lines 371-380 add them one by one; the section banners (85-87, 93-95, ...) add ~25 more comment-only lines. mobile_grasp.launch.py shows the compact style (116-122). Roughly 60 lines of the 405 are structural repetition.

Evidence detail: Line ranges cited; descriptions at 155-159 and 170-176 restate docstring lines 22-32.

Estimated gain: -50 to -60 LOC in the most-read launch file

Fix sketch: args = [DeclareLaunchArgument(n, default_value=d, description=s, **kw) for ...]; return LaunchDescription(args + [launch_info, urdf_launch, ...]); keep the detailed prose only in the docstring.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-24 — Fixed worst-case TimerAction staggers (up to 46 s sim, 28 s hardware) instead of readiness events

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp.launch.py:77` · package jetank_ros_main · lens runtime · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: mobile_grasp.launch.py delays move_group 18 s, perception 26 s, detector 30 s, pipeline 34 s, lifecycle configure 40 s, activate 46 s; mobile_grasp_hw.launch.py uses 12/16/22/28 s. These are hard-coded sleeps sized for the slowest observed machine, so every bring-up on the Jetson waits the full budget even when nodes are ready earlier, and a slower run still races. Time-to-ready is pure latency with no feedback.

Evidence detail: TimerAction periods at mobile_grasp.launch.py:77,81,87,107,110,112 and mobile_grasp_hw.launch.py:116,149,152,154.

Estimated gain: Tens of seconds off each bring-up; no more timing races

Fix sketch: Use RegisterEventHandler(OnProcessStart/OnProcessIO) on the gazebo/move_group actions, or a small 'wait_for_topic/service' ExecuteProcess gate, to trigger the next stage; keep a single fallback timeout.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_ros_main-25 — Detector include + SetParameter + lifecycle-activation block copy-pasted across three launch files

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/sim_demo.launch.py:188` · package jetank_ros_main · lens minimality · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: The GroupAction[SetParameter(detections_topic), SetParameter(debug_image_topic), Include(detect_*.launch.py, input_image_topic=camera_left_raw(), continuous='true', ...)] block appears in sim_demo.launch.py:188-204, mobile_grasp.launch.py:87-96 and mobile_grasp_hw.launch.py:116-132, each followed by its own configure/activate mechanism (sim_demo:220-228, mobile_grasp:110-113, mobile_grasp_hw:152-155). The web-control SetParameter+include block is likewise duplicated in unified.launch.py:224-240 and sim_demo.launch.py:162-176. ~80 duplicated lines, and the topics contract wiring must be edited in 3-5 places when it changes.

Evidence detail: Line ranges cited; the bodies differ only in sim/real launch file name and model arg name.

Estimated gain: -60 to -80 LOC; one place to change detector/web wiring

Fix sketch: Add jetank_ros_main/launch_helpers.py with detector_stack(sim: bool, model_path, confidence, delay) and web_control_stack(sim: bool, ...) returning the GroupAction; import it from the launch files (module is already installed as a Python package, like topics.py).

Verifier (confirmed, adjusted low): The fix is safe and in-scope: jetank_ros_main is already installed as a Python package via find_packages() (setup.py:10) and all four launch files already import jetank_ros_main.topics (sim_demo.launch.py:43, mobile_grasp.launch.py:24, mobile_grasp_hw.launch.py:47, unified.launch.py:63), so adding a launch_helpers.py sibling introduces no new install mechanism. Grepping src/ finds no package outside jetank_ros_main importing jetank_ros_main modules, and other packages only reference these launch files by path (jetank_moveit_config, jetank_manipulation), which an internal refactor does not change; no package merge or perception abstraction is touched. The duplicated blocks are confirmed at sim_demo.launch.py:188-204, mobile_grasp.launch.py:87-96, mobile_grasp_hw.launch.py:116-132 (detector) and sim_demo.launch.py:162-176 / unified.launch.py:224-240 (web), differing only in sim/real launch name, model_path_sim/model_path_real, optional confidence, web_port and raw-vs-compressed topic — all expressible as helper parameters. One caveat: the helper must keep the confidence and web_port args optional so behavior stays identical, and the per-file activation timers (polling vs fixed TimerAction) are correctly left out of the sketch.

## jetank_motor_control

Coverage: 23 files read; 10 measurements run. Notes: All 20 non-git files in the package were read in full (2353 lines incl. README/LICENSE/.gitignore). .git/ contents were not read (not source). No file was unreadable. Runtime claims for the ros2_control serial plugin (findings 16-19) are static because the plugin's host process is dead; robot_controller findings are backed by live measurements on an idle robot (no cmd_vel traffic during the session, so cmd_vel-callback I2C cost could not be timed). Finding 01's claim that PCA9685 channels 0-3 are unwired is inferred from the JetBot/Waveshare HAT layout and needs hardware confirmation.

Measurements:
- mcp__ros2-mcp__get_node_list: /robot_controller live; no ros2_control_node/controller_manager (spawners present but CM dead, as stated)
- mcp__ros2-mcp__get_node_info /robot_controller: pubs /odom, /robot_status, /tf; sub /cmd_vel
- mcp__ros2-mcp__get_node_params /robot_controller: odom_rate=30.0, publish_odom=true, left_motor=0, right_motor=1, no *_channel params
- mcp__ros2-mcp__profile_node /robot_controller 10 s: CPU mean 1.19% / p95 9.9% / peak 9.9%, RSS 29.5 MB, 11 threads, 18 fds, ctx switches 408 vol / 189 invol
- mcp__ros2-mcp__measure_topic_perf /odom 10 s: 30.302 Hz, 22509 B/s, 303 msgs, jitter 2.09 ms, latency p50 2.35 / p95 6.4 / p99 8.32 ms, 0 drops
- mcp__ros2-mcp__get_topic_hz /robot_status 5 s: 1.0 Hz
- mcp__ros2-mcp__get_topic_bw /tf 5 s: 34527 B/s aggregate (multiple publishers; robot_controller share not isolated)
- mcp__ros2-mcp__read_topic /robot_status: 'Robot Status. Left=0 Right=0 Standby: Active' (robot idle during all measurements)
- ps -p 8236: robot_controller loaded with --params-file install/jetank_ros_main/share/jetank_ros_main/config/motor_params.yaml, 1.4% CPU, RSS 28.7 MB
- Not measured: serial hardware plugin timing (ros2_control_node not running: servo id 1 no ping); I2C per-transaction latency (no read-only tool; hardware-test MCP forbidden)

Findings: 36 (high 0, medium 3, low 33).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_motor_control-17 | runtime | write() re-sends unchanged goal positions every cycle (including S5, which no controller commands) | `src/hardware/jetank_serial_hardware.cpp:378` | medium | static | Near-zero write traffic when the arm is holding; 1 fewer transaction per cycle for S5 always | S | confirmed |
| jetank_motor_control-18 | runtime | 12 ms reply timeout is ~10x longer than a 1 Mbps status frame needs; a single silent servo overruns the 20 ms cycle | `src/hardware/feetech_bus.cpp:142` | medium | static | Worst-case per-cycle blocking drops from 120 ms to ~20-30 ms | S | confirmed |
| jetank_motor_control-16 | runtime | Hardware read()/write() perform 2 blocking serial transactions per servo per 50 Hz cycle (10 total) inside the controller_manager loop | `src/hardware/jetank_serial_hardware.cpp:346` | medium (orig high) | static | ~50% fewer bus transactions per cycle; removes up to 60 ms worst-case blocking on write timeouts | M | confirmed |
| jetank_motor_control-01 | runtime | setValue issues 5 blocking I2C bursts per motor per cmd_vel (10 per Twist) inside the subscriber callback | `src/motor/motor.cpp:124` | low (orig medium) | measured | ~40-60% fewer I2C transactions per command (drop channel 0-3 writes; PCA9685 ALL_LED or contiguous-register burst for pins 8-10 / 11-13 could collapse to 1-2 writes) | S | confirmed |
| jetank_motor_control-03 | runtime | safetyCheck re-issues stop() (4 I2C writes) every 500 ms forever while the robot is idle | `src/motor/robot_controller.cpp:204` | low | measured | Removes all idle I2C traffic and the periodic WARN | S | **UNVERIFIED** |
| jetank_motor_control-04 | runtime | Odometry + TF published at 30 Hz with unchanged pose while stationary | `src/motor/robot_controller.cpp:141` | low | measured | ~22 KB/s /odom + share of /tf while idle; small CPU reduction | S | **UNVERIFIED** |
| jetank_motor_control-05 | runtime | odom timer period truncated to integer milliseconds (30 Hz becomes 30.3 Hz) | `src/motor/robot_controller.cpp:89` | low | measured | Exact configured rate | S | **UNVERIFIED** |
| jetank_motor_control-11 | minimality | robot_status topic has no consumer in the workspace; drags std_msgs and <sstream> | `src/motor/robot_controller.cpp:188` | low | measured | ~20 LOC, one timer, one publisher, std_msgs build/exec dep | S | **UNVERIFIED** |
| jetank_motor_control-14 | minimality | left_motor/right_motor parameters are over-general and the launch config passes keys the node never declares | `src/motor/robot_controller.cpp:21` | low | measured | Removes a misleading config surface; prevents silent misconfiguration | S | **UNVERIFIED** |
| jetank_motor_control-02 | runtime | No change detection: identical cmd_vel values re-write the PCA9685 every message | `src/motor/motor.cpp:108` | low (orig medium) | static | Eliminates most I2C traffic during steady-state driving | S | confirmed |
| jetank_motor_control-06 | runtime | cmd_vel subscription QoS depth 10 lets stale commands queue behind blocking I2C | `src/motor/robot_controller.cpp:63` | low | static | Bounded command latency, fewer wasted I2C writes under bursts | S | **UNVERIFIED** |
| jetank_motor_control-07 | runtime | Missing I2C device causes RCLCPP_ERROR on every register write in the hot path | `src/motor/motor.cpp:170` | low | static | No log storm on degraded hardware | S | **UNVERIFIED** |
| jetank_motor_control-09 | minimality | try/catch blocks around code that cannot throw (setValue, setPin, constructor self-throw) | `src/motor/motor.cpp:110` | low | static | ~15 LOC removed, clearer error propagation | S | **UNVERIFIED** |
| jetank_motor_control-10 | minimality | setPin validates constant internal arguments on every call | `src/motor/motor.cpp:295` | low | static | ~12 LOC | S | **UNVERIFIED** |
| jetank_motor_control-12 | minimality | Redundant second clamp after wheel-velocity normalisation | `src/motor/robot_controller.cpp:127` | low | static | 2 LOC, clearer intent | S | **UNVERIFIED** |
| jetank_motor_control-13 | minimality | Duplicate shutdown logging: on_shutdown lambda and destructor both log; lambda holds node alive | `src/motor/robot_controller.cpp:260` | low | static | 5 LOC | S | **UNVERIFIED** |
| jetank_motor_control-15 | minimality | motor.hpp pulls the whole rclcpp/rclcpp.hpp for one rclcpp::Logger member | `include/motor.hpp:6` | low | static | Faster compile of 3 TUs on the Jetson | S | **UNVERIFIED** |
| jetank_motor_control-19 | runtime | tcflush(TCIFLUSH) syscall before every packet plus echo-compare on every RX chunk | `src/hardware/feetech_bus.cpp:250` | low | static | A few hundred microseconds per cycle | S | **UNVERIFIED** |
| jetank_motor_control-20 | minimality | on_activate uses string-keyed map lookups although resolved pointers already exist | `src/hardware/jetank_serial_hardware.cpp:293` | low | static | Removes an invariant hazard; 2 LOC | S | **UNVERIFIED** |
| jetank_motor_control-22 | minimality | expect_write_replies parsing enumerates 8 string spellings by hand | `src/hardware/jetank_serial_hardware.cpp:152` | low | static | ~10 LOC | S | **UNVERIFIED** |
| jetank_motor_control-23 | minimality | jetank_serial_hardware.cpp includes the full rclcpp umbrella for logging only | `src/hardware/jetank_serial_hardware.cpp:110` | low | static | Compile-time reduction for the plugin TU | S | **UNVERIFIED** |
| jetank_motor_control-24 | minimality | S5_joint exported to the hardware interface but claimed by no controller | `config/ros2_control.xacro:112` | low | static | 1-2 fewer serial transactions per 50 Hz cycle | S | **UNVERIFIED** |
| jetank_motor_control-25 | minimality | Dead xacro blocks: commented-out S4_joint and <gazebo reference='S4_joint'> for a joint that is not exported | `config/ros2_control.xacro:100` | low | static | ~15 LOC | S | **UNVERIFIED** |
| jetank_motor_control-27 | minimality | Misleading comment: diff_drive cmd_vel_timeout 0.5 s claims to match robot_controller, which uses 1.0 s | `config/jetank_controllers.yaml:154` | low | static | Consistent sim/hw safety timeout | S | **UNVERIFIED** |
| jetank_motor_control-29 | footprint | test_urdf.launch.py uses joint_state_publisher_gui and rviz2, which are not declared; launch duplicates jetank_description's robot_description.launch | `launch/test_urdf.launch.py:355` | low | static | 84 LOC + 3 exec_depends (xacro, joint_state_publisher, robot_state_publisher) removable from this package | S | **UNVERIFIED** |
| jetank_motor_control-30 | footprint | Deprecated gazebo_sim.launch.py stub is still installed | `launch/gazebo_sim.launch.py:292` | low | static | 30 LOC, one fewer installed file | S | **UNVERIFIED** |
| jetank_motor_control-31 | footprint | Global include_directories(include) is redundant with per-target include dirs | `CMakeLists.txt:46` | low | static | ~6 LOC, cleaner target model | S | **UNVERIFIED** |
| jetank_motor_control-32 | footprint | motor.hpp is installed at the include root although it is a private implementation header | `CMakeLists.txt:72` | low | static | Avoids polluting the workspace include root; removes a stray installed file | S | **UNVERIFIED** |
| jetank_motor_control-33 | footprint | Plugin description exported twice (CMake macro and package.xml <export>) | `package.xml:157` | low | static | 2 LOC, no duplicated registration | S | **UNVERIFIED** |
| jetank_motor_control-34 | footprint | gtest target exists only to check type traits that could be static_asserts in the header | `test/test_motor_header.cpp:292` | low | static | One fewer test binary/link on the Jetson; faster colcon test | S | **UNVERIFIED** |
| jetank_motor_control-35 | minimality | README describes the hardware plugin as a non-existent stub and libgpiod as a dependency | `README.md:58` | low | static | Documentation accuracy; prevents misdirected work | S | **UNVERIFIED** |
| jetank_motor_control-36 | runtime | Throttled WARN every 5 s forever while the robot is legitimately idle | `src/motor/robot_controller.cpp:206` | low | static | Quieter logs, less /rosout traffic | S | **UNVERIFIED** |
| jetank_motor_control-37 | minimality | readRegister lacks the fd guard the other I2C helpers have; setPWM ignores writeRegister results | `src/motor/motor.cpp:206` | low | static | Consistent error handling, fewer wasted syscalls on failure | S | **UNVERIFIED** |
| jetank_motor_control-08 | runtime | Two Motor instances each open /dev/i2c-7 and re-run the PCA9685 reset/prescale sequence | `src/motor/motor.cpp:90` | low | static | 1 fd, 1 reset sequence; ~50 LOC simpler | M | **UNVERIFIED** |
| jetank_motor_control-21 | minimality | Nested std::map<string, map<string,double>> storage for 9 joints where flat vectors suffice | `include/jetank_motor_control/jetank_serial_hardware.hpp:90` | low | static | ~30 LOC simpler on_init, fewer heap nodes | M | **UNVERIFIED** |
| jetank_motor_control-26 | minimality | arm_controller / gripper_controller parameters duplicated between jetank_controllers.yaml and config/controllers/*.yaml | `config/jetank_controllers.yaml:41` | low | static | ~50 LOC of duplicated config | M | **UNVERIFIED** |

### jetank_motor_control-17 — write() re-sends unchanged goal positions every cycle (including S5, which no controller commands)

`src/hardware/jetank_serial_hardware.cpp:378` · package jetank_motor_control · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: There is no last-sent-ticks cache, so every 20 ms each servo receives a WRITE of the same goal. S5_joint (servo 5) is excluded from arm_controller (jetank_controllers.yaml:39-46) so its command never changes after activation, yet it costs a full transaction + reply wait per cycle; the same applies to all arm joints whenever the trajectory is idle.

Evidence detail: write() at lines 378-385 has no change check; ServoJoint (hpp:54-72) has no last_ticks field.

Estimated gain: Near-zero write traffic when the arm is holding; 1 fewer transaction per cycle for S5 always

Fix sketch: Add `uint16_t last_ticks; bool has_last` to ServoJoint; skip write_goal_position when equal. Optionally drop S5's command interface from the xacro.

Verifier (confirmed, adjusted medium): The cited line number is wrong (file is 290 lines) but the behavior is real: write() at src/hardware/jetank_serial_hardware.cpp:273-285 loops every servo_joints_ entry and calls bus_.write_goal_position unconditionally, with no change check, and ServoJoint (include/jetank_motor_control/jetank_serial_hardware.hpp:54-72) has no last-ticks field. S5_joint declares a position command_interface (config/ros2_control.xacro:112-120) but is excluded from arm_controller (config/jetank_controllers.yaml:39-46), and on_activate seeds commands_ from current state (cpp:192), so S5 gets a redundant WRITE + reply poll (feetech_bus.cpp:225-229, default expect_write_replies_) every 20 ms at update_rate 50 (jetank_controllers.yaml:6). Severity kept medium since the cost is one extra half-duplex transaction per idle joint per cycle, no functional error.

### jetank_motor_control-18 — 12 ms reply timeout is ~10x longer than a 1 Mbps status frame needs; a single silent servo overruns the 20 ms cycle

`src/hardware/feetech_bus.cpp:142` · package jetank_motor_control · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: At 1 Mbps a 6-8 byte status packet takes <0.1 ms and SCS servos reply within ~0.5 ms; kReplyTimeoutMs=12 exists only to match an old usleep-spin budget. With 5 servos, one non-responding servo costs 12 ms in read() plus 12 ms in write() = 24 ms > the 20 ms period, causing controller_manager overruns.

Evidence detail: Constant at line 142 with comment stating it mirrors the previous 60 x 0.2 ms spin. Not timed live (no ros2_control_node running).

Estimated gain: Worst-case per-cycle blocking drops from 120 ms to ~20-30 ms

Fix sketch: Reduce to 2-3 ms (validate on hardware); make it a hardware param.

Verifier (confirmed, adjusted medium): kReplyTimeoutMs is a file-local constant (feetech_bus.cpp:21) used only at :155 and referenced nowhere outside jetank_motor_control (workspace grep found no other package touching FeetechBus/feetech_bus), and the xacro (ros2_control.xacro:44-52) plus jetank_serial_hardware.cpp:29-48 already parse serial_port/baud_rate/expect_write_replies from hardware_parameters, so exposing the timeout as another hardware param is the established pattern and stays inside the package. The arithmetic holds: read() (jetank_serial_hardware.cpp:245) and write() with expect_write_replies=true (feetech_bus.cpp:226-230) each pay the full deadline for a silent servo, so 2x12 ms exceeds the 20 ms period at update_rate 50 (jetank_controllers.yaml:6). One safety caveat the fix_sketch should keep: cutting the wait too short while a servo still replies leaves a late status packet on the half-duplex line, the exact contention risk the code itself documents at feetech_bus.cpp:232-238, so the value must be validated on the Tegra UART rather than just set to 2 ms; defaulting the new param to the current 12 ms and tuning down is behavior-preserving.

### jetank_motor_control-16 — Hardware read()/write() perform 2 blocking serial transactions per servo per 50 Hz cycle (10 total) inside the controller_manager loop

`src/hardware/jetank_serial_hardware.cpp:346` · package jetank_motor_control · lens runtime · severity medium (orig high) · evidence static · effort M · verdict confirmed

Description: read() loops over 5 servos doing read_present_position (send + poll for reply) and write() does write_goal_position (send + poll for the status reply since expect_write_replies=true). Each transaction includes tcflush + write + poll + echo-strip; at 1 Mbps with servo response latency this is roughly 0.5-1 ms each, i.e. ~5-10 ms of the 20 ms cycle spent blocking the realtime update thread, and 12 ms per timed-out transaction. The Feetech protocol supports SYNC WRITE (0x83) to command all servos in one packet, halving transactions.

Evidence detail: ros2_control_node is not running (servo 1 no ping), so no live timing; reasoning from feetech_bus.cpp:218-332 and kReplyTimeoutMs=12 (line 142). update_rate 50 from config/jetank_controllers.yaml:6.

Estimated gain: ~50% fewer bus transactions per cycle; removes up to 60 ms worst-case blocking on write timeouts

Fix sketch: Add FeetechBus::sync_write_goal_positions(ids, ticks) using INST_SYNC_WRITE (0x83, broadcast id 0xFE, no status reply); call it once from write(). Consider reading positions round-robin (one servo per cycle) if 50 Hz state is not required.

Verifier (confirmed, adjusted medium): Structure is confirmed (cited line 346 is wrong; file is 290 lines): read() at jetank_serial_hardware.cpp:243-246 issues read_present_position per servo and write() at :273-282 issues write_goal_position per servo, each waiting on recv_packet with kReplyTimeoutMs=12 (feetech_bus.cpp:21, :155, :225-230) because ros2_control.xacro:53 sets expect_write_replies=true; with 5 servo_ids (xacro:67-119) that is 10 blocking transactions per 20 ms cycle and a 120 ms worst case if all time out. The gain estimates are somewhat inflated: SYNC WRITE cuts 10 to 6 transactions (~40%, not 50%), and the "60 ms write-timeout" case only occurs when servos are absent, in which case the 5 reads still burn 60 ms; nominal savings are ~2.5-5 ms/cycle (unmeasured, node not running). A zero-code mitigation already exists (expect_write_replies=false, xacro:47-53), so effort/value is lower than a high rating suggests; medium is honest.

### jetank_motor_control-01 — setValue issues 5 blocking I2C bursts per motor per cmd_vel (10 per Twist) inside the subscriber callback

`src/motor/motor.cpp:124` · package jetank_motor_control · lens runtime · severity low (orig medium) · evidence measured · effort S · verdict confirmed

Description: Motor::setState performs setPWM(pwm_pin) + setPin(fwd) + setPin(rev) + setPWM(fwd_ch) + setPWM(rev_ch): five separate write(2) syscalls on /dev/i2c-7, each a ~6-byte transaction (~0.6 ms at 100 kHz I2C). Two motors -> ~10 transactions (~6 ms) of blocking I/O executed synchronously in cmdVelCallback on the single-threaded executor, delaying the 30 Hz odom timer and the safety timer. The Waveshare/JetBot PCA9685 motor HAT wiring uses channels 8-13 only; the extra writes to fwd_ch/rev_ch (channels 0-3) look redundant with the pwm_pin+fwd_pin/rev_pin writes (needs HW confirmation).

Evidence detail: profile_node /robot_controller: CPU mean 1.19%, p95 9.9%, 11 threads, RSS 29.5 MB while idle (no cmd_vel). The p95 spikes coincide with timer callbacks; the per-transaction cost is static reasoning (5 write() calls per setState visible at lines 124-138), not timed in this session.

Estimated gain: ~40-60% fewer I2C transactions per command (drop channel 0-3 writes; PCA9685 ALL_LED or contiguous-register burst for pins 8-10 / 11-13 could collapse to 1-2 writes)

Fix sketch: Confirm on the HAT which channels are wired; delete fwd_ch/rev_ch writes if unused. Since pwm/fwd/rev pins are contiguous (8,9,10 and 11,12,13), write all 12 LEDn bytes in one auto-increment burst (raise writeRegisters buffer from 8 to 16).

Verifier (confirmed, adjusted low): motor.cpp:78-97 setState issues setPWM(pwm_pin) + setPin(fwd) + setPin(rev) + setPWM(fwd_ch) + setPWM(rev_ch) = 5 write(2) bursts per motor (setPWM at :228-247 uses one writeRegisters burst per channel; buffer capped at 8 bytes at :148), and robot_controller.cpp:130-131 calls setValue for both motors inside cmdVelCallback, which runs on a single-threaded rclcpp::spin (:265) shared with the 500 ms safety timer and 30 Hz odom timer (:74, :90). Channels 0-3 (fwd_ch/rev_ch, motor.cpp:35-42) are used nowhere else and their physical wiring cannot be confirmed from the workspace or the live-probed bus map; the ~6 ms figure is static reasoning, not measured. Severity lowered: ~6 ms of blocking I/O per Twist at typical 10-20 Hz cmd_vel is well below the timer periods, so the cost is real but modest.

### jetank_motor_control-03 — safetyCheck re-issues stop() (4 I2C writes) every 500 ms forever while the robot is idle

`src/motor/robot_controller.cpp:204` · package jetank_motor_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Once no cmd_vel has arrived for >1 s, every 500 ms tick calls stop(), which performs 2 setPin (2 I2C bursts) per motor on an already-stopped motor. On an idle robot this is 8 I2C transactions/s of pure waste plus a throttled WARN every 5 s to /rosout.

Evidence detail: read_topic /robot_status shows Left=0 Right=0 (idle) while profile_node reports non-zero CPU with p95 9.9%; the periodic stop() path is the only I/O in that state besides odom publishing. Exact I2C share not timed.

Estimated gain: Removes all idle I2C traffic and the periodic WARN

Fix sketch: Keep a `stopped_` flag: call stop()/warn only on the running->stopped transition; reset the flag in cmdVelCallback.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-04 — Odometry + TF published at 30 Hz with unchanged pose while stationary

`src/motor/robot_controller.cpp:141` · package jetank_motor_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: publishOdometry always serialises a 700+ byte nav_msgs/Odometry and a TransformStamped at 30 Hz even when cur_lin_vel_ == cur_ang_vel_ == 0 and the pose is identical to the previous tick. On an idle Jetson this is the node's dominant CPU/DDS cost. Consumers (slam_toolbox/Nav2) tolerate a lower rate when static, though a floor rate should be kept for TF freshness.

Evidence detail: measure_topic_perf /odom: 30.302 Hz, 22509 B/s, 303 msgs/10 s, robot idle (robot_status Left=0 Right=0). get_topic_bw /tf: 34527 B/s aggregate (shared with robot_state_publisher etc.).

Estimated gain: ~22 KB/s /odom + share of /tf while idle; small CPU reduction

Fix sketch: When both velocities are zero, publish at a reduced keep-alive rate (e.g. 5 Hz) or skip integration math; keep full rate while moving.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-05 — odom timer period truncated to integer milliseconds (30 Hz becomes 30.3 Hz)

`src/motor/robot_controller.cpp:89` · package jetank_motor_control · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: `std::chrono::milliseconds(static_cast<int>(1000.0 / odom_rate))` truncates 33.33 ms to 33 ms, producing 30.3 Hz rather than the configured 30 Hz; any non-divisor rate (e.g. 60 -> 16 ms = 62.5 Hz) drifts further. Cosmetic but measurable.

Evidence detail: measure_topic_perf /odom hz = 30.302 with odom_rate param = 30.0 (get_node_params).

Estimated gain: Exact configured rate

Fix sketch: Use `std::chrono::duration<double>(1.0 / odom_rate)` with create_wall_timer (it accepts any chrono duration).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-11 — robot_status topic has no consumer in the workspace; drags std_msgs and <sstream>

`src/motor/robot_controller.cpp:188` · package jetank_motor_control · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: publishStatus builds a fixed string 'Robot Status. Left=.. Right=.. Standby: Active' at 1 Hz. grep over all packages finds no subscriber to robot_status; the 'Standby: Active' literal is meaningless. Removing it also removes last_left_value_/last_right_value_, timer_, the std_msgs include/dependency and <sstream>.

Evidence detail: get_topic_hz /robot_status = 1.0 Hz; read_topic shows constant 'Robot Status. Left=0 Right=0 Standby: Active'. grep -rn robot_status src/ matches only robot_controller.cpp:67.

Estimated gain: ~20 LOC, one timer, one publisher, std_msgs build/exec dep

Fix sketch: Delete publishStatus/timer_/status_publisher_ and the std_msgs dependency in CMakeLists.txt:14,42 and package.xml:114; or replace with diagnostic_msgs only if something needs it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-14 — left_motor/right_motor parameters are over-general and the launch config passes keys the node never declares

`src/motor/robot_controller.cpp:21` · package jetank_motor_control · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Motor hardcodes two pin maps selected by `motor_num == 0` else-branch, so left_motor/right_motor accept any int but only 0 vs non-0 matters. Meanwhile the params file actually loaded by the live node (jetank_ros_main/config/motor_params.yaml: left_motor_forward_channel, left_motor_reverse_channel, right_*_channel) uses keys this node never declares, so they are silently ignored — config and code have diverged.

Evidence detail: Live process cmdline (ps -p 8236): --params-file .../jetank_ros_main/config/motor_params.yaml; get_node_params lists left_motor=0,right_motor=1 and no *_channel keys; motor_params.yaml lines 3-9 contain *_channel keys.

Estimated gain: Removes a misleading config surface; prevents silent misconfiguration

Fix sketch: Either declare the channel parameters and use them in Motor (replacing the hardcoded pin tables), or delete the stale keys from motor_params.yaml and document left_motor/right_motor as 0|1.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-02 — No change detection: identical cmd_vel values re-write the PCA9685 every message

`src/motor/motor.cpp:108` · package jetank_motor_control · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: setValue always calls setState regardless of whether mapped_value/speed changed since the last call. Teleop and Nav2 publish cmd_vel at 10-20 Hz with mostly repeated values, so the full I2C sequence (finding 01) runs on every message even when nothing changes.

Evidence detail: No last_value_/last_speed_ member exists in motor.hpp (lines 23-29); setState is unconditional at line 114.

Estimated gain: Eliminates most I2C traffic during steady-state driving

Fix sketch: Cache the last (value_sign, speed) pair in Motor; return early from setValue when unchanged. Same for stop().

Verifier (confirmed, adjusted low): Finding holds: motor.cpp:65-97 calls setState unconditionally on every setValue and motor.hpp has no cached state, and the only consumers are robot_controller.cpp:130-131 (setValue) and :215-216 (stop) in the same package, so adding a private cache does not touch any cross-package consumer or the perception abstractions. The fix is in-scope but needs two guards to be safe: stop() (motor.cpp:99-103) only clears the direction pins without zeroing pwm/fwd_ch/rev_ch, so the cache must be invalidated in stop() rather than "cached the same way", or a repeated identical cmd_vel after the 1 s safetyCheck watchdog stop (robot_controller.cpp:204) would be skipped and the motor would stay off; and setPWM/writeRegisters return failures silently through the void setState, so the cache must only be updated after successful writes or a transient I2C error would leave the chip out of sync until the value changes. Severity lowered: at 10-20 Hz the redundant traffic is ~100-200 short I2C transactions/s, negligible CPU on an Orin Nano, so the gain is bus hygiene rather than a measurable performance win.

### jetank_motor_control-06 — cmd_vel subscription QoS depth 10 lets stale commands queue behind blocking I2C

`src/motor/robot_controller.cpp:63` · package jetank_motor_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: With reliable/keep_last(10) QoS, a burst of Twist messages is queued and each is executed with its full I2C sequence (finding 01) even though only the newest matters. Up to 10 stale commands (~60 ms of I2C) can be applied before the current one, adding command latency and wasted bus traffic.

Evidence detail: Depth 10 literal at line 63; callback does synchronous I/O (lines 130-131).

Estimated gain: Bounded command latency, fewer wasted I2C writes under bursts

Fix sketch: Use depth 1 (rclcpp::QoS(1)) or SensorDataQoS for /cmd_vel.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-07 — Missing I2C device causes RCLCPP_ERROR on every register write in the hot path

`src/motor/motor.cpp:170` · package jetank_motor_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: If openI2C fails, the constructor only logs and continues; afterwards every writeRegister/writeRegisters call logs an ERROR (5 per setValue, 2 per stop) on every cmd_vel and every 500 ms safety tick. That is unbounded log spam through rosout/DDS when the HAT is absent or on a wrong bus.

Evidence detail: Lines 170-173 and 186-189 log unconditionally on fd_<0; stop() is invoked periodically from safetyCheck (robot_controller.cpp:205).

Estimated gain: No log storm on degraded hardware

Fix sketch: Return false silently (or RCLCPP_ERROR_ONCE) when fd_<0; better, make the constructor failure fatal or expose is_open() so RobotController skips I/O.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-09 — try/catch blocks around code that cannot throw (setValue, setPin, constructor self-throw)

`src/motor/motor.cpp:110` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: setValue wraps setState in try/catch(...) and setPin wraps setPWM in try/catch(std::exception); neither callee throws (all return bool / void with C syscalls). The constructor throws a runtime_error at line 91 only to catch it at line 96 in the same function. This is dead control flow that also hides real error returns (setPWM's bool is ignored).

Evidence detail: No throw expressions in setState/setPWM/writeRegister paths (lines 121-317).

Estimated gain: ~15 LOC removed, clearer error propagation

Fix sketch: Remove the try/catch blocks; replace the constructor throw/catch with `if (!openI2C()) { RCLCPP_ERROR(...); return; }`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-10 — setPin validates constant internal arguments on every call

`src/motor/motor.cpp:295` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: setPin is private and only ever called with fwd_pin/rev_pin (8-13) and literal 0/1, so the pin-range and value checks (lines 295-304) and their error logs are unreachable. They run twice per setState in the hot path.

Evidence detail: All call sites: motor.cpp:126,127,131,132,136,137,144,145 with fixed members and literals.

Estimated gain: ~12 LOC

Fix sketch: Delete the checks or replace with assert(); take `bool high` instead of int.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-12 — Redundant second clamp after wheel-velocity normalisation

`src/motor/robot_controller.cpp:127` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: After lines 121-125 scale both wheel velocities so max(|l|,|r|) <= max_linear_velocity_, dividing by max_linear_velocity_ already yields values in [-1,1]; the std::clamp at 127-128 can never change anything. Also `std::clamp` requires <algorithm>, which is not included (works only transitively).

Evidence detail: Arithmetic at lines 118-128.

Estimated gain: 2 LOC, clearer intent

Fix sketch: Drop the clamp lines; keep the division. Add #include <algorithm> if std::max/min remain.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-13 — Duplicate shutdown logging: on_shutdown lambda and destructor both log; lambda holds node alive

`src/motor/robot_controller.cpp:260` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: rclcpp::on_shutdown registers a lambda capturing `node` by shared_ptr solely to print a message; ~RobotController prints another. The capture keeps the node alive until shutdown-callback teardown, and the two messages are redundant.

Evidence detail: Lines 101-105 and 260-263.

Estimated gain: 5 LOC

Fix sketch: Remove the on_shutdown block; the destructor already stops motors and logs.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-15 — motor.hpp pulls the whole rclcpp/rclcpp.hpp for one rclcpp::Logger member

`include/motor.hpp:6` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The header only needs rclcpp::Logger; including rclcpp/rclcpp.hpp (the umbrella header) makes motor.cpp, robot_controller.cpp and the gtest each parse the full rclcpp tree. motor.cpp re-includes it again at line 45.

Evidence detail: Only rclcpp::Logger and RCLCPP_* macros are used (motor.hpp:23, motor.cpp throughout).

Estimated gain: Faster compile of 3 TUs on the Jetson

Fix sketch: Include <rclcpp/logger.hpp> and <rclcpp/logging.hpp> only; drop the duplicate include in motor.cpp.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-19 — tcflush(TCIFLUSH) syscall before every packet plus echo-compare on every RX chunk

`src/hardware/feetech_bus.cpp:250` · package jetank_motor_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Each send_packet issues a tcflush ioctl, and recv_packet re-runs std::equal against last_tx_ and re-scans buf from `start` after every read() chunk (O(n^2) over chunks). Per transaction this is 3-4 extra syscalls/scans; at 10 transactions per cycle it is modest but avoidable on the RT thread.

Evidence detail: Lines 250, 285-289, 292-308.

Estimated gain: A few hundred microseconds per cycle

Fix sketch: Flush once per cycle (or only after a failed transaction); track the scan position across chunks instead of rescanning from `start`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-20 — on_activate uses string-keyed map lookups although resolved pointers already exist

`src/hardware/jetank_serial_hardware.cpp:293` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: states_[sj.name]["position"] / commands_[sj.name]["position"] use operator[] (which can insert and thus invalidate the 'no inserts after on_init' invariant relied on by the cached pointers) even though sj.state_position/sj.cmd_position were resolved in on_init.

Evidence detail: Lines 293-294 vs pointer resolution at 214-227.

Estimated gain: Removes an invariant hazard; 2 LOC

Fix sketch: Write through sj.state_position / sj.cmd_position with null checks.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-22 — expect_write_replies parsing enumerates 8 string spellings by hand

`src/hardware/jetank_serial_hardware.cpp:152` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 149-162 compare against 'false','False','FALSE','0','true','True','TRUE','1' plus a fallback warning. A one-line lowercase compare (or just accept 'true'/'false') removes ~12 LOC.

Evidence detail: Lines 149-162.

Estimated gain: ~10 LOC

Fix sketch: Lowercase the string once; `expect_write_replies_ = !(v == "false" || v == "0")`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-23 — jetank_serial_hardware.cpp includes the full rclcpp umbrella for logging only

`src/hardware/jetank_serial_hardware.cpp:110` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Only rclcpp::get_logger/RCLCPP_* are used; including rclcpp/rclcpp.hpp adds compile time to the plugin TU. RCLCPP_SHARED_PTR_DEFINITIONS (hpp:33) is also unused by pluginlib's ClassLoader and can go with rclcpp/macros.hpp.

Evidence detail: Usages: rclcpp::get_logger at lines 141,186,251,274,285,301,312 only.

Estimated gain: Compile-time reduction for the plugin TU

Fix sketch: Include <rclcpp/logging.hpp> (and <rclcpp/logger.hpp>); drop the macro and macros.hpp include.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-24 — S5_joint exported to the hardware interface but claimed by no controller

`config/ros2_control.xacro:112` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: S5 (camera servo) has a position command interface + servo_id 5 while arm_controller lists only S1-S3 (jetank_controllers.yaml:43-46). The driver therefore reads and writes servo 5 every cycle purely to hold torque (see finding 17). If torque-hold is the goal, a state-only joint (or torque enable on activate with no per-cycle write) achieves it with zero bus traffic.

Evidence detail: xacro lines 112-120; controller joints list yaml:43-46; README still claims S5 in arm_controller (README.md:67).

Estimated gain: 1-2 fewer serial transactions per 50 Hz cycle

Fix sketch: Remove the command_interface from S5_joint (keep state interfaces) and skip write for joints without cmd_position (already handled by nullptr check).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-25 — Dead xacro blocks: commented-out S4_joint and <gazebo reference='S4_joint'> for a joint that is not exported

`config/ros2_control.xacro:100` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 100-110 are a commented-out joint definition; lines 246-249 configure Gazebo physics for S4_joint although no S4 ros2_control joint exists. Both are dead content processed on every xacro expansion.

Evidence detail: Lines 99-110 and 246-249.

Estimated gain: ~15 LOC

Fix sketch: Delete both blocks (git history keeps them).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-27 — Misleading comment: diff_drive cmd_vel_timeout 0.5 s claims to match robot_controller, which uses 1.0 s

`config/jetank_controllers.yaml:154` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Comment says '500ms (matches robot_controller.cpp pattern)' but robot_controller.cpp:204 stops after 1.0 s. Not a runtime cost, but sim/hardware behaviour diverges silently.

Evidence detail: yaml:154 vs robot_controller.cpp:204.

Estimated gain: Consistent sim/hw safety timeout

Fix sketch: Pick one value (0.5 s is safer) and apply it in both places; fix the comment.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-29 — test_urdf.launch.py uses joint_state_publisher_gui and rviz2, which are not declared; launch duplicates jetank_description's robot_description.launch

`launch/test_urdf.launch.py:355` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The launch starts joint_state_publisher_gui and rviz2 (undeclared deps) and loads jetank_description's URDF — functionality that jetank_description/launch/robot_description.launch.py already provides. Nothing in the workspace references this file except the README. Installing it (CMakeLists.txt:76) ships dead launch code and an implicit GUI dependency from a hardware driver package.

Evidence detail: grep for test_urdf.launch across the workspace matches only jetank_motor_control/README.md:76.

Estimated gain: 84 LOC + 3 exec_depends (xacro, joint_state_publisher, robot_state_publisher) removable from this package

Fix sketch: Delete launch/test_urdf.launch.py; point README at jetank_description's launch.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-30 — Deprecated gazebo_sim.launch.py stub is still installed

`launch/gazebo_sim.launch.py:292` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The file only emits a deprecation LogInfo. It is installed via install(DIRECTORY config launch) and kept 'as a sign-post'; it is dead code shipped to the robot.

Evidence detail: Lines 265-294; CMakeLists.txt:76 installs the whole launch dir.

Estimated gain: 30 LOC, one fewer installed file

Fix sketch: Delete the file (if both launch files go, drop `launch` from the install(DIRECTORY ...) rule).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-31 — Global include_directories(include) is redundant with per-target include dirs

`CMakeLists.txt:46` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Line 46 adds `include` globally while jetank_motor_lib (83-86), jetank_serial_hardware (54-56) and test_motor_header (91) each declare it per-target; the test also gets it transitively via jetank_motor_lib PUBLIC. Three of the four declarations are dead.

Evidence detail: CMakeLists.txt lines 46, 54-56, 83-86, 91.

Estimated gain: ~6 LOC, cleaner target model

Fix sketch: Remove line 46 and line 91; keep the two PUBLIC target_include_directories.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-21** — same global include_directories(include); footprint-21 also notes jetank_motor_lib links all of rclcpp for one rclcpp::Logger (see jetank_motor_control-15).

### jetank_motor_control-32 — motor.hpp is installed at the include root although it is a private implementation header

`CMakeLists.txt:72` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: install(DIRECTORY include/ DESTINATION include/) exports include/motor.hpp as a top-level, un-namespaced header (`#include "motor.hpp"`) into the install tree, even though jetank_motor_lib is a private static lib that is not installed or exported. Only the jetank_motor_control/ subdirectory is legitimately public (plugin headers).

Evidence detail: include/motor.hpp path; no ament_export_targets/ament_export_include_directories in CMakeLists.txt.

Estimated gain: Avoids polluting the workspace include root; removes a stray installed file

Fix sketch: Move motor.hpp to src/motor/ (or include/jetank_motor_control/) and install only include/jetank_motor_control/.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-33 — Plugin description exported twice (CMake macro and package.xml <export>)

`package.xml:157` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: pluginlib_export_plugin_description_file(hardware_interface jetank_hardware.xml) at CMakeLists.txt:60 already registers the plugin in the ament index; the `<hardware_interface plugin="${prefix}/jetank_hardware.xml"/>` export line in package.xml is the legacy ROS 1 mechanism and is redundant. Comment also says 'for future real hardware' although the implementation exists.

Evidence detail: CMakeLists.txt:60 and package.xml:156-157.

Estimated gain: 2 LOC, no duplicated registration

Fix sketch: Remove the export line and stale comment from package.xml.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-34 — gtest target exists only to check type traits that could be static_asserts in the header

`test/test_motor_header.cpp:292` · package jetank_motor_control · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The four tests assert compile-time properties (not default/copy constructible, constructor signature, member-pointer types). Building them requires compiling and linking a gtest binary against rclcpp on every `colcon test`; the same guarantees as `static_assert` cost zero build time. ament_lint_common (package.xml:151) additionally runs cppcheck/uncrustify/xmllint/etc. on each test run.

Evidence detail: All EXPECT_* calls operate on std::is_* traits (lines 294-323); CMakeLists.txt 88-94 builds the target.

Estimated gain: One fewer test binary/link on the Jetson; faster colcon test

Fix sketch: Replace with static_assert lines at the bottom of motor.hpp (or a tiny .cpp compiled into jetank_motor_lib) and drop ament_cmake_gtest; keep lint_auto only if wanted.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-35 — README describes the hardware plugin as a non-existent stub and libgpiod as a dependency

`README.md:58` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: README.md:58 says the plugin has 'no C++ implementation yet' and CMake does not build it — both false (src/hardware/*, CMakeLists.txt:51-60). Line 3 and the node table mention libgpiod, which no source or manifest references. README also lists S5_joint in arm_controller (lines 35, 67) contrary to the YAML, and omits the odom parameters.

Evidence detail: grep -rn gpiod in package sources/CMake/package.xml returns nothing; plugin sources read this session.

Estimated gain: Documentation accuracy; prevents misdirected work

Fix sketch: Rewrite the plugin note, drop libgpiod mentions, fix joint lists, add publish_odom/odom_frame/base_frame/odom_rate to the params table.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-36 — Throttled WARN every 5 s forever while the robot is legitimately idle

`src/motor/robot_controller.cpp:206` · package jetank_motor_control · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: safetyCheck logs 'No cmd_vel received ... motors stopped' at 0.2 Hz for as long as the robot idles, generating rosout/DDS traffic and log noise on a headless Jetson for a non-event.

Evidence detail: Lines 204-210; the warn condition stays true indefinitely once idle.

Estimated gain: Quieter logs, less /rosout traffic

Fix sketch: Log once on the transition to stopped (pairs with finding 03).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-37 — readRegister lacks the fd guard the other I2C helpers have; setPWM ignores writeRegister results

`src/motor/motor.cpp:206` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: readRegister writes/reads on i2c_fd_ without the `i2c_fd_ < 0` check used by writeRegister/writeRegisters (inconsistent, calls write(-1)). setPWM's single-register fallback discards four bool results and setState ignores setPWM entirely, so an I2C failure is invisible to RobotController. Small, but it means the fallback path at lines 286-289 issues 4 more failing syscalls after a failed burst.

Evidence detail: Lines 206-217 vs 170-173; 286-289.

Estimated gain: Consistent error handling, fewer wasted syscalls on failure

Fix sketch: Add the guard; make setPWM/setState return bool and stop early on failure.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-08 — Two Motor instances each open /dev/i2c-7 and re-run the PCA9685 reset/prescale sequence

`src/motor/motor.cpp:90` · package jetank_motor_control · lens runtime · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: Both motors talk to the same PCA9685 at 0x60, but each Motor opens its own fd, issues resetPCA9685 (4 x usleep(1000) + 2 reads) and setPWMFreq (usleep(500)). The second reset re-initialises the chip after the first motor configured it (redundant ~10 ms startup, duplicate fd, and a potential output glitch on motor 0).

Evidence detail: Constructor lines 89-98; openI2C at 148-166; no shared-device abstraction exists.

Estimated gain: 1 fd, 1 reset sequence; ~50 LOC simpler

Fix sketch: Introduce a small Pca9685 object (fd + reset + setPWM) owned by RobotController and shared by both Motor instances (shared_ptr or reference).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-21 — Nested std::map<string, map<string,double>> storage for 9 joints where flat vectors suffice

`include/jetank_motor_control/jetank_serial_hardware.hpp:90` · package jetank_motor_control · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: Two nested string-keyed maps hold all interface values only to obtain stable addresses. Since all hot-loop access is already via cached pointers, a std::vector reserved once (or std::deque) of {joint, iface, value} gives the same stability with less code (the any_of/find plumbing in on_init lines 214-248) and fewer allocations.

Evidence detail: Header lines 88-97; on_init lines 165-248.

Estimated gain: ~30 LOC simpler on_init, fewer heap nodes

Fix sketch: Store `std::vector<InterfaceSlot>` reserved to total interface count; keep index-based pointers in ServoJoint and mirror_pairs_.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_motor_control-26 — arm_controller / gripper_controller parameters duplicated between jetank_controllers.yaml and config/controllers/*.yaml

`config/jetank_controllers.yaml:41` · package jetank_motor_control · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: The arm_controller (lines 41-74) and gripper_controller (84-98) sections are verbatim copies of config/controllers/arm_controller.yaml and gripper_controller.yaml, which the standalone path (moveit_bringup.launch.py) loads via --param-file. Two sources of truth must be kept in sync by hand (acknowledged in the file comments).

Evidence detail: Diffed by reading both files this session; moveit_bringup.launch.py:92-117 loads both jetank_controllers.yaml and the per-controller files.

Estimated gain: ~50 LOC of duplicated config

Fix sketch: Have the Gazebo path also spawn with --param-file from config/controllers/ and strip the per-controller sections from jetank_controllers.yaml (keep only controller_manager + sim-only controllers).

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_detection

Coverage: 18 files read; 5 measurements run. Notes: All 18 files in the package (source, tests, launch, interfaces, build files, README, .gitignore, pytest cache nodeids) were read in full; no file was skipped. The package has no config YAML, xacro, JS or HTML files. .pytest_cache is an untracked, gitignored artefact (CACHEDIR.TAG/README/stepwise not read — non-source). No live node from this package existed, so every runtime finding is static; the ROS measurement tools were run only to confirm that absence. Cross-package launch/consumer files in jetank_ros_main, jetank_web_control, jetank_perception and jetank_manipulation were grepped (not fully read) solely to establish who consumes this package's interfaces and topics.

Measurements:
- mcp__ros2-mcp__get_node_list: 17 nodes live on ROS_DOMAIN_ID=42; no /sock_detector or /frame_capture node from jetank_detection is running, so no hz/bw/profile measurement was possible for this package
- mcp__ros2-mcp__get_topic_list: /detections/socks (vision_msgs/Detection2DArray) exists only as a subscriber endpoint from /web_control_node; /detect_socks action and /detections/socks/debug are absent — confirms no live publisher from this package
- nproc = 6 (host CPU count, basis for finding 07); rclpy MultiThreadedExecutor default num_threads = multiprocessing.cpu_count() confirmed in .pixi rclpy/executors.py:830-832
- grep across workspace src: no DetectSocks action client outside the package; SegmentSocks/SockCloud used only by jetank_perception (server) and jetank_manipulation (client); jetank_web_control subscribes only to the Detection2DArray topic, not the debug image
- ls of .pixi site-packages: ultralytics/torch not installed in the pixi env (backend cannot be exercised here)

Findings: 28 (high 0, medium 4, low 24).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_detection-01 | runtime | Action execute loop busy-polls _latest_image with 10 ms sleeps | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:363` | medium | static | ~100 wakeups/s -> ~30/s during a goal; zero CPU while waiting for a first frame | S | confirmed |
| jetank_detection-02 | runtime | Continuous mode runs inference on every camera frame with no rate limit | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:275` | medium | static | GPU/CPU duty from ~100% to ~20-30% at 5-10 Hz processing | S | confirmed |
| jetank_detection-04 | runtime | No warm-up inference after model load; first action goal pays CUDA/cuDNN init inside its timeout | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:72` | medium | static | First-goal latency drops by the CUDA init time (seconds); avoids spurious first-goal timeouts | S | confirmed |
| jetank_detection-05 | runtime | predict() called without imgsz/half/device; FP16 and fixed input size not exploited on Jetson | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:79` | medium | static | ~1.5-2x inference speed with half=True; further with smaller imgsz | S | confirmed |
| jetank_detection-03 | runtime | Backend with no model loaded is non-None, so every frame is converted and logs a warning at camera rate | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:146` | low (orig medium) | static | Removes 30 warn logs/s + 30 image conversions/s in the no-model state | S | confirmed |
| jetank_detection-06 | runtime | Three separate GPU->host transfers per inference (xyxy, conf, cls) | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:93` | low | static | 2 fewer cudaMemcpy/sync per frame (~0.1-0.3 ms) | S | **UNVERIFIED** |
| jetank_detection-07 | runtime | MultiThreadedExecutor uses cpu_count() (6) threads for one subscription + one action server | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:494` | low | static | 4 fewer threads; less GIL churn | S | **UNVERIFIED** |
| jetank_detection-08 | runtime | Image subscriptions use RELIABLE QoS for 30 Hz raw frames | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:204` | low | static | Lower DDS overhead for ~20-40 MB/s image stream | S | **UNVERIFIED** |
| jetank_detection-10 | runtime | Per-goal subscription create/destroy adds DDS matching latency to every on-demand goal | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:345` | low | static | ~50-300 ms lower goal latency; no discovery churn per goal | S | **UNVERIFIED** |
| jetank_detection-11 | runtime | _latest_image retains a full frame indefinitely after a goal finishes | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:258` | low | static | ~1 MB freed between goals; avoids one stale inference per goal | S | **UNVERIFIED** |
| jetank_detection-12 | runtime | capture_frames: depth-10 reliable queue and blocking JPEG write in the subscription callback | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/capture_frames.py:75` | low | static | ~7-14 MB less queued image memory; fewer deserialisations | S | **UNVERIFIED** |
| jetank_detection-13 | minimality | cv2 deferred import + _cv2 cache + RuntimeError path are dead: cv_bridge already hard-imports cv2 | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:138` | low | static | ~15 LOC in node + 5 LOC test removed | S | **UNVERIFIED** |
| jetank_detection-14 | minimality | Per-call deferred imports of DetectSocks and SetParametersResult with void rationale | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:316` | low | static | ~6 LOC; removes per-goal/per-param import overhead | S | **UNVERIFIED** |
| jetank_detection-15 | minimality | is_activated guards in continuous callback are always true | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:281` | low | static | 2 attribute checks/frame, ~3 LOC | S | **UNVERIFIED** |
| jetank_detection-16 | minimality | _image_lock is unnecessary for a single reference assignment | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:71` | low | static | ~5 LOC; one lock op per frame and per poll | S | **UNVERIFIED** |
| jetank_detection-17 | minimality | DetectorBackend ABC + make_backend factory with NotImplementedError branches for backends that do not exist | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:111` | low | static | ~45 LOC + 2 tests | S | **UNVERIFIED** |
| jetank_detection-18 | minimality | debug defaults to true although the only in-tree consumer draws overlays from Detection2DArray | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:97` | low | static | One fewer publisher + per-frame check in default runs | S | **UNVERIFIED** |
| jetank_detection-21 | minimality | detect.launch.py omits detections_topic/debug_image_topic args, forcing scoped SetParameter workarounds in three parent launches | `/home/koen/workspaces/ros2_ws/src/jetank_detection/launch/detect.launch.py:84` | low | static | ~20 LOC removed across parents; simpler launch graph | S | **UNVERIFIED** |
| jetank_detection-22 | minimality | 55 lines of import-stub scaffolding in test_logic.py for deps that are always present in the pixi env | `/home/koen/workspaces/ros2_ws/src/jetank_detection/test/test_logic.py:28` | low | static | ~60 LOC | S | **UNVERIFIED** |
| jetank_detection-23 | footprint | Redundant test_depends: ament_copyright/flake8/pep257 already come via ament_lint_common, and copyright is disabled | `/home/koen/workspaces/ros2_ws/src/jetank_detection/package.xml:27` | low | static | 3 lines; correct dependency declaration | S | **UNVERIFIED** |
| jetank_detection-24 | footprint | C++ warning flags added to a package with no C++ targets | `/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:4` | low | static | 3 LOC | S | **UNVERIFIED** |
| jetank_detection-25 | footprint | Node scripts installed twice (site-packages module + lib/ executable) | `/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:42` | low | static | ~650 duplicated installed lines removed; clearer entry points | S | **UNVERIFIED** |
| jetank_detection-26 | footprint | Explicit ament_cmake_python buildtool_depend/find_package is redundant with ament_cmake | `/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:10` | low | static | 2 LOC | S | **UNVERIFIED** |
| jetank_detection-27 | footprint | rclpy and cv_bridge declared as <depend> (build+exec) for a runtime-only Python use | `/home/koen/workspaces/ros2_ws/src/jetank_detection/package.xml:14` | low | static | Accurate dep graph; no build-order coupling | S | **UNVERIFIED** |
| jetank_detection-09 | runtime | Debug image published as raw bgr8 Image; full-frame copy per debug frame | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:293` | low | static | ~10-20x less debug bandwidth; one fewer full-frame copy if encoded directly | M | **UNVERIFIED** |
| jetank_detection-19 | minimality | Three-way model selection (sim/model_path/model_path_sim/model_path_real) duplicated across node, launch and wrappers | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:101` | low | static | ~35 LOC across node + launch; 3 fewer parameters | M | **UNVERIFIED** |
| jetank_detection-20 | minimality | detect_sim/detect_real wrappers duplicate 70 lines each to set two values | `/home/koen/workspaces/ros2_ws/src/jetank_detection/launch/detect_real.launch.py:51` | low | static | ~140 LOC could become ~20 if kept as one parametrised wrapper, or 0 if parents include detect.launch.py directly | M | **UNVERIFIED** |
| jetank_detection-28 | footprint | SegmentSocks/SockCloud interfaces live here but are implemented and consumed in jetank_perception and jetank_manipulation | `/home/koen/workspaces/ros2_ws/src/jetank_detection/action/SegmentSocks.action:1` | low | static | Removes jetank_perception -> jetank_detection build edge; -1 rosidl dependency here | M | **UNVERIFIED** |

### jetank_detection-01 — Action execute loop busy-polls _latest_image with 10 ms sleeps

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:363` · package jetank_detection · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: _execute_callback spins a while-loop that takes the lock, compares the header stamp to last_stamp and time.sleep(0.01)s when nothing new arrived (lines 363-380, 388, 395). At a 30 Hz camera that is ~3 wake-ups per frame, and 100 wake-ups/s for the whole `timeout` window when no image arrives at all (e.g. camera down), each holding the GIL on one of the executor threads. A threading.Condition notified from _store_latest would wake exactly once per frame and block otherwise.

Evidence detail: No live /sock_detector node (get_node_list showed none) so not measured. Reasoning from code: poll period 10 ms vs typical 33 ms frame period; loop body does lock + tuple compare + sleep on every miss.

Estimated gain: ~100 wakeups/s -> ~30/s during a goal; zero CPU while waiting for a first frame

Fix sketch: Replace _image_lock with threading.Condition; in _store_latest set self._latest_image and notify(); in the loop cond.wait(timeout=remaining) and re-check stamp. Drop the three duplicate time.sleep(0.01) branches.

Verifier (confirmed, adjusted medium): sock_detector_node.py:358-396 spins a while-loop that takes self._image_lock (a plain threading.Lock, line 71), reads self._latest_image, and time.sleep(0.01)s on every miss (lines 367, 373, 379, 388, 395). _store_latest at lines 255-258 only assigns under the lock with no Condition/notify, so nothing wakes the loop per frame; the 10 ms poll vs ~33 ms frame period and 100 wakeups/s while no image arrives are as described. Not measured live (no running node); static reasoning only. Cost is bounded to the goal's timeout window, so medium is fair but arguably low.

### jetank_detection-02 — Continuous mode runs inference on every camera frame with no rate limit

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:275` · package jetank_detection · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: _continuous_image_callback converts + infers on every message delivered by the depth-1 subscription. With a 30 Hz camera and ~30-40 ms/frame Stage-1 inference (README table) the GPU and one CPU thread are pegged at 100% while active, even though the consumer (web overlay, grasp pipeline) needs only a few Hz. On the Orin Nano this drives thermals and starves stereo/Nav2. No `max_rate`/`process_every_n` parameter exists.

Evidence detail: Camera publishes at 30 Hz per stereo_camera_node (left/image_raw); no throttle logic present in callback lines 260-298. Not measured: node not running.

Estimated gain: GPU/CPU duty from ~100% to ~20-30% at 5-10 Hz processing

Fix sketch: Declare `max_rate_hz` (default e.g. 5.0); in the callback compare msg stamp / monotonic time against last processed and return early. Keep _store_latest before the early-return so the action still sees fresh frames.

Verifier (confirmed, adjusted medium): sock_detector_node.py:203-204 subscribes with depth 1 and _continuous_image_callback (lines 260-298) has no throttle, so with camera.fps: 30 (jetank_perception/config/stereo_camera_config.yaml:12) and ~30-40 ms/frame Stage-1 inference (jetank_detection/README.md:24) the callback runs back-to-back. The fix is safe and in-scope: the only cross-package consumer is web_control_node.py:498/824-870, which just caches the latest Detection2DArray and treats it stale after DET_STALE_SEC = 1.0 s, so any max_rate_hz >= ~2 Hz keeps the overlay fresh; the action path (lines 343-346, 360-370) reads _latest_image via _store_latest, which the sketch keeps before the early return, and does its own stamp dedup. The change is a single additive parameter inside jetank_detection, touching no package boundaries or perception strategy/factory abstractions; topics.py only validates topic names, not rates.

### jetank_detection-04 — No warm-up inference after model load; first action goal pays CUDA/cuDNN init inside its timeout

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:72` · package jetank_detection · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: load() only constructs YOLO(model_path). The first predict() triggers CUDA context creation, kernel JIT and cuDNN autotune, typically 1-5 s on Jetson. That latency lands inside the first DetectSocks goal (default timeout 5 s) or the first continuous frame, instead of at on_configure where it is expected.

Evidence detail: load() has no dummy predict; on_configure calls load() then returns. Not measured (no ultralytics in the pixi env, no live node).

Estimated gain: First-goal latency drops by the CUDA init time (seconds); avoids spurious first-goal timeouts

Fix sketch: After YOLO(model_path) run self._model.predict(np.zeros((h,w,3),np.uint8), verbose=False) once in load() (h,w from an `imgsz` param).

Verifier (confirmed, adjusted medium): backends.py:60-72 load() only does `self._model = YOLO(model_path)`; grep across jetank_detection finds no warm-up/dummy predict, the only predict call is in infer() at backends.py:79. sock_detector_node.py:150 calls load() in on_configure and returns, so CUDA/cuDNN first-call init lands inside the first infer(), which the goal loop at sock_detector_node.py:326/349 runs against a 5 s default deadline (or the first continuous frame). The magnitude (1-5 s) is not measured in this session (no ultralytics in env, no live node), but the structural claim is accurate.

### jetank_detection-05 — predict() called without imgsz/half/device; FP16 and fixed input size not exploited on Jetson

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:79` · package jetank_detection · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: predict(image_bgr, conf=..., verbose=False) uses ultralytics defaults: FP32, imgsz=640, device auto-selected each call. On the Orin GPU `half=True` roughly halves inference time for YOLO11n; pinning `device` avoids the per-call selection, and exposing `imgsz` lets the user trade accuracy for speed (e.g. 416) without touching code.

Evidence detail: Argument list at backends.py:79-83 contains only conf and verbose. Not measured.

Estimated gain: ~1.5-2x inference speed with half=True; further with smaller imgsz

Fix sketch: Add imgsz/half/device kwargs to UltralyticsBackend.__init__ (from node params) and pass them to predict(); default half=True when device is cuda.

Verifier (confirmed, adjusted medium): backends.py:79-83 passes only conf and verbose to predict(), so ultralytics defaults (FP32, imgsz=640, per-call device selection) apply; the fix is in-scope since the only consumers are in-package: sock_detector_node.py:146 (make_backend("ultralytics"), no kwargs) and infer() calls at sock_detector_node.py:275/392, with no other package importing jetank_detection.backends and no perception strategy/factory code touched. One safety caveat: the test double at test/test_logic.py:140 defines predict(self, image, conf, verbose), so adding imgsz/half/device kwargs will TypeError that fake unless the test is updated in the same change, and half=True/smaller imgsz slightly alter detection scores, so defaults should preserve current behaviour (half only when device is cuda, imgsz default 640). Speed gain is static reasoning only, not measured.

### jetank_detection-03 — Backend with no model loaded is non-None, so every frame is converted and logs a warning at camera rate

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:146` · package jetank_detection · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: make_backend() always returns an UltralyticsBackend; if resolved_model is empty or load() fails (lines 148-160) self._backend stays a backend object with _model=None. The `if self._backend is None: return` guards (264, 378) are therefore dead in practice, and in continuous mode each frame still pays cv_bridge conversion (line 268) then infer() raises RuntimeError('Model not loaded') -> get_logger().warn per frame (line 277) at 30 Hz, spamming rosout and doing a full-frame conversion for nothing.

Evidence detail: Traced on_configure: backend created at 146 unconditionally; load failure caught at 152 without resetting _backend. infer() raises at backends.py:77 when _model is None.

Estimated gain: Removes 30 warn logs/s + 30 image conversions/s in the no-model state

Fix sketch: Set self._backend = None when no model resolves or load() raises (so the existing guards work), or add a `loaded` property checked before conversion; use get_logger().warn(..., throttle_duration_sec=5.0) for the remaining per-frame failure paths.

Verifier (confirmed, adjusted low): sock_detector_node.py:146 creates the backend unconditionally and lines 148-160 never reset self._backend on empty model or load() failure, so the guards at 264 and 378 are dead; backends.py infer() raises RuntimeError when _model is None, hitting the per-frame warn at 277 after a full imgmsg_to_cv2 at 268. The gain is real but only in a misconfigured state (no model) and only when continuous=true, which defaults to False (line 96) and is off in detect_real.launch.py; the action path (378) polls at ~100 Hz via 10 ms sleeps in that state, which is a second wasted-conversion loop. Since it costs nothing in the normal configured path and is a small fix (S), low-medium is the honest severity; I lean low because it requires a startup misconfiguration that already logs a warn/error at configure.

### jetank_detection-06 — Three separate GPU->host transfers per inference (xyxy, conf, cls)

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:93` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist() each synchronise and copy from the device. boxes.data is a single (N,6) tensor [x1,y1,x2,y2,conf,cls]; one .tolist() gives all three.

Evidence detail: Lines 93-95 issue three tensor conversions; ultralytics Boxes.data holds the concatenated tensor.

Estimated gain: 2 fewer cudaMemcpy/sync per frame (~0.1-0.3 ms)

Fix sketch: for x1,y1,x2,y2,score,cls_id in boxes.data.tolist(): ...

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-07 — MultiThreadedExecutor uses cpu_count() (6) threads for one subscription + one action server

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:494` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: MultiThreadedExecutor() with num_threads=None spawns multiprocessing.cpu_count() worker threads (rclpy executors.py:830-832) — 6 on the Orin Nano. The node needs at most 2 concurrent callbacks (the image callback and the action execute). Extra Python threads add GIL contention and idle stack memory.

Evidence detail: nproc = 6 on this host; rclpy default confirmed in .pixi rclpy/executors.py line 832.

Estimated gain: 4 fewer threads; less GIL churn

Fix sketch: MultiThreadedExecutor(num_threads=2).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-08 — Image subscriptions use RELIABLE QoS for 30 Hz raw frames

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:204` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: create_subscription(Image, topic, cb, 1) (also line 346) uses default RELIABLE reliability. For large Image samples DDS reliable delivery adds ACK/NACK traffic and retransmission of frames the depth-1 queue will drop anyway. A BEST_EFFORT (SensorDataQoS) subscriber is compatible with the RELIABLE publisher in stereo_camera_node and avoids that overhead.

Evidence detail: Publisher QoS verified: stereo_camera_node.cpp:703 create_publisher(..., 10) = reliable. Subscription QoS is the integer depth form at lines 204 and 346.

Estimated gain: Lower DDS overhead for ~20-40 MB/s image stream

Fix sketch: from rclpy.qos import qos_profile_sensor_data; pass it (or QoSProfile(depth=1, reliability=BEST_EFFORT)) in both create_subscription calls.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-10 — Per-goal subscription create/destroy adds DDS matching latency to every on-demand goal

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:345` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: In on-demand mode every DetectSocks goal creates a new subscription (345) and destroys it in finally (411). Subscription creation + DDS endpoint matching typically costs tens to hundreds of ms before the first sample arrives, and this is spent inside the goal's timeout budget each time.

Evidence detail: Code path lines 342-347 and 409-411; not measured (no live node).

Estimated gain: ~50-300 ms lower goal latency; no discovery churn per goal

Fix sketch: Create one persistent depth-1 subscription in on_activate regardless of mode (continuous only decides whether the callback also infers); accept the idle cost of one deserialisation per frame.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-11 — _latest_image retains a full frame indefinitely after a goal finishes

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:258` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: After an on-demand goal the temporary subscription is destroyed but self._latest_image keeps the last Image message (0.7-1.4 MB) alive until on_cleanup. A subsequent goal will also see this stale frame first and only skips it if the stamp matches last_stamp (which is reset per goal), so a stale frame can be inferred once per goal.

Evidence detail: _latest_image only cleared at on_cleanup (242); last_stamp initialised to None per goal (355) so the first loop iteration accepts whatever is stored.

Estimated gain: ~1 MB freed between goals; avoids one stale inference per goal

Fix sketch: In the finally block set self._latest_image = None when tmp_sub was used; or record the stored frame's stamp before subscribing and seed last_stamp with it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-12 — capture_frames: depth-10 reliable queue and blocking JPEG write in the subscription callback

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/capture_frames.py:75` · package jetank_detection · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The subscription uses depth 10 while only one frame per `interval_sec` is saved; up to 10 raw frames (~7-14 MB) can queue behind the blocking cv2.imwrite (line 114) on the executor thread. All 30 Hz frames are also deserialised into Python only to be discarded by the interval check (line 101). Depth 1 + best-effort halves the memory and drop-handling cost; `[int(cv2.IMWRITE_JPEG_QUALITY), q]` is rebuilt per call.

Evidence detail: create_subscription depth 10 at line 75; interval gate at 101; imwrite at 114-115.

Estimated gain: ~7-14 MB less queued image memory; fewer deserialisations

Fix sketch: Use qos_profile_sensor_data with depth 1; precompute self._jpeg_params once in __init__.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-13 — cv2 deferred import + _cv2 cache + RuntimeError path are dead: cv_bridge already hard-imports cv2

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:138` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 76, 138-143 and 471-473 exist so the module imports without cv2, but line 31 `from cv_bridge import CvBridge` (module top) imports cv2 unconditionally, so the ImportError branch can never be reached. The associated test TestDrawDetectionsGeometry.test_missing_cv2_raises_runtime_error (test_logic.py:303) tests an impossible state.

Evidence detail: cv_bridge/core.py imports cv2 at module scope; sock_detector_node.py:31 is an unconditional top-level import.

Estimated gain: ~15 LOC in node + 5 LOC test removed

Fix sketch: import cv2 at module top next to cv_bridge; delete self._cv2, the try/except and the None check; drop the test.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-14 — Per-call deferred imports of DetectSocks and SetParametersResult with void rationale

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:316` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: `from jetank_detection.action import DetectSocks` is repeated in on_configure (173) and inside _execute_callback (316, per goal), and `from rcl_interfaces.msg import SetParametersResult` runs inside every parameter-set callback (462). The stated reason ('keeps the module importable in a bare env') is void because rclpy, cv_bridge, sensor_msgs and vision_msgs are already imported at module top (30-37); the test file stubs those anyway. Each executes a sys.modules lookup + attribute fetch per call and duplicates code.

Evidence detail: Three deferred imports at 173, 316, 462 vs unconditional ROS imports at 30-37.

Estimated gain: ~6 LOC; removes per-goal/per-param import overhead

Fix sketch: Move both imports to module level.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-15 — is_activated guards in continuous callback are always true

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:281` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _continuous_sub is created only in on_activate (203) after super().on_activate activates the lifecycle publishers, and destroyed in on_deactivate (215) before super().on_deactivate. Inside the callback, _det_pub.is_activated and _debug_pub.is_activated (281, 289) therefore cannot be false; the checks are redundant per-frame work and lines.

Evidence detail: Lifecycle ordering at lines 196-206 and 214-219.

Estimated gain: 2 attribute checks/frame, ~3 LOC

Fix sketch: Keep only the `is not None` and get_subscription_count() checks.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-16 — _image_lock is unnecessary for a single reference assignment

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:71` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The lock only wraps `self._latest_image = msg` (257-258) and `current_msg = self._latest_image` (363-364). Attribute reference assignment/read is atomic under the GIL, so the Lock adds an acquire/release per frame and per poll iteration for no safety gain. (If finding 01 is applied, a Condition replaces it with actual purpose.)

Evidence detail: Only two lock sites, each guarding one attribute access.

Estimated gain: ~5 LOC; one lock op per frame and per poll

Fix sketch: Drop the lock, or convert to threading.Condition per finding 01.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-17 — DetectorBackend ABC + make_backend factory with NotImplementedError branches for backends that do not exist

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:111` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Only UltralyticsBackend exists; make_backend() is called once with the literal 'ultralytics' (sock_detector_node.py:146). The ABC (29-46), the factory with tensorrt/subprocess NotImplementedError and ValueError branches (111-140), and the two tests that exercise the error branches (test_import.py:38-54) are ~45 lines of speculative indirection. Detection.class_id (25) is populated but never read by the node (only det.label is used).

Evidence detail: grep shows the single make_backend call site; node uses det.label/score/cx/cy/w/h only (436-456, 469-487).

Estimated gain: ~45 LOC + 2 tests

Fix sketch: Instantiate UltralyticsBackend() directly; drop ABC/factory until a second backend exists (a `backend` string param can be added then). Remove class_id or use it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-18 — debug defaults to true although the only in-tree consumer draws overlays from Detection2DArray

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:97` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: `debug` defaults True (97, launch 64-68), creating the Image lifecycle publisher and running the subscriber-count check on every frame. jetank_web_control consumes /detections/socks (Detection2DArray) and draws boxes client-side; nothing in the workspace subscribes to /detections/socks/debug. A false default matches actual use and removes the per-frame check and publisher from the normal path.

Evidence detail: grep of workspace src: only web_control_node.py:498 subscribes, to the detections topic; debug topic has no subscriber in code.

Estimated gain: One fewer publisher + per-frame check in default runs

Fix sketch: Default debug to False in node and detect.launch.py; README table update.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-21 — detect.launch.py omits detections_topic/debug_image_topic args, forcing scoped SetParameter workarounds in three parent launches

`/home/koen/workspaces/ros2_ws/src/jetank_detection/launch/detect.launch.py:84` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The parameters block forwards 9 params but not detections_topic/debug_image_topic (declared by the node at 98-99). jetank_ros_main sim_demo.launch.py:191-192, mobile_grasp.launch.py:89-90 and mobile_grasp_hw.launch.py:118-119 each wrap the include in a GroupAction with two SetParameter actions plus explanatory comments to compensate. Declaring the two args here removes ~6 lines x 3 files of indirection and one GroupAction per parent.

Evidence detail: Parent launch grep shows the SetParameter pattern with comments citing 'detect_sim.launch.py declares no launch args for them'.

Estimated gain: ~20 LOC removed across parents; simpler launch graph

Fix sketch: Add DeclareLaunchArgument for detections_topic and debug_image_topic in detect.launch.py and forward them; parents pass them as launch_arguments.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-22 — 55 lines of import-stub scaffolding in test_logic.py for deps that are always present in the pixi env

`/home/koen/workspaces/ros2_ws/src/jetank_detection/test/test_logic.py:28` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _install_stubs (28-89) fabricates rclpy/cv_bridge/cv2/sensor_msgs/vision_msgs modules when missing. The tests are run via ament_add_pytest_test inside colcon (CMakeLists.txt:70-71) or `pixi run` (README:317), where all of these are installed, so the stub branches never execute. It is dead code that also masks real import errors.

Evidence detail: CMake registers the tests under colcon; workspace is pixi/RoboStack managed (CLAUDE.md), so rclpy etc. are always importable.

Estimated gain: ~60 LOC

Fix sketch: Delete _install_stubs and the sys.path insertion; import the modules directly (pytest.importorskip if a bare-env run is still desired).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-23 — Redundant test_depends: ament_copyright/flake8/pep257 already come via ament_lint_common, and copyright is disabled

`/home/koen/workspaces/ros2_ws/src/jetank_detection/package.xml:27` · package jetank_detection · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_lint_common (line 31) already depends on ament_cmake_copyright, ament_cmake_flake8, ament_cmake_pep257 etc.; the explicit ament_copyright/ament_flake8/ament_pep257 entries (27-29) are redundant, and ament_copyright is explicitly disabled in CMakeLists.txt:63. Conversely ament_cmake_pytest is find_package'd (CMakeLists.txt:69) without a matching test_depend, relying on ament_cmake's transitive export.

Evidence detail: package.xml lines 27-32 vs CMakeLists.txt 60-71.

Estimated gain: 3 lines; correct dependency declaration

Fix sketch: Remove ament_copyright/ament_flake8/ament_pep257 test_depends; add <test_depend>ament_cmake_pytest</test_depend>.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-24 — C++ warning flags added to a package with no C++ targets

`/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:4` · package jetank_detection · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: add_compile_options(-Wall -Wextra -Wpedantic) (4-6) is the ament C++ template boilerplate; this package compiles only rosidl-generated code (whose flags are set by rosidl) and installs Python. The block is dead and misleading about the package type.

Evidence detail: No add_executable/add_library in CMakeLists.txt; only rosidl_generate_interfaces and install rules.

Estimated gain: 3 LOC

Fix sketch: Delete lines 4-6.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-25 — Node scripts installed twice (site-packages module + lib/ executable)

`/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:42` · package jetank_detection · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: install(DIRECTORY jetank_detection/ ...) (34-39) copies sock_detector_node.py and capture_frames.py into site-packages, and install(PROGRAMS ...) (42-53) copies the same two files again into lib/jetank_detection. Two copies of each ~500/140-line file in the install tree; a change requires both to be consistent. A 3-line launcher in scripts/ (or excluding the node files from the module install) avoids the duplication.

Evidence detail: Two install rules reference the same source files.

Estimated gain: ~650 duplicated installed lines removed; clearer entry points

Fix sketch: Add scripts/sock_detector_node and scripts/capture_frames stubs that `from jetank_detection.sock_detector_node import main; main()` and install those with PROGRAMS; or add PATTERN excludes to the DIRECTORY install.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-30** — same double install of the node scripts; footprint-30 additionally asks to declare python3-opencv/python3-numpy and to put ultralytics under pixi [pypi-dependencies] (see footprint-09).

### jetank_detection-26 — Explicit ament_cmake_python buildtool_depend/find_package is redundant with ament_cmake

`/home/koen/workspaces/ros2_ws/src/jetank_detection/CMakeLists.txt:10` · package jetank_detection · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_cmake's package exports ament_cmake_python (and ament_cmake_pytest), so find_package(ament_cmake REQUIRED) already provides ament_get_python_install_dir(). The separate find_package(ament_cmake_python) (10) and <buildtool_depend>ament_cmake_python</buildtool_depend> (package.xml:11) add a redundant lookup and line each.

Evidence detail: ament_cmake metapackage exports ament_cmake_python as buildtool_export_depend; only ament_get_python_install_dir is used (line 33).

Estimated gain: 2 LOC

Fix sketch: Remove CMakeLists.txt:10 and package.xml:11 (verify configure still finds ament_get_python_install_dir).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-27 — rclpy and cv_bridge declared as <depend> (build+exec) for a runtime-only Python use

`/home/koen/workspaces/ros2_ws/src/jetank_detection/package.xml:14` · package jetank_detection · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: rclpy (14) and cv_bridge (17) are only imported at runtime by Python; nothing at build time needs them (rosidl DEPENDENCIES are vision/sensor/geometry_msgs). <depend> forces colcon to order this package after them at build time and advertises a build dependency that does not exist. exec_depend is the accurate, lighter declaration.

Evidence detail: CMakeLists.txt has no find_package for rclpy/cv_bridge; imports only in the .py files.

Estimated gain: Accurate dep graph; no build-order coupling

Fix sketch: Change both to <exec_depend>.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-09 — Debug image published as raw bgr8 Image; full-frame copy per debug frame

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:293` · package jetank_detection · lens runtime · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: When a debug subscriber exists, _draw_detections does image_bgr.copy() (line 475, ~0.7 MB at 640x360, 1.4 MB at 640x480) and cv2_to_imgmsg serialises the raw frame (another copy) at up to 30 Hz — 20-40 MB/s on the DDS bus. The web UI draws its own overlay from Detection2DArray (web_control_node.py:498), so the debug image is only for RViz/rqt. Publishing as CompressedImage (JPEG) would cut the payload ~10-20x; note in-place drawing is not safe because image_bgr may alias the message stored in _latest_image.

Evidence detail: copy at 475; cv2_to_imgmsg at 294; web_control subscribes to Detection2DArray not the debug image (grep of web_control_node.py).

Estimated gain: ~10-20x less debug bandwidth; one fewer full-frame copy if encoded directly

Fix sketch: Publish sensor_msgs/CompressedImage with cv2.imencode('.jpg', img, [IMWRITE_JPEG_QUALITY, 80]); or default `debug` to false (finding 18).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-19 — Three-way model selection (sim/model_path/model_path_sim/model_path_real) duplicated across node, launch and wrappers

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:101` · package jetank_detection · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: The node declares four parameters and ~25 lines of selection logic (89-123) so that the launch layer can pass both model paths and a flag. The launch wrappers (detect_sim/detect_real) already pin `sim` and expose only one model path each, and every parent launch (sim_demo, mobile_grasp, mobile_grasp_hw) passes exactly one path. A single `model_path` parameter resolved in the wrappers achieves the same with 3 fewer params, ~25 fewer node lines and 3 fewer launch args.

Evidence detail: Parents pass model_path_sim or model_path_real only (jetank_ros_main launch grep); the node never needs both at once.

Estimated gain: ~35 LOC across node + launch; 3 fewer parameters

Fix sketch: Node keeps only model_path; detect_sim.launch.py maps its model_path_sim arg to model_path (same for real). Keep the SIM/REAL log line from the wrapper's `sim` arg if desired.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-20 — detect_sim/detect_real wrappers duplicate 70 lines each to set two values

`/home/koen/workspaces/ros2_ws/src/jetank_detection/launch/detect_real.launch.py:51` · package jetank_detection · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: Each wrapper (detect_real.launch.py, detect_sim.launch.py) is ~70 lines whose effect is `sim:=<bool>` plus a different default for `continuous` and one model-path arg. The parents in jetank_ros_main already pass continuous explicitly (mobile_grasp.launch.py:93 `continuous="true"`), so the wrapper defaults are overridden anyway. Both files share identical docstring/import/include boilerplate.

Evidence detail: Both wrappers read in full; mobile_grasp.launch.py:93 passes continuous explicitly.

Estimated gain: ~140 LOC could become ~20 if kept as one parametrised wrapper, or 0 if parents include detect.launch.py directly

Fix sketch: Have sim_demo/mobile_grasp include detect.launch.py with sim/continuous/model_path set directly; or keep a single wrapper with a `sim` arg using launch conditions for the continuous default.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_detection-28 — SegmentSocks/SockCloud interfaces live here but are implemented and consumed in jetank_perception and jetank_manipulation

`/home/koen/workspaces/ros2_ws/src/jetank_detection/action/SegmentSocks.action:1` · package jetank_detection · lens footprint · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: SegmentSocks.action and SockCloud.msg are not used by any code in this package; the server is jetank_perception/src/sock_segmentation_server.cpp and the client is jetank_manipulation. Hosting them here makes the C++ jetank_perception package (CMakeLists.txt:18) and jetank_manipulation depend on jetank_detection, so any change to this Python package's interfaces triggers rebuilds of the heavier perception C++ tree, and pulls geometry_msgs into this package's rosidl DEPENDENCIES only for these types. Reported for completeness — moving them (e.g. to jetank_perception, which owns the server) is a cross-package decision.

Evidence detail: grep: SegmentSocks/SockCloud referenced only in jetank_perception and jetank_manipulation outside this package's build files; no Python here imports them.

Estimated gain: Removes jetank_perception -> jetank_detection build edge; -1 rosidl dependency here

Fix sketch: Move SegmentSocks.action + SockCloud.msg to the package that implements the server (or a small interfaces package) and update the two consumers' imports; drop geometry_msgs from this package if nothing else needs it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_navigation

Coverage: 24 files read; 13 measurements run. Notes: All source, build, config, launch, rviz and script files in the package were read in full. Documentation files were only skimmed for stale references: README.md (lines 1-60 of 256), QUICKSTART.md (1-40 of 166), PROGRESS.md (full); TROUBLESHOOTING.md (642 lines), LEARNING_PLAN.md (882 lines) and LICENSE were not read because they are not installed and contain no code/config. maps/ directory exists but is empty. Runtime measurements were only possible for /icm20948_imu; rplidar_node, slam_toolbox and the Nav2 stack were not running, so findings 14-18 and 28 are static tuning claims. profile_node required the leading-slash node name to succeed.

Measurements:
- mcp__ros2-mcp__get_node_list: /icm20948_imu live; rplidar_node absent (no /dev/ttyUSB0); no slam_toolbox or nav2 nodes running
- mcp__ros2-mcp__get_node_info(/icm20948_imu): publishers /imu/data_raw, /imu/magnetic_field, /imu/temperature (+ /rosout, /parameter_events)
- mcp__ros2-mcp__get_node_params(/icm20948_imu): frame_id=imu_link, i2c_address=104, i2c_bus=1, publish_rate=100.0; accel_range/gyro_range NOT declared
- mcp__ros2-mcp__profile_node(icm20948_imu) without leading slash: FAILED 'Process 27970 not found'
- mcp__ros2-mcp__profile_node(/icm20948_imu, 10 s): cpu mean 3.03% p95 9.8% peak 19.6%; RSS 27.55 MB; 11 threads; 17 fds; ctx switches vol 3074 / invol 1156 over window
- /proc/8240/stat 5 s sample: 15 ticks -> 3.00% CPU; VmRSS 26908 kB; Threads 11; 17 fds
- mcp__ros2-mcp__measure_topic_perf(/imu/data_raw, 10 s): 1000 msgs, 99.962 Hz, 68231 B/s, jitter 4.554 ms, drop_estimate 2, latency p50 4.91 ms / p95 13.95 ms / p99 23.05 ms (header.stamp)
- mcp__ros2-mcp__get_topic_bw(/imu/data_raw, 5 s): 68643.6 B/s
- mcp__ros2-mcp__get_topic_hz(/imu/magnetic_field, 5 s): 100.068 Hz
- mcp__ros2-mcp__get_topic_hz(/imu/temperature, 5 s): 10.0 Hz
- mcp__ros2-mcp__read_topic(/imu/magnetic_field, 3 msgs): 10 ms spacing, covariance all zeros
- ros2 topic info -v (Bash, read-only): all three IMU topics RELIABLE/VOLATILE, Subscription count 0
- Not run: read_diagnostics (node publishes no /diagnostics); nav2/slam_toolbox/rplidar measurements impossible (nodes not running)

Findings: 28 (high 0, medium 3, low 25).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_navigation-01 | runtime | IMU node reads I2C and publishes 3 topics at 100 Hz with zero subscribers (no lazy publishing) | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:260` | medium | measured | ~3% of one core and ~70 KB/s DDS traffic while no consumer exists; ~0.4 MB/s RSS untouched | S | confirmed |
| jetank_navigation-16 | runtime | DWB samples 800 trajectories per 20 Hz control cycle for a 0.3 m/s robot | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:150` | medium | static | ~4x fewer trajectory evaluations (800 -> 200) per control cycle | S | confirmed |
| jetank_navigation-17 | runtime | Nav2 lifecycle set launches waypoint_follower, velocity_smoother and (SLAM variant) smoother_server that no workflow uses | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/nav2_bringup.launch.py:63` | medium | static | 2-3 fewer processes (~100-150 MB RSS, dozens of threads), one fewer /cmd_vel hop | M | confirmed |
| jetank_navigation-02 | runtime | Sensor publishers use RELIABLE default QoS instead of SensorDataQoS | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:96` | low | measured | Fewer RTPS control messages per matched reader; lower latency under load | S | **UNVERIFIED** |
| jetank_navigation-03 | runtime | Magnetometer republished at 100 Hz although the I2C-master shadow updates at ~69 Hz and DRDY is never checked | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:344` | low | measured | ~30% fewer MagneticField messages; no stale duplicates | S | **UNVERIFIED** |
| jetank_navigation-10 | minimality | icm20948.yaml carries accel_range/gyro_range that the node never declares; bus comments contradict the value | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/icm20948.yaml:21` | low | measured | -2 dead config keys; removes doc drift | S | **UNVERIFIED** |
| jetank_navigation-04 | runtime | Per-tick heap allocation from exception construction on I2C read failure path | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:165` | low | static | Removes 100 heap allocs/s + stack unwinds in the fault case | S | **UNVERIFIED** |
| jetank_navigation-05 | runtime | Messages published by const-ref (copy into middleware) instead of unique_ptr / loaned | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:339` | low | static | Removes ~210 message copies/s (each incl. a std::string copy of frame_id) | S | **UNVERIFIED** |
| jetank_navigation-06 | minimality | Redundant fill(0.0) on freshly default-constructed covariance arrays | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:329` | low | static | -3 LOC, ~27 stores/tick | S | **UNVERIFIED** |
| jetank_navigation-07 | minimality | Temperature raw value parsed every tick but only consumed every 10th tick | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:298` | low | static | Negligible CPU; clearer hot path | S | **UNVERIFIED** |
| jetank_navigation-08 | minimality | Covariance constants computed at static-init with std::pow instead of constexpr | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:59` | low | static | Two fewer dynamic initializers; constant folding in publish_imu | S | **UNVERIFIED** |
| jetank_navigation-09 | minimality | Error messages print register in decimal after a '0x' prefix | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:140` | low | static | Correct diagnostics; fewer temporaries | S | **UNVERIFIED** |
| jetank_navigation-11 | minimality | slam_nav2.launch.py and navigation_only.launch.py are self-declared LEGACY and have no code caller | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/slam_nav2.launch.py:5` | low (orig medium) | static | -121 LOC launch, -~12 LOC branch in nav2_bringup.launch.py, one fewer installed entry-point | S | confirmed |
| jetank_navigation-12 | minimality | No-op remapping ('/scan' -> '/scan') in slam.launch.py | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/slam.launch.py:44` | low | static | -1 LOC, one fewer ros-args token | S | **UNVERIFIED** |
| jetank_navigation-13 | minimality | use_sim_time: False repeated in 14 nav2_params.yaml sections that RewrittenYaml overrides anyway | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:9` | low | static | -26 LOC of dead YAML | S | **UNVERIFIED** |
| jetank_navigation-14 | runtime | Local costmap uses VoxelLayer with publish_voxel_map for a 2D-lidar-only robot | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:195` | low (orig medium) | static | Lower per-scan costmap CPU at 5 Hz; removes a 5 Hz VoxelGrid publish; -3 LOC dead config | S | confirmed |
| jetank_navigation-15 | runtime | always_send_full_costmap: True on both costmaps forces full-grid publishes every cycle | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:218` | low | static | Reduces costmap publish bytes by the ratio of changed/total cells per cycle | S | **UNVERIFIED** |
| jetank_navigation-18 | runtime | slam_toolbox enable_interactive_mode: true publishes interactive markers for every graph node | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/slam/slam_toolbox.yaml:43` | low | static | Removes marker server + per-vertex marker traffic; halves map->odom TF publish rate | S | **UNVERIFIED** |
| jetank_navigation-19 | runtime | RViz config subscribes to /stereo_camera/points (dropped pipeline) and renders all TF frames | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/rviz/navigation.rviz:38` | low | static | Avoids a dense PointCloud2 subscription + render in RViz; fewer TF frames drawn | S | **UNVERIFIED** |
| jetank_navigation-20 | footprint | Nine nav2_* exec_depends are redundant with nav2_bringup; actually-imported packages are missing | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/package.xml:16` | low | static | -9 LOC redundant deps; accurate rosdep set | S | **UNVERIFIED** |
| jetank_navigation-21 | footprint | install(DIRECTORY maps/ ... OPTIONAL) installs an empty, untracked directory | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:39` | low | static | -5 LOC CMake; removes an empty share/jetank_navigation/maps install | S | **UNVERIFIED** |
| jetank_navigation-22 | footprint | BUILD_TESTING pulls ament_lint_common (cpplint/uncrustify/xmllint/flake8...) for one .cpp with copyright+cpplint already disabled | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:57` | low | static | Faster colcon test; -2 test_depends | S | **UNVERIFIED** |
| jetank_navigation-23 | footprint | No default CMAKE_BUILD_TYPE: ad-hoc colcon builds of the IMU node are unoptimized | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:4` | low | static | Consistent -O2/-O3 binary regardless of build entry point | S | **UNVERIFIED** |
| jetank_navigation-24 | minimality | nav2_bringup.launch.py carries upstream-only arguments (namespace, use_respawn, log_level) no caller sets | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/nav2_bringup.launch.py:141` | low | static | ~25 LOC | S | **UNVERIFIED** |
| jetank_navigation-25 | minimality | RViz launched through nav2_bringup's rviz_launch.py wrapper instead of a direct Node | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/navigation_full.launch.py:122` | low | static | ~8 LOC and one fewer nested launch include at startup | S | **UNVERIFIED** |
| jetank_navigation-26 | minimality | Dead/duplicated DWB and AMCL keys in nav2_params.yaml | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:157` | low | static | -5 LOC | S | **UNVERIFIED** |
| jetank_navigation-27 | minimality | Stray .claude/settings.local.json and empty first line in .gitignore inside the package | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/.claude/settings.local.json:1` | low | static | -1 stray file | S | **UNVERIFIED** |
| jetank_navigation-28 | runtime | AMCL particle count and update thresholds set high for a small indoor robot | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:44` | low | static | ~2x fewer particle updates per metre travelled | S | **UNVERIFIED** |

### jetank_navigation-01 — IMU node reads I2C and publishes 3 topics at 100 Hz with zero subscribers (no lazy publishing)

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:260` · package jetank_navigation · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: publish_imu() runs unconditionally from a 100 Hz wall timer: one blocking I2C_RDWR ioctl plus three publishes per tick. Nothing in the workspace subscribes to imu/data_raw, imu/magnetic_field or imu/temperature (grep found only launch/URDF references; live subscription count is 0 on all three). The node therefore burns ~3% of a core continuously and pushes ~68 KB/s into DDS for nobody. Checking get_subscription_count() on the publishers (or a subscriber-count-based timer gate) drops this to ~0 while idle, which matters on a Jetson Orin Nano shared with stereo perception and SLAM.

Evidence detail: profile_node(/icm20948_imu, 10 s): cpu mean 3.03%, p95 9.8%, peak 19.6%, RSS 27.5 MB, 11 threads, 17 fds; /proc 5 s sample: 15 ticks = 3.00% CPU. measure_topic_perf(/imu/data_raw): 99.96 Hz, 68231 B/s, jitter 4.55 ms, p99 latency 23 ms. `ros2 topic info -v`: Subscription count 0 on /imu/data_raw, /imu/magnetic_field, /imu/temperature.

Estimated gain: ~3% of one core and ~70 KB/s DDS traffic while no consumer exists; ~0.4 MB/s RSS untouched

Fix sketch: In publish_imu(): `if (imu_pub_->get_subscription_count()==0 && mag_pub_->get_subscription_count()==0 && temp_pub_->get_subscription_count()==0) return;` before read_bytes(). Optionally cancel/recreate the timer on match events.

Verifier (confirmed, adjusted medium): icm20948_node.cpp:102-104 creates an unconditional wall timer calling publish_imu(), and publish_imu() (line 260-285) does the I2C read_bytes() and then publishes with no get_subscription_count() check anywhere in the file (grep: zero hits). Workspace grep for imu/data_raw, imu/magnetic_field, imu/temperature finds only the publisher itself, launch docstrings (imu.launch.py:9-17) and a unified.launch.py banner string; no subscriber, EKF/robot_localization or Madgwick config exists. Live check this session: `ros2 topic info -v` shows Subscription count 0 on all three topics while get_topic_hz measured /imu/data_raw at 100.1 Hz (503 msgs/5 s). Severity stays medium: real but small (~3% of a core) and the gate is a one-line fix.

### jetank_navigation-16 — DWB samples 800 trajectories per 20 Hz control cycle for a 0.3 m/s robot

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:150` · package jetank_navigation · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: vx_samples: 20 × vth_samples: 40 = 800 candidate trajectories, each simulated for sim_time 1.5 s at linear_granularity 0.05 (≈9 poses at 0.3 m/s) and scored by 7 critics, at controller_frequency 20 Hz (line 111). That is ~16k trajectory evaluations per second; DWB is typically the single largest Nav2 CPU consumer on Jetson-class hardware. The velocity envelope here is tiny (0..0.3 m/s, ±1 rad/s) so 10×20 samples gives the same resolution per m/s that upstream defaults assume for faster robots.

Evidence detail: Lines 138-155; 20*40 = 800 samples at 20 Hz. Nav2 not running, so not measured.

Estimated gain: ~4x fewer trajectory evaluations (800 -> 200) per control cycle

Fix sketch: vx_samples: 10, vth_samples: 20 (retune critics if oscillation); optionally controller_frequency 10.0 for a 0.3 m/s base.

Verifier (confirmed, adjusted medium): nav2_params.yaml:150/152 set vx_samples: 20 and vth_samples: 40 (800 candidates) with sim_time 1.5 (line 153) and controller_frequency 20.0 (line 111) for a 0.0-0.3 m/s, +/-1 rad/s envelope (lines 136-140); the file is the default params in all three launch files (nav2_bringup.launch.py:153, navigation_only.launch.py:30, slam_nav2.launch.py:40), so it is live config. Mitigating detail the finding omits: short_circuit_trajectory_evaluation: True (line 159) lets critics abort rejected trajectories early, so per-cycle cost is below the naive 800 x 7-critic figure, and angular_granularity 0.025 (line 155) means rotating trajectories have up to ~60 poses, not 9. Not measured (Nav2 not running); the static inefficiency exists as described.

### jetank_navigation-17 — Nav2 lifecycle set launches waypoint_follower, velocity_smoother and (SLAM variant) smoother_server that no workflow uses

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/nav2_bringup.launch.py:63` · package jetank_navigation · lens runtime · severity medium · evidence static · effort M · verdict confirmed

Description: Every Nav2 server is a separate process with its own DDS participant, executor threads and ~30-60 MB RSS on aarch64. waypoint_follower (action server never called by RViz GoalTool or web control), velocity_smoother (duplicates DWB's own acc_lim_* limiting, adds one hop of latency on /cmd_vel at 20 Hz) and smoother_server (SLAM variant only, has no `smoother_server:` section in nav2_params.yaml so it runs on defaults and is never invoked by the default BT) are launched in both modes. Trimming lifecycle_nodes to the servers the default NavigateToPose tree actually needs saves 2-3 processes on the Jetson.

Evidence detail: lifecycle_nodes lists at lines 63-81; no `smoother_server` key in nav2_params.yaml (grep); velocity_smoother remaps cmd_vel_nav -> cmd_vel (line 100), DWB acc_lim_x/theta already set at 144-146. RViz/web control only issue NavigateToPose (rviz GoalTool, web_control_node).

Estimated gain: 2-3 fewer processes (~100-150 MB RSS, dozens of threads), one fewer /cmd_vel hop

Fix sketch: Remove 'waypoint_follower' and 'smoother_server' from both lists; drop velocity_smoother and remap controller_server cmd_vel directly to /cmd_vel (delete the velocity_smoother YAML block 307-320 and waypoint_follower block 296-305).

Verifier (confirmed, adjusted medium): The fix is in-scope and safe: no code in the workspace sends FollowWaypoints/NavigateThroughPoses/SmoothPath actions (workspace grep finds only launch/docs hits), bt_navigator uses the stock Humble NavigateToPose tree with no custom BT XML (nav2_params.yaml:7-27, no smoother_server section), so removing waypoint_follower and smoother_server from nav2_bringup.launch.py:63-81 changes nothing consumers observe. The only cross-package references are the pkill pattern list in jetank_web_control/web_control_node.py:1016 (harmlessly matches nothing) and cmd_vel_bridge.py:23 / docs, which only care that Nav2 publishes on /cmd_vel — satisfied by remapping controller_server directly (line 91). Dropping velocity_smoother's velocity_timeout zero-publish is covered by diff_drive_controller's own cmd_vel_timeout: 0.5 (jetank_motor_control/config/jetank_controllers.yaml:154); the one residual behavior change is losing OPEN_LOOP accel smoothing on top of DWB's acc_lim_* (nav2_params.yaml:144-149), which is a duplicate limiter, plus stale README/PROGRESS text that should be updated alongside.

### jetank_navigation-02 — Sensor publishers use RELIABLE default QoS instead of SensorDataQoS

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:96` · package jetank_navigation · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: create_publisher<...>("imu/data_raw", 10) uses the default reliable/volatile/keep-last-10 profile. For 100 Hz sensor streams this adds heartbeat/ACK-NACK traffic and history retention per matched reader, and forces subscribers (imu_filter, robot_localization) to negotiate reliable. Standard practice is rclcpp::SensorDataQoS() (best-effort, depth 5), which is lighter on the RMW layer and on the Jetson network stack.

Evidence detail: `ros2 topic info -v /imu/data_raw` reports Reliability: RELIABLE, Durability: VOLATILE; same for /imu/magnetic_field and /imu/temperature. Lines 96-98 pass a bare depth of 10.

Estimated gain: Fewer RTPS control messages per matched reader; lower latency under load

Fix sketch: Replace `10` with `rclcpp::SensorDataQoS()` on lines 96-98.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-03 — Magnetometer republished at 100 Hz although the I2C-master shadow updates at ~69 Hz and DRDY is never checked

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:344` · package jetank_navigation · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: I2C_MST_ODR_CONFIG is set to 0x04 (line 238), giving an external-sensor ODR of 1.1 kHz / 2^4 ≈ 69 Hz, and the AK09916 is in continuous mode 4 (100 Hz). The node reads ST1 (mag_raw[0]) into the burst but never tests its DRDY bit; it publishes a MagneticField every 100 Hz tick, so roughly 1 in 3 messages is a duplicate of the previous shadow contents. That is wasted serialization and DDS traffic, and downstream filters see stale samples with fresh timestamps.

Evidence detail: get_topic_hz(/imu/magnetic_field): 100.07 Hz (501 msgs / 5 s). read_topic shows consecutive samples at 10 ms spacing. Static: line 238 writes 0x04 to I2C_MST_ODR_CONFIG; line 302 reads mag_raw but line 303-308 skip byte 0 (ST1) without a DRDY test.

Estimated gain: ~30% fewer MagneticField messages; no stale duplicates

Fix sketch: `if (mag_raw[0] & 0x01) { ...publish mag... }` around lines 344-355, or decimate to every other tick.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-10 — icm20948.yaml carries accel_range/gyro_range that the node never declares; bus comments contradict the value

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/icm20948.yaml:21` · package jetank_navigation · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: accel_range and gyro_range are not declared in ICM20948Node (only i2c_bus, i2c_address, frame_id, publish_rate at node lines 73-76), so rclcpp silently drops them; the ranges are hard-coded to ±2g/±250°/s at node lines 225-229 and in the scale constants. The file header (line 3) and imu.launch.py docstring (lines 5-6) say /dev/i2c-7 while the value on line 17 is bus 1 and the node default (line 73) is 7. Over-general config plus three inconsistent statements of the same fact.

Evidence detail: get_node_params(/icm20948_imu) lists only frame_id, i2c_address, i2c_bus, publish_rate (+ builtin qos_overrides/use_sim_time); no accel_range/gyro_range. grep found no other reader of those keys.

Estimated gain: -2 dead config keys; removes doc drift

Fix sketch: Delete lines 20-22 (or declare/implement the range params); fix header comment line 3 and imu.launch.py lines 5-6 to bus 1; change node default at line 73 to 1.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-04 — Per-tick heap allocation from exception construction on I2C read failure path

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:165` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: read_bytes() reports failure by throwing std::runtime_error built from std::string concatenation + std::to_string + strerror. In publish_imu() this is caught every tick (lines 278-285). If the IMU is unplugged or the bus glitches, the node allocates and unwinds 100 times per second. The throttle only suppresses the log, not the exception cost. A bool/errno return from read_bytes with a throttled warn avoids the allocation and unwinding entirely on the hot path.

Evidence detail: Lines 165-168 throw on ioctl failure; lines 278-285 catch per tick in the 100 Hz callback. Not exercised during measurement (IMU was healthy).

Estimated gain: Removes 100 heap allocs/s + stack unwinds in the fault case

Fix sketch: Make read_bytes return bool (keep throwing variant for init only), and in publish_imu `if (!read_bytes(...)) { RCLCPP_WARN_THROTTLE(..., strerror(errno)); return; }`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-05 — Messages published by const-ref (copy into middleware) instead of unique_ptr / loaned

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:339` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: imu_msg, mag_msg and temp_msg are stack objects published via publish(const T&). rclcpp then copies the message (and for intra-process, allocates) on every call. Publishing std::unique_ptr lets rclcpp move the message and avoids the copy; for the Imu message (~330 bytes with three 9-double covariances + frame_id string) this is small but happens 200-210 times per second.

Evidence detail: Lines 313/339, 344/355, 366/372 use stack messages with publish(msg). No intra-process comms configured, so the cost is one struct copy + string copy per publish.

Estimated gain: Removes ~210 message copies/s (each incl. a std::string copy of frame_id)

Fix sketch: `auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>(); ... imu_pub_->publish(std::move(imu_msg));`

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-06 — Redundant fill(0.0) on freshly default-constructed covariance arrays

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:329` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: rosidl-generated messages zero-initialize all array fields in their default constructor, so `linear_acceleration_covariance.fill(0.0)` (329), `angular_velocity_covariance.fill(0.0)` (334) and `magnetic_field_covariance.fill(0.0)` (353) are no-ops executed 100 times/s (27 double stores). Pure LOC and a few nanoseconds per tick.

Evidence detail: Messages are constructed at lines 313 and 344 immediately before the fills; rosidl default ctor (rosidl_runtime_cpp::MessageInitialization::ALL) zeroes arrays.

Estimated gain: -3 LOC, ~27 stores/tick

Fix sketch: Delete lines 329, 334, 353.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-07 — Temperature raw value parsed every tick but only consumed every 10th tick

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:298` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: t_raw is decoded on every 100 Hz tick (line 298) but only used inside the `if (++publish_count_ >= 10)` block (line 360-373). Trivial CPU, but it is work in the hot path that belongs inside the branch. Same pattern: the be16s lambda (288-290) is re-created per call; a static inline helper reads cleaner.

Evidence detail: Line 298 vs usage at line 364 only.

Estimated gain: Negligible CPU; clearer hot path

Fix sketch: Move `int16_t t_raw = be16s(&raw[TEMP_OFF]);` into the temperature block; hoist be16s to a file-scope `static inline`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-08 — Covariance constants computed at static-init with std::pow instead of constexpr

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:59` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ACCEL_VAR and GYRO_VAR are `static const double` initialised via std::pow at dynamic-init time. Writing them as `constexpr` with a multiplication (x*x) removes the runtime initializer and lets the compiler fold them into the message assignment at lines 330-337.

Evidence detail: Lines 59-60 use std::pow(..., 2) in a non-constexpr static initializer.

Estimated gain: Two fewer dynamic initializers; constant folding in publish_imu

Fix sketch: `static constexpr double ACCEL_SIGMA = 400e-6 * 9.80665; static constexpr double ACCEL_VAR = ACCEL_SIGMA * ACCEL_SIGMA;` etc.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-09 — Error messages print register in decimal after a '0x' prefix

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:140` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: `std::string("write_reg(0x") + std::to_string(reg)` (140), same at 167 and 212 (WHO_AM_I) produce e.g. "0x127" for register 0x7F. Misleading diagnostics that cost debugging time; also each builds 3-4 temporary strings. Not a hot path except via finding 04.

Evidence detail: Lines 140, 167, 212 combine a literal "0x" with std::to_string (decimal).

Estimated gain: Correct diagnostics; fewer temporaries

Fix sketch: Use snprintf with %02X into a small stack buffer, or drop the 0x prefix.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-11 — slam_nav2.launch.py and navigation_only.launch.py are self-declared LEGACY and have no code caller

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/slam_nav2.launch.py:5` · package jetank_navigation · lens minimality · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: slam_nav2.launch.py's docstring says it is LEGACY and that the web UI now uses slam.launch.py + nav2_bringup.launch.py separately; jetank_web_control/web_control_node.py lines 1037/1043 confirm it launches exactly those two. navigation_only.launch.py exists only as the wrapper for slam_nav2 (its docstring line 7). Together they are 121 LOC of launch code, installed via CMakeLists install(DIRECTORY launch/), plus the `use_localization=False` branch in nav2_bringup.launch.py (lines 71-81, 92) that exists only to serve them. README.md still documents them as 'used by the web control'.

Evidence detail: grep across src/ for slam_nav2/navigation_only: hits only in README/plans and jetank_ros_main/workspace_template/pixi.toml (which references navigation_full, not these). web_control_node.py:1037 -> 'slam.launch.py', :1043 -> 'nav2_bringup.launch.py'.

Estimated gain: -121 LOC launch, -~12 LOC branch in nav2_bringup.launch.py, one fewer installed entry-point

Fix sketch: Delete slam_nav2.launch.py and navigation_only.launch.py, drop the use_localization argument/branch and 'smoother_server' spec from nav2_bringup.launch.py, update README 'Sim launch files used by the web control' section.

Verifier (confirmed, adjusted low): Fix is safe and in-scope: every runtime include of nav2_bringup.launch.py (jetank_navigation/launch/navigation_full.launch.py:116, jetank_ros_main/launch/unified.launch.py:330, jetank_web_control/web_control_node.py:1043) uses the default use_localization=True, so dropping the False branch and the 'smoother_server' spec (which has no section in config/nav2/nav2_params.yaml) changes nothing for any consumer; the only caller of navigation_only.launch.py is slam_nav2.launch.py:57 and the only caller of use_localization:=False is navigation_only.launch.py:42. Non-code references remain in jetank_web_control/README.md:94-95 (stale doc, must be updated in the fix beyond the jetank_navigation README) and web_control_node.py:1016 keeps 'smoother_server' in a pkill pattern list, which is harmless. jetank_ros_main/plans/mapping-nav-separation-plan.md:19 explicitly chose to keep slam_nav2 on disk for manual use, but line 66 lists its removal as planned follow-up, so deletion is consistent with intent; no package merge or perception abstraction is touched. Severity lowered: dead launch files are never loaded, so there is zero CPU/memory cost on the Jetson, only LOC and stale docs.

Duplicate folded in: **cross-22** — same LEGACY slam_nav2/navigation_only pair and use_localization branch.

### jetank_navigation-12 — No-op remapping ('/scan' -> '/scan') in slam.launch.py

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/slam.launch.py:44` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: remappings=[('/scan', '/scan')] maps a topic to itself; it adds a `-r /scan:=/scan` argument to the process command line and does nothing.

Evidence detail: Line 44 of slam.launch.py.

Estimated gain: -1 LOC, one fewer ros-args token

Fix sketch: Delete line 44.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-13 — use_sim_time: False repeated in 14 nav2_params.yaml sections that RewrittenYaml overrides anyway

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:9` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: nav2_bringup.launch.py lines 51-59 rewrite use_sim_time for every node from the launch argument, so the 14 `use_sim_time: False` lines (9, 23, 27, 31, 110, 175, 184, 227, 257, 263, 273, 290, 298, 309) never take effect. Additionally the three `*_rclcpp_node` sections (lines 21-27, 173-175, 271-273) target sub-nodes that Nav2 Humble no longer creates, so those 12 lines are dead in their entirety.

Evidence detail: param_rewrites={'use_sim_time': use_sim_time, ...} in nav2_bringup.launch.py:51-53 applies to all keys; Humble nav2 removed the *_rclcpp_node helper nodes.

Estimated gain: -26 LOC of dead YAML

Fix sketch: Remove the use_sim_time keys and the bt_navigator_*_rclcpp_node / controller_server_rclcpp_node / planner_server_rclcpp_node sections.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-14 — Local costmap uses VoxelLayer with publish_voxel_map for a 2D-lidar-only robot

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:195` · package jetank_navigation · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: The only observation source is a planar LaserScan (line 204-210) yet the local costmap runs nav2_costmap_2d::VoxelLayer with 16 z-voxels and publish_voxel_map: True. VoxelLayer does 3D raytracing/marking per scan and publishes a VoxelGrid message on every update (5 Hz) that nothing in this package's RViz config displays. ObstacleLayer (already used by the global costmap, line 232) is the 2D equivalent with lower CPU and no extra topic. Also line 215-217 defines a `static_layer` block for the local costmap that is not in its `plugins` list (190) — dead config.

Evidence detail: nav2_params.yaml 190-218; no PointCloud2 observation source; rviz/navigation.rviz has no VoxelGrid display. Not measurable now (Nav2 not running).

Estimated gain: Lower per-scan costmap CPU at 5 Hz; removes a 5 Hz VoxelGrid publish; -3 LOC dead config

Fix sketch: Replace voxel_layer with `obstacle_layer: {plugin: nav2_costmap_2d::ObstacleLayer, observation_sources: scan, scan: {...}}`, drop publish_voxel_map/z_* keys, delete the unused local static_layer block.

Verifier (confirmed, adjusted low): nav2_params.yaml:190-218 confirms the local costmap runs VoxelLayer (z_voxels: 16, publish_voxel_map: True) fed only by a LaserScan source (:205-210), plus an orphan static_layer block (:215-217) absent from plugins (:190); rviz/navigation.rviz has no VoxelGrid display, so the 5 Hz VoxelGrid publish has no consumer. The gain is real but small: the local window is 3 m x 3 m at 0.025 m = 14,400 cells, so 3D raytracing of ~360 beams and a ~58 KB VoxelGrid publish at 5 Hz is sub-millisecond-scale on an Orin Nano, and this is just the nav2_bringup default config copied over. Fix is correct and trivial, but the est_gain wording overstates the CPU relevance; low severity is honest.

### jetank_navigation-15 — always_send_full_costmap: True on both costmaps forces full-grid publishes every cycle

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:218` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: With always_send_full_costmap True the costmap publisher sends the whole OccupancyGrid each publish tick instead of the changed window (costmap_updates). Local: 120x120 cells at 2 Hz; global: whole map at 1 Hz. On a Jetson with RViz over WiFi this is avoidable bandwidth and serialization; the default (False) sends updates.

Evidence detail: Lines 218 and 253 set True; publish_frequency 2.0 (181) and 1.0 (224).

Estimated gain: Reduces costmap publish bytes by the ratio of changed/total cells per cycle

Fix sketch: Set always_send_full_costmap: False (or delete the key) at 218 and 253.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-18 — slam_toolbox enable_interactive_mode: true publishes interactive markers for every graph node

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/slam/slam_toolbox.yaml:43` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Interactive mode keeps an InteractiveMarkerServer alive and re-publishes a marker per pose-graph vertex, growing with the map; it exists for manual graph editing in RViz and is off in the upstream default configs. On a headless or WiFi-streamed Jetson it is pure overhead. Line 35 transform_publish_period 0.02 also broadcasts map->odom at 50 Hz, above what AMCL/Nav2 need (upstream default 0.05).

Evidence detail: slam_toolbox.yaml lines 35 and 43. slam_toolbox not running during this session so not measured.

Estimated gain: Removes marker server + per-vertex marker traffic; halves map->odom TF publish rate

Fix sketch: enable_interactive_mode: false; transform_publish_period: 0.05.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-19 — RViz config subscribes to /stereo_camera/points (dropped pipeline) and renders all TF frames

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/rviz/navigation.rviz:38` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The PointCloud2 display on /stereo_camera/points belongs to the stereo-to-laserscan path that PROGRESS.md says was dropped; the stereo node is live on this robot, so opening navigation.rviz would subscribe to and render a full-rate dense point cloud on the Jetson GPU/CPU for no navigation purpose. `TF: All Enabled: true` (line 26) also renders every arm/gripper/wheel frame. Both cost RViz render time on the same box that runs SLAM/Nav2.

Evidence detail: rviz lines 38-41 and 23-26; get_node_list shows /stereo_camera/stereo_camera_node live; PROGRESS.md states the pointcloud path was removed.

Estimated gain: Avoids a dense PointCloud2 subscription + render in RViz; fewer TF frames drawn

Fix sketch: Delete the PointCloud display block (38-41); set TF 'All Enabled: false' and enable only map/odom/base_link/laser.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-20 — Nine nav2_* exec_depends are redundant with nav2_bringup; actually-imported packages are missing

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/package.xml:16` · package jetank_navigation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: nav2_bringup already exec-depends on nav2_map_server, nav2_amcl, nav2_controller, nav2_planner, nav2_behaviors, nav2_bt_navigator, nav2_waypoint_follower, nav2_velocity_smoother, nav2_lifecycle_manager, so lines 17-25 restate transitive deps (9 LOC). Meanwhile packages the launch files import directly are not declared: nav2_common (RewrittenYaml, nav2_bringup.launch.py:28), nav2_smoother (spec at :92), launch, launch_ros, ament_index_python, rviz2 + nav2_rviz_plugins (rviz config). rosdep-based installs would be incomplete while the list looks complete.

Evidence detail: package.xml lines 16-31 vs imports in launch/*.py and Panels in rviz/navigation.rviz.

Estimated gain: -9 LOC redundant deps; accurate rosdep set

Fix sketch: Keep nav2_bringup, slam_toolbox, rplidar_ros; add exec_depend launch, launch_ros, ament_index_python, nav2_common, nav2_rviz_plugins, rviz2; drop the individual nav2_* lines (or drop nav2_bringup and keep the explicit list, plus nav2_common/nav2_smoother).

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-27** — same redundant nav2_* exec_depends and missing nav2_common/launch/launch_ros/rviz2.

### jetank_navigation-21 — install(DIRECTORY maps/ ... OPTIONAL) installs an empty, untracked directory

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:39` · package jetank_navigation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: src/jetank_navigation/maps/ exists but is empty; save_map.sh writes to $HOME/maps (script line 16) and nav2 launches take an absolute map:= path. The install rule therefore never ships anything and misleads readers into thinking maps live in the package share dir.

Evidence detail: `ls -la maps` -> empty directory; save_map.sh MAP_DIR=${HOME}/maps.

Estimated gain: -5 LOC CMake; removes an empty share/jetank_navigation/maps install

Fix sketch: Delete lines 38-43 and the empty maps/ directory.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-22 — BUILD_TESTING pulls ament_lint_common (cpplint/uncrustify/xmllint/flake8...) for one .cpp with copyright+cpplint already disabled

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:57` · package jetank_navigation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_lint_auto_find_test_dependencies() with test_depend ament_lint_common (package.xml:33-34) registers ~6 linter tests that run on every `pixi run test`/colcon test on the Jetson, two of which are already forced off (lines 61, 65). There are no unit tests in the package, so the test stage is 100% linting overhead. Either keep only the linters wanted (ament_cmake_cppcheck / lint_cmake) or drop the block.

Evidence detail: CMakeLists.txt 57-67; package.xml 33-34; no test/ directory in the package.

Estimated gain: Faster colcon test; -2 test_depends

Fix sketch: Remove the BUILD_TESTING block and ament_lint_* test_depends, or replace with explicit find_package(ament_cmake_cppcheck) + ament_cppcheck().

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-23 — No default CMAKE_BUILD_TYPE: ad-hoc colcon builds of the IMU node are unoptimized

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:4` · package jetank_navigation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: pixi's build-nav task passes -DCMAKE_BUILD_TYPE=Release, but a plain `colcon build --packages-select jetank_navigation` (as QUICKSTART.md line 15 instructs) builds with no optimization flags. The node is small so the effect is modest, but the common ROS idiom of defaulting to Release when unset costs 3 lines and removes the trap.

Evidence detail: CMakeLists.txt sets only warning flags (lines 4-6); pixi.toml:25 passes Release explicitly; QUICKSTART.md:15 uses bare colcon build.

Estimated gain: Consistent -O2/-O3 binary regardless of build entry point

Fix sketch: `if(NOT CMAKE_BUILD_TYPE) set(CMAKE_BUILD_TYPE Release) endif()` before add_compile_options.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-24 — nav2_bringup.launch.py carries upstream-only arguments (namespace, use_respawn, log_level) no caller sets

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/nav2_bringup.launch.py:141` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: namespace (141-144, threaded through RewrittenYaml root_key and tf remappings 47-48), use_respawn (161-164, respawn/respawn_delay 109-110) and log_level (166-169, passed as --ros-args to every node) are copied from upstream nav2_bringup but never set by navigation_full.launch.py, navigation_only.launch.py or web_control. About 25 LOC of generality the project does not use; the tf remapping in particular adds two -r tokens to nine processes.

Evidence detail: Callers: navigation_full.launch.py:114-120 passes only map/use_sim_time; web_control_node.py:1043 passes map; navigation_only passes use_sim_time/params_file/autostart/log_level.

Estimated gain: ~25 LOC

Fix sketch: Drop namespace/use_respawn/log_level args, the tf remappings, and RewrittenYaml root_key.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-25 — RViz launched through nav2_bringup's rviz_launch.py wrapper instead of a direct Node

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/navigation_full.launch.py:122` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 122-135 include nav2_bringup/launch/rviz_launch.py (which itself declares namespace/use_namespace args and a GroupAction with remappings) just to run rviz2 with navigation.rviz. A direct `Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_file])` is 4 lines, removes an extra launch-file parse and the hidden dependency on nav2_bringup's launch layout.

Evidence detail: navigation_full.launch.py 122-135; the only launch_arguments passed are rviz_config and use_sim_time.

Estimated gain: ~8 LOC and one fewer nested launch include at startup

Fix sketch: Replace the GroupAction/IncludeLaunchDescription with a conditioned rviz2 Node.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-26 — Dead/duplicated DWB and AMCL keys in nav2_params.yaml

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:157` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: FollowPath.xy_goal_tolerance (157) duplicates general_goal_checker.xy_goal_tolerance (130) — DWB reads the goal checker's value, not its own. amcl.laser_min_range 0.05 (76) and scan_topic/map_topic (103-104) equal upstream defaults; `map_server.yaml_filename: ""` (258) is always overwritten by RewrittenYaml (launch line 53). Each is a line that reads as configuration but changes nothing.

Evidence detail: nav2_params.yaml lines 157, 76, 103-104, 258; nav2_bringup.launch.py param_rewrites includes yaml_filename.

Estimated gain: -5 LOC

Fix sketch: Delete the listed keys.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-27 — Stray .claude/settings.local.json and empty first line in .gitignore inside the package

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/.claude/settings.local.json:1` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: A per-package Claude Code permissions file (WebFetch allow-list) lives inside the ROS package tree; it is not part of the package and is picked up by nothing in the build. .gitignore begins with a blank line. Both are noise in a package that otherwise contains only build/launch/config assets.

Evidence detail: Files listed by find; contents shown above (7-line JSON, 4-line gitignore).

Estimated gain: -1 stray file

Fix sketch: Move the setting to the workspace-level .claude/settings.local.json and delete the package copy.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_navigation-28 — AMCL particle count and update thresholds set high for a small indoor robot

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:44` · package jetank_navigation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: min_particles 500 / max_particles 2000 with resample_interval 1 and update_min_d 0.05 m / update_min_a 0.1 rad (lines 44-57) means AMCL re-weights up to 2000 particles against 120 beams every 5 cm of travel. For a ~5 m room with a 0.3 m/s robot and a 12 m lidar, upstream-typical values (min 200-300, max 1000-2000, resample_interval 2) give equivalent convergence at roughly half the per-update CPU. Static tuning claim only; AMCL was not running to measure.

Evidence detail: nav2_params.yaml 44-57 and 77 (max_beams 120).

Estimated gain: ~2x fewer particle updates per metre travelled

Fix sketch: min_particles: 300, resample_interval: 2; keep max_particles 2000 for recovery.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_simulation

Coverage: 24 files read; 3 measurements run. Notes: All source, config, build, launch, world, test and doc files in the package were read in full (the package has no C++, YAML config, xacro, JS or HTML files of its own; the ~880-line estimate corresponds to the launch+script+test Python, the rest is SDF/Markdown). Not read: .git/ internals, .pytest_cache/ (build artefacts, out of scope). All findings are evidence='static' because no node of this package is running; runtime claims about Gazebo step cost, shadow passes, bridge RSS and relay publish rate are reasoned from the read files and Gazebo/rclpy semantics, not measured. Finding 07's fix depends on the installed controller_manager spawner accepting multiple controller names (Humble >= 2.27); verify before applying.

Measurements:
- mcp__ros2-mcp__get_node_list: 16 nodes live (robot_state_publisher, joint_state_publisher, robot_controller, icm20948_imu, web_control_node, stereo_camera_node, move_group, 3 controller spawners, ...) — none belong to jetank_simulation (no gz sim, ros_gz_bridge, ros_gz_sim create or gripper_mimic_relay node), so no runtime metric of this package could be measured.
- mcp__ros2-mcp__get_topic_list: 49 topics; no /clock, /scan, /imu (sim), or /gripper_right_mimic_controller/commands present, confirming the sim stack is down. get_topic_hz / profile_node / get_topic_bw were therefore not run.
- Static cross-checks via grep: gazebo_remote/robot_remote referenced by no other package; ros_gz_image and joint_state_publisher referenced only in package.xml; no contact sensor in jetank_description/urdf; controller_manager update_rate 50 Hz, IMU 100 Hz, cameras 640x360 @ 30 Hz.

Findings: 26 (high 0, medium 3, low 23).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_simulation-09 | runtime | robot_remote.launch.py spawners have no sequencing and no --controller-manager-timeout | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/robot_remote.launch.py:74` | medium | static | Removes a startup race on the Jetson; avoids re-running the launch | S | confirmed |
| jetank_simulation-14 | runtime | 1 ms physics step in all five worlds is 20x the controller rate | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:7` | medium | static | 2-4x fewer physics steps per sim-second; largest single CPU reduction available in this package | S | confirmed |
| jetank_simulation-19 | footprint | ros2_control / ros2_controllers metapackages pulled in instead of the five controllers actually used | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:33` | medium | static | ~15 fewer controller packages required for a rosdep/pixi install of this package | S | confirmed |
| jetank_simulation-01 | runtime | Relay dedup threshold 1e-6 m republishes on every /joint_states message under physics jitter | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:91` | low | static | Drop steady-state publish rate from ~50 Hz to ~0 Hz when gripper is idle; removes ~50 msg allocations/s | S | **UNVERIFIED** |
| jetank_simulation-02 | runtime | Eager f-string formatting in the per-message debug log | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:98` | low | static | Removes one string format per publish (~50/s worst case) | S | **UNVERIFIED** |
| jetank_simulation-03 | minimality | Explicit QoSProfile in relay is identical to the rclpy default depth-10 profile | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:49` | low | static | -12 LOC | S | **UNVERIFIED** |
| jetank_simulation-04 | runtime | Relay is given use_sim_time although it never reads the clock | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:223` | low | static | Removes one /clock subscription and its callback stream from the relay process | S | **UNVERIFIED** |
| jetank_simulation-05 | runtime | Two separate ros_gz_bridge parameter_bridge processes where one suffices | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:144` | low (orig medium) | static | One fewer process/DDS participant; est. 20-40 MB RSS and one discovery participant saved | S | confirmed |
| jetank_simulation-06 | runtime | Gazebo launched with -v 4 (debug verbosity) in every launch path | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:89` | low (orig medium) | static | Eliminates debug-level log formatting and console I/O from the Gazebo server process | S | confirmed |
| jetank_simulation-07 | runtime | Five separate spawner processes launched in parallel for the controllers | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:159` | low (orig medium) | static | 4 fewer transient Python processes/participants at startup; faster, less contended controller bring-up | S | confirmed |
| jetank_simulation-08 | minimality | Duplicate GUI/headless IncludeLaunchDescription blocks differ only by '-s' | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:94` | low | static | -10 LOC, one fewer launch entity | S | **UNVERIFIED** |
| jetank_simulation-11 | runtime | Remote launch bridges raw 640x360 RGB images over the network via parameter_bridge | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_remote.launch.py:53` | low | static | ~41 MB/s raw -> ~2-4 MB/s compressed over the network if the remote path is kept | S | **UNVERIFIED** |
| jetank_simulation-12 | minimality | gazebo_headless.launch.py is a 72-line wrapper for a single gui:=false argument | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_headless.launch.py:31` | low | static | -72 LOC, -1 test parametrization, one fewer launch include level | S | **UNVERIFIED** |
| jetank_simulation-13 | minimality | load_robot_description parameters are never varied and one mapping is the xacro default | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/jetank_sim_description.py:18` | low | static | -3 LOC, simpler call sites | S | **UNVERIFIED** |
| jetank_simulation-15 | runtime | Shadow mapping enabled in every world for two 30 Hz camera sensors | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:27` | low | static | Removes one shadow depth pass per rendered sensor frame (60+ passes/s) | S | **UNVERIFIED** |
| jetank_simulation-16 | runtime | Contact system plugin loaded in all worlds but the robot has no contact sensor | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:15` | low | static | One fewer system iterated every physics step (1000/s at current step size) | S | **UNVERIFIED** |
| jetank_simulation-18 | runtime | Semi-transparent marker models cost an extra ogre2 transparency pass per frame | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/sock_arena.sdf:555` | low | static | Removes the transparency pass from 60 sensor frames/s in those worlds | S | **UNVERIFIED** |
| jetank_simulation-20 | footprint | ros_gz_image declared but never used | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:15` | low | static | One unused runtime dependency removed | S | **UNVERIFIED** |
| jetank_simulation-21 | footprint | joint_state_publisher declared but never launched | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:20` | low | static | One unused runtime dependency removed | S | **UNVERIFIED** |
| jetank_simulation-22 | footprint | ament_lint_common runs the full C++ linter suite on a package with no C++ | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:39` | low | static | Fewer lint dependencies installed; faster `colcon test` | S | **UNVERIFIED** |
| jetank_simulation-23 | minimality | Dead C++ compile options and template comments in a data-only CMakeLists | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:4` | low | static | -15 LOC CMake, -4 LOC README | S | **UNVERIFIED** |
| jetank_simulation-24 | minimality | 80 lines of rclpy/msg stub scaffolding for an environment the test never runs in | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/test/test_gripper_mimic_relay.py:39` | low | static | -60 LOC test code | S | **UNVERIFIED** |
| jetank_simulation-25 | minimality | Launch import tests re-run xacro processing for every parametrized case | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/test/test_launch_import.py:66` | low | static | ~50% fewer xacro invocations in `colcon test` | S | **UNVERIFIED** |
| jetank_simulation-26 | minimality | README duplicates its own launch/world tables and is stale on the test action counts | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/README.md:70` | low | static | -40 LOC docs, one source of truth | S | **UNVERIFIED** |
| jetank_simulation-10 | minimality | gazebo_remote/robot_remote pair duplicates gazebo.launch.py and has no callers | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_remote.launch.py:1` | low (orig medium) | static | -189 LOC launch, -2 test parametrizations, -20 lines README | M | confirmed |
| jetank_simulation-17 | minimality | ~90 lines of identical physics/plugin/scene/GUI boilerplate copied into each of five worlds | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:32` | low | static | -250 to -450 LOC across worlds; single place to tune physics/rendering | M | **UNVERIFIED** |

### jetank_simulation-09 — robot_remote.launch.py spawners have no sequencing and no --controller-manager-timeout

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/robot_remote.launch.py:74` · package jetank_simulation · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: The three spawners (lines 74-98) start at launch time in parallel with spawn_robot, without the OnProcessExit chaining and 60 s timeout that gazebo.launch.py added specifically to fix the 'spawners fire before /controller_manager exists' race (gazebo.launch.py lines 154-158, 245-250). On the Jetson side of the remote setup they will retry against a not-yet-existing manager and give up after the 10 s default, leaving the robot without controllers — wasted startup CPU and a broken bring-up.

Evidence detail: Compared robot_remote.launch.py 74-112 with gazebo.launch.py 159-269.

Estimated gain: Removes a startup race on the Jetson; avoids re-running the launch

Fix sketch: Reuse the same OnProcessExit chain and _cm_timeout list, or better, delete the remote pair per finding 10.

Verifier (confirmed, adjusted medium): robot_remote.launch.py:74-98 defines three spawners with only `--controller-manager` and no `--controller-manager-timeout`, and lines 107-111 add them flat to the LaunchDescription alongside spawn_robot; grep for OnProcessExit/RegisterEventHandler/controller-manager-timeout in that file returns nothing. gazebo.launch.py:154-159 adds the 60 s timeout and lines 245-269 chain spawners on spawn_robot exit specifically to fix this race, so the remote pair lacks the fix. The file is still documented in jetank_simulation/README.md and imported by test/test_launch_import.py, so it is a live launch path rather than dead code. Static evidence only; no runtime measurement was taken.

### jetank_simulation-14 — 1 ms physics step in all five worlds is 20x the controller rate

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:7` · package jetank_simulation · lens runtime · severity medium · evidence static · effort S · verdict confirmed

Description: Every world sets max_step_size 0.001 (1000 physics steps per simulated second) while controller_manager runs at 50 Hz and the fastest sensor (IMU) at 100 Hz. On a CPU-bound Jetson the physics loop is the dominant sim cost and scales linearly with step count; the ign_ros2_control plugin also runs its read/write hooks per step. A 2-4 ms step (250-500 Hz) is standard for small differential-drive robots. Same value in simple_test.sdf:7, obstacle_course.sdf:7, sock_arena.sdf:7, house.sdf:7.

Evidence detail: Read <physics> blocks of all five worlds; controller_manager update_rate 50 (jetank_controllers.yaml:6), IMU 100 Hz (imu.xacro:43). Gripper contact stiffness was already lowered to 1e4 (gripper.xacro:132) which makes larger steps more tolerable, but stability must be checked with the gripper grasp test. No live sim to measure RTF.

Estimated gain: 2-4x fewer physics steps per sim-second; largest single CPU reduction available in this package

Fix sketch: Set <max_step_size>0.004</max_step_size> (or 0.002 if grasping jitters) in all worlds and re-run the SIM_TESTING grasp + drive checks; keep real_time_factor 1.0.

Verifier (confirmed, adjusted medium): All five worlds do set max_step_size 0.001 (e.g. worlds/empty_fortress.sdf:7, sock_arena.sdf:7) against controller update_rate 50 (jetank_motor_control/config/jetank_controllers.yaml:6) and IMU 100 Hz (imu.xacro:43), so a 2-4 ms step really does cut physics + ign_ros2_control per-step hooks by 2-4x linearly; no launch file overrides the step. However "largest single CPU reduction in the package" is unverified: every world also loads the Sensors system with ogre2 (sock_arena.sdf:17-18) rendering two 30 Hz cameras (camera.xacro:89,127) plus gpu_lidar, which on an Orin Nano is plausibly a comparable or larger cost, and no live sim was running to measure RTF. The gripper kp/kd=1e4 argument (gripper.xacro:136-144) is weak since Fortress/DART ignores ODE-style contact stiffness, so grasp stability at 4 ms is a genuine open risk rather than a mitigated one; medium is honest for an S-effort, config-only change with a real but unbounded gain.

### jetank_simulation-19 — ros2_control / ros2_controllers metapackages pulled in instead of the five controllers actually used

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:33` · package jetank_simulation · lens footprint · severity medium · evidence static · effort S · verdict confirmed

Description: exec_depend ros2_control and ros2_controllers are metapackages: ros2_controllers drags in every controller (admittance, tricycle, steering, pid, imu/range broadcasters, ...) and ros2_control the full stack. The launch files only need controller_manager (spawner) and the controller types named in jetank_controllers.yaml: joint_state_broadcaster, diff_drive_controller, joint_trajectory_controller, position_controllers, forward_command_controller. diff_drive_controller (line 37) is then redundant with ros2_controllers. ament_index_python (used by three launch files) and rosgraph_msgs (bridged /clock type) are not declared at all.

Evidence detail: package.xml lines 32-37; controller types from jetank_motor_control/config/jetank_controllers.yaml lines 9-33; launch imports of ament_index_python at gazebo.launch.py:4, gazebo_remote.launch.py:16, gazebo_headless.launch.py:20.

Estimated gain: ~15 fewer controller packages required for a rosdep/pixi install of this package

Fix sketch: Replace ros2_control/ros2_controllers/diff_drive_controller with controller_manager, joint_state_broadcaster, diff_drive_controller, joint_trajectory_controller, position_controllers, forward_command_controller; add ament_index_python and rosgraph_msgs.

Verifier (confirmed, adjusted medium): package.xml:33-37 declares the ros2_control and ros2_controllers metapackages plus controller_manager and diff_drive_controller (redundant with ros2_controllers). The controllers actually loaded (jetank_motor_control/config/ros2_control.xacro:229 -> jetank_controllers.yaml:10-33) are only joint_state_broadcaster, joint_trajectory_controller, position_controllers, forward_command_controller and diff_drive_controller. ament_index_python is imported at gazebo.launch.py:4, gazebo_remote.launch.py:16, gazebo_headless.launch.py:20 and jetank_sim_description.py:14, and rosgraph_msgs/msg/Clock is bridged at gazebo.launch.py:138 and gazebo_remote.launch.py:61, yet neither is declared in package.xml. Severity stays medium: this is install/dependency weight only, no runtime cost.

### jetank_simulation-01 — Relay dedup threshold 1e-6 m republishes on every /joint_states message under physics jitter

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:91` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _js_callback skips a publish only if |pos - last| < 1e-6 m. In Gazebo the held gripper_left_joint position reported by ign_ros2_control jitters at ~1e-5..1e-4 m, so the guard almost never fires and the node allocates a Float64MultiArray, a Python list and a debug f-string, then publishes, on every /joint_states message (50 Hz per controller_manager update_rate in jetank_motor_control/config/jetank_controllers.yaml). The ForwardCommandController then re-applies an identical command each cycle. Cost: ~50 msg/s + 50 allocations/s of pure churn on a Jetson core.

Evidence detail: Read scripts/gripper_mimic_relay lines 87-100 and controller_manager update_rate 50 Hz in jetank_controllers.yaml line 6. No relay node is live, so the actual republish rate was not measured.

Estimated gain: Drop steady-state publish rate from ~50 Hz to ~0 Hz when gripper is idle; removes ~50 msg allocations/s

Fix sketch: Raise the threshold to a physically meaningful value (e.g. 1e-4 m, 0.1 mm — well below the 5 mm gripper goal_tolerance) and pre-allocate `self._cmd = Float64MultiArray(); self._cmd.data = [0.0]` in __init__, updating `self._cmd.data[0] = pos` before publish.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-02 — Eager f-string formatting in the per-message debug log

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:98` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: `self.get_logger().debug(f"Forwarded gripper_right position: {pos:.5f} m")` formats the string on every publish even though the DEBUG level is disabled by default, so the float formatting and string concat run at the publish rate for nothing.

Evidence detail: Python evaluates f-string arguments before the call; rclpy has no lazy-format API, so only a level check or removal avoids it. Read at line 98-100.

Estimated gain: Removes one string format per publish (~50/s worst case)

Fix sketch: Delete the debug log (the topic itself is observable with `ros2 topic echo`), or guard with `if self.get_logger().is_enabled_for(LoggingSeverity.DEBUG):`.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-03 — Explicit QoSProfile in relay is identical to the rclpy default depth-10 profile

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:49` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: QoSProfile(reliability=RELIABLE, durability=VOLATILE, history=KEEP_LAST, depth=10) is exactly what passing the integer `10` yields in rclpy. The 6-line profile plus the 6-line qos import block (lines 28-33) add 12 lines and no behaviour. `self._left_joint` (line 44) is also a per-instance constant that could be a module-level constant.

Evidence detail: rclpy.qos.QoSProfile(depth=10) defaults: RELIABLE, VOLATILE, KEEP_LAST. Read lines 28-33 and 48-54.

Estimated gain: -12 LOC

Fix sketch: Pass `10` as the QoS argument to create_subscription and drop the rclpy.qos import block.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-04 — Relay is given use_sim_time although it never reads the clock

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:223` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gripper_mimic_relay is launched with parameters=[{'use_sim_time': use_sim_time}]. The node uses no timers or time stamps, but with use_sim_time=true rclpy's TimeSource creates a /clock subscription and runs a callback per bridged clock message for the lifetime of the node. Pure overhead on the Jetson when the sim stack is co-hosted.

Evidence detail: Read scripts/gripper_mimic_relay (no get_clock/now/timer usage) and gazebo.launch.py lines 218-224.

Estimated gain: Removes one /clock subscription and its callback stream from the relay process

Fix sketch: Drop the `parameters=[...]` line from the gripper_mimic_relay Node action.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-05 — Two separate ros_gz_bridge parameter_bridge processes where one suffices

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:144` · package jetank_simulation · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: bridge_camera (lines 130-141) and bridge_sensors (lines 144-152) each start a full parameter_bridge process, i.e. two DDS participants, two gz-transport nodes, two sets of discovery traffic and roughly double the resident memory (each parameter_bridge is a C++ node linking rclcpp + gz-transport + gz-msgs; typically tens of MB RSS each). parameter_bridge accepts any number of topic mappings in one invocation, so the split gives no isolation benefit.

Evidence detail: Read gazebo.launch.py lines 129-152; parameter_bridge takes a variadic list of <topic>@<ros>[<gz> arguments. No bridge node is live to measure RSS.

Estimated gain: One fewer process/DDS participant; est. 20-40 MB RSS and one discovery participant saved

Fix sketch: Merge the seven mappings into a single Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[...]) and remove bridge_sensors (also fixes the README/test action count).

Verifier (confirmed, adjusted low): gazebo.launch.py:130-141 (bridge_camera) and :144-152 (bridge_sensors) are two unconditional ros_gz_bridge parameter_bridge Nodes, both added at :243-244, with identical package/executable/output and no differing condition, namespace, remap, or params that would justify separate processes; parameter_bridge accepts all seven mappings in one arguments list. Merging is straightforward but must also update the pinned top-level action count in test/test_launch_import.py:28-66 and README.md:70 (13 -> 12). Gain is one extra process/DDS participant in simulation only, so I rate it low-medium.

### jetank_simulation-06 — Gazebo launched with -v 4 (debug verbosity) in every launch path

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:89` · package jetank_simulation · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: gz_args is `<world> -r -v 4` for both GUI (line 89) and headless (line 97) includes and in gazebo_remote.launch.py line 46. Level 4 enables debug-level console output from every gz-sim system (sensors, physics, transport) which is piped through the launch process with output='screen'. At 1 kHz physics + 2 cameras at 30 Hz this is a steady stream of stdout I/O competing with the sim loop on the Jetson.

Evidence detail: gz sim verbosity levels: 1=error, 2=warning, 3=info, 4=debug. Read gazebo.launch.py lines 86-101 and gazebo_remote.launch.py lines 43-49.

Estimated gain: Eliminates debug-level log formatting and console I/O from the Gazebo server process

Fix sketch: Use `-v 2` (warnings + errors) by default and expose a `gz_verbosity` launch argument for debugging sessions.

Verifier (confirmed, adjusted low): The `-v 4` flags are real at gazebo.launch.py:89, :97 and gazebo_remote.launch.py:46, and a workspace grep shows no consumer parses gz console output (no OnProcessIO/stdout handlers, no [Dbg]/[Msg] matching in any package), so changing to `-v 2` plus a `gz_verbosity` DeclareLaunchArgument alters only log volume, stays inside jetank_simulation, and touches no perception strategy/factory code. One in-scope side effect: adding a DeclareLaunchArgument changes the top-level action count that jetank_simulation/test/test_launch_import.py:28-71 asserts (13/1/3/6), so that test's expected counts must be bumped in the same change. Severity lowered to low: the evidence is static only, and gz-sim debug output is not emitted per physics tick, so the steady-state I/O cost claimed is unmeasured and likely modest.

### jetank_simulation-07 — Five separate spawner processes launched in parallel for the controllers

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:159` · package jetank_simulation · lens runtime · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: After joint_state_broadcaster exits, five Python spawner processes (diff_drive, arm inactive/active, gripper, gripper_right_mimic) start simultaneously (lines 257-269). Each is an rclpy process with its own DDS participant that spins up (~1-2 s import + discovery each on Jetson) and hammers /controller_manager services concurrently — the very contention the 60 s timeout comment describes. The Humble spawner accepts multiple controller names in one invocation.

Evidence detail: Read gazebo.launch.py lines 154-224 and 257-269. controller_manager spawner in Humble (>=2.27) takes `controller_names` as nargs='+'; verify the installed RoboStack version supports it before applying.

Estimated gain: 4 fewer transient Python processes/participants at startup; faster, less contended controller bring-up

Fix sketch: One spawner Node with arguments=['diff_drive_controller','gripper_controller','gripper_right_mimic_controller', '--controller-manager','/controller_manager', *_cm_timeout]; keep arm_controller as its own (conditional --inactive) spawner, or chain the arm spawner after the combined one.

Verifier (confirmed, adjusted low): The mechanism is real: gazebo.launch.py:257-269 fires the spawners in parallel and pixi.lock:1509 pins ros-humble-controller-manager 2.54.0 (aarch64), which is >=2.27 and accepts multiple controller names, so the fix applies. But the gain is overstated: arm_controller_spawner_inactive/active are mutually exclusive (UnlessCondition/IfCondition at lines 187/195), so only 4 spawners run concurrently, and a combined spawner saves 3 (or 2 if the arm spawner stays separate) transient processes, not 4. These are one-shot startup processes with zero steady-state cost on the Jetson, and Gazebo bring-up is a sim-only path, so the honest severity is low.

### jetank_simulation-08 — Duplicate GUI/headless IncludeLaunchDescription blocks differ only by '-s'

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:94` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gz_sim_gui (86-93) and gz_sim_headless (94-101) are two 8-line includes of the same gz_sim.launch.py with conditions IfCondition/UnlessCondition on `gui`; only the `-s` flag differs. One include with gz_args built from a PythonExpression removes 8 lines and the two conditions, and removes one entity from the LaunchDescription that the tests must count.

Evidence detail: Read gazebo.launch.py lines 78-101 and 239-240.

Estimated gain: -10 LOC, one fewer launch entity

Fix sketch: gz_args=[world, PythonExpression(["' -r -v 2' if '", gui, "' == 'true' else ' -s -r -v 2'"])] on a single IncludeLaunchDescription.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-11 — Remote launch bridges raw 640x360 RGB images over the network via parameter_bridge

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_remote.launch.py:53` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The laptop-side bridge publishes both camera streams as raw sensor_msgs/Image (640x360x3 B x 30 Hz x 2 = ~41 MB/s) for the Jetson to subscribe to over Wi-Fi. ros_gz_image's image_bridge (already declared in package.xml but unused) publishes via image_transport, so the Jetson could subscribe to the compressed transport instead of saturating the link and the Jetson's DDS receive path.

Evidence detail: Camera resolution/rate from jetank_description/urdf/components/camera.xacro lines 89-94; gazebo_remote.launch.py lines 53-64 uses parameter_bridge for Image.

Estimated gain: ~41 MB/s raw -> ~2-4 MB/s compressed over the network if the remote path is kept

Fix sketch: If finding 10 keeps the remote pair: use Node(package='ros_gz_image', executable='image_bridge', arguments=['/stereo_camera/left/image_raw','/stereo_camera/right/image_raw']) for images and parameter_bridge only for camera_info/clock. Otherwise delete with finding 10.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-12 — gazebo_headless.launch.py is a 72-line wrapper for a single gui:=false argument

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_headless.launch.py:31` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The file re-declares three arguments and imports os/get_package_share_directory only to duplicate the default world path, then includes gazebo.launch.py with gui:=false. The only external caller is jetank_moveit_config/launch/moveit_sim.launch.py line 77, which already branches on a `headless` flag and could pass gui:=false directly; jetank_ros_main's gazebo_sim.launch.py already does exactly that. The wrapper also forces a second xacro-free but non-trivial launch-description evaluation and a 4-entity test case.

Evidence detail: grep for gazebo_headless.launch outside the package: moveit_sim.launch.py:77 (chooses filename by headless flag) and jetank_motor_control deprecation strings/docs only.

Estimated gain: -72 LOC, -1 test parametrization, one fewer launch include level

Fix sketch: Change moveit_sim.launch.py to include gazebo.launch.py with 'gui': 'false' when headless, update docs/SIM_TESTING to `gazebo.launch.py gui:=false`, delete the wrapper.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-13 — load_robot_description parameters are never varied and one mapping is the xacro default

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/jetank_sim_description.py:18` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Both callers (gazebo.launch.py:33, robot_remote.launch.py:33) pass use_sim='true', use_ros2_control='true', identical to the function defaults, and use_ros2_control='true' is already the xacro's own default (jetank_ros2_control.urdf.xacro line 8). The parameters and the mapping entry are generality that nothing uses.

Evidence detail: Read jetank_sim_description.py lines 18-33; xacro:arg defaults at jetank_description/urdf/jetank_ros2_control.urdf.xacro lines 7-8.

Estimated gain: -3 LOC, simpler call sites

Fix sketch: def load_robot_description(): ... mappings={'use_sim': 'true'}; call without arguments.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-15 — Shadow mapping enabled in every world for two 30 Hz camera sensors

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:27` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: <scene><shadows>true</shadows> plus <cast_shadows>true</cast_shadows> on the directional light (line 95) makes ogre2 render a shadow-depth pass for the sun for every camera-sensor frame (2 cameras x 30 Hz) and the GUI view, even headless. Shadows add little to stereo-depth or sock-detection realism on primitive-box worlds. Repeated in simple_test.sdf:27/92, obstacle_course.sdf:27/92, sock_arena.sdf:27/92, house.sdf:25/87.

Evidence detail: Read <scene> and <light> blocks in all five worlds; camera sensors at 30 Hz from camera.xacro:89/127. Rendering cost not measured (no live sim).

Estimated gain: Removes one shadow depth pass per rendered sensor frame (60+ passes/s)

Fix sketch: Set <shadows>false</shadows> and <cast_shadows>false</cast_shadows> in all worlds; re-enable per world only if a vision test needs them.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-16 — Contact system plugin loaded in all worlds but the robot has no contact sensor

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:15` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ignition::gazebo::systems::Contact only serves <sensor type="contact"> elements; collision response itself is handled by the Physics system. The URDF has no contact sensor (the only 'contact' hit in jetank_description/urdf is a comment in gripper.xacro:132), so the system runs its per-step Update for nothing. Same line in simple_test.sdf:15, obstacle_course.sdf:15, sock_arena.sdf:15, house.sdf:15.

Evidence detail: grep -i contact over jetank_description/urdf found only gripper.xacro comment lines 132-133; no type="contact" sensor.

Estimated gain: One fewer system iterated every physics step (1000/s at current step size)

Fix sketch: Remove the libignition-gazebo-contact-system.so plugin line from all five worlds.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-18 — Semi-transparent marker models cost an extra ogre2 transparency pass per frame

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/sock_arena.sdf:555` · package jetank_simulation · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: basket (alpha 0.3, line 555) and collection_zone (alpha 0.5, line 536) in sock_arena, and start_zone/goal_zone (alpha 0.5) in obstacle_course.sdf lines 468 and 487, are visual-only models with translucent materials. ogre2 sorts and renders transparent objects in a separate pass for every camera sensor frame; opaque flat markers give the same navigational cue at no extra cost. The basket is also a solid translucent box the robot cannot enter, not a container.

Evidence detail: Read material blocks at the cited lines; ogre2 handles alpha<1 materials via its transparency queue.

Estimated gain: Removes the transparency pass from 60 sensor frames/s in those worlds

Fix sketch: Set alpha to 1 on the zone markers (they are 2 cm thick floor tiles) and model the basket as four thin opaque wall boxes or delete it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-20 — ros_gz_image declared but never used

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:15` · package jetank_simulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: No launch file references ros_gz_image or image_bridge; images are bridged through parameter_bridge. The dependency pulls image_transport + plugins into the install for nothing (unless finding 11 adopts image_bridge).

Evidence detail: grep -rn 'ros_gz_image|image_bridge' jetank_simulation matched only package.xml:15.

Estimated gain: One unused runtime dependency removed

Fix sketch: Delete the exec_depend, or start using image_bridge per finding 11 and keep it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-08** — same unused ros_gz_image exec_depend; footprint-08 also points at pixi.toml (ros-humble-ros-gz-image), but see jetank_ros_main-31 verifier note that ros_gz_image is sim-only rather than unused.

### jetank_simulation-21 — joint_state_publisher declared but never launched

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/package.xml:20` · package jetank_simulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Joint states come from joint_state_broadcaster (ros2_control); no launch file starts joint_state_publisher. The exec_depend is dead.

Evidence detail: grep -rn 'joint_state_publisher' jetank_simulation matched only package.xml:20.

Estimated gain: One unused runtime dependency removed

Fix sketch: Delete the exec_depend line.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-22 — ament_lint_common runs the full C++ linter suite on a package with no C++

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:39` · package jetank_simulation · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_lint_auto_find_test_dependencies() with test_depend ament_lint_common (package.xml:40) registers cppcheck, uncrustify, lint_cmake, flake8, pep257, xmllint (cpplint/copyright are disabled at lines 34/38) as colcon tests. cppcheck and uncrustify have nothing to check here and add test-time and install-time weight (ament_lint_common depends on the uncrustify/cppcheck binaries).

Evidence detail: Read CMakeLists.txt lines 30-44 and package.xml lines 39-41; package has no C/C++ sources.

Estimated gain: Fewer lint dependencies installed; faster `colcon test`

Fix sketch: Replace ament_lint_common with ament_cmake_flake8 + ament_cmake_xmllint (and ament_cmake_pep257 if wanted) and call them explicitly, dropping ament_lint_auto and the two *_FOUND overrides.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-23 — Dead C++ compile options and template comments in a data-only CMakeLists

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:4` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 4-6 add -Wall -Wextra -Wpedantic but there is no compiled target; lines 10-12 are ament template placeholder comments; lines 32-38 are the template's copyright/cpplint explanations. The install loop (16-22) lists `config` and `models` directories that do not exist in the package, and the if(EXISTS) guard plus the README section explaining it (README.md:58-61) exist only to tolerate those absent entries.

Evidence detail: Directory listing shows only launch/, worlds/, scripts/, test/ — no config/ or models/. Read CMakeLists.txt in full.

Estimated gain: -15 LOC CMake, -4 LOC README

Fix sketch: Delete lines 4-6 and 10-12; replace the loop with `install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})`; trim the lint comments.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-24 — 80 lines of rclpy/msg stub scaffolding for an environment the test never runs in

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/test/test_gripper_mimic_relay.py:39` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: _install_stubs (lines 33-86) fakes rclpy, rclpy.node, rclpy.qos, sensor_msgs and std_msgs 'if absent (bare pixi env)'. The test is registered only through ament_add_pytest_test (CMakeLists.txt:43), which runs inside the ROS env where all of these are real, and README.md:84 documents running it via `pixi run`, also inside the env. The fallback path is dead and inflates a 230-line test file for a 30-line callback.

Evidence detail: Read test_gripper_mimic_relay.py lines 29-107 and the two documented invocation paths (CMakeLists.txt:41-43, README.md:75-85).

Estimated gain: -60 LOC test code

Fix sketch: Delete _make_stub/_install_stubs and the module-level pytest.skip wrapper; import rclpy/msgs directly (they are test-time available) and keep the _make_relay + callback tests.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-25 — Launch import tests re-run xacro processing for every parametrized case

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/test/test_launch_import.py:66` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: test_generate_launch_description_returns_launch_description and test_gazebo_declares_expected_launch_arguments each call generate_launch_description() afresh, and gazebo.launch.py/robot_remote.launch.py run xacro.process_file on the full robot model on every call (4 xacro runs per test session, plus 2 more if finding 10/12 is not applied). test_launch_file_has_generate_function additionally takes an unused `_expected` parameter. A module-scoped fixture that generates once per file would halve test wall time.

Evidence detail: Read test_launch_import.py lines 29-87; xacro processing at jetank_sim_description.py:29.

Estimated gain: ~50% fewer xacro invocations in `colcon test`

Fix sketch: Add a @pytest.fixture(scope='module', params=LAUNCH_FILES) that loads the module and calls _generate once, and reuse it across the assertions.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-26 — README duplicates its own launch/world tables and is stale on the test action counts

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/README.md:70` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Line 70 says the expected action counts are '13 / 1 / 3 / 6' while the test (test_launch_import.py:30-34) expects 13 / 4 / 3 / 6. The launch-file table (lines 8-13) and worlds list (45-51) are repeated in the 'ROS 2 API' section (95-100, 111-117) with the same content, and SIM_TESTING.md lines 55-58 repeat the bring-up list a third time. ~40 duplicated lines that will drift again with findings 5/8/10/12.

Evidence detail: Compared README.md 6-13 vs 93-100, 45-51 vs 109-117, and line 70 vs test_launch_import.py LAUNCH_FILES.

Estimated gain: -40 LOC docs, one source of truth

Fix sketch: Keep the 'ROS 2 API' tables, delete the earlier duplicates, and either fix the count string or drop it (the test is the source of truth).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_simulation-10 — gazebo_remote/robot_remote pair duplicates gazebo.launch.py and has no callers

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_remote.launch.py:1` · package jetank_simulation · lens minimality · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: gazebo_remote.launch.py (76 lines) and robot_remote.launch.py (113 lines) re-implement the gz include, camera bridge, robot_state_publisher, spawn and three spawners already in gazebo.launch.py, split across two hosts. A workspace-wide grep shows no launch file, pixi task or script references either file; only their own docstrings, README and test do. They also drift (no sensor bridge, no IGN_GAZEBO_SYSTEM_PLUGIN_PATH, no gripper controllers, no timeout). 189 lines + 2 test cases maintained for an unused workflow.

Evidence detail: grep -rn 'gazebo_remote|robot_remote' over src/ and pixi.toml returned hits only inside jetank_simulation (README, tests, the files themselves).

Estimated gain: -189 LOC launch, -2 test parametrizations, -20 lines README

Fix sketch: Delete both files and their README rows/test entries; if the split-host workflow is ever needed, add a `role:=all|gazebo|robot` argument to gazebo.launch.py that conditions the existing actions instead.

Verifier (confirmed, adjusted low): The fix is in-scope: grep shows no launch file, pixi task, CMake target or other package imports gazebo_remote/robot_remote (only src/jetank_simulation/README.md:12-13,99-100, test/test_launch_import.py:33-34, and the files' own docstrings), so deleting them cannot break a consumer package and touches no perception abstractions. Duplication is real: robot_remote.launch.py:47-94 re-declares robot_state_publisher, spawn and three spawners that gazebo.launch.py:104-201 already has, and it lacks the gripper spawners, sensor bridge and the timeout/event sequencing at gazebo.launch.py:154-265. One correction to the evidence: the finding's "only inside jetank_simulation" claim is incomplete — the workspace-root REMOTE_SIMULATION_GUIDE.md:181-574 documents these two files as the split-host workflow (about 10 references) and plans/2026-05-28-whole-workspace-in-gazebo-plan.md:258 mentions gazebo_remote, so the deletion must also rewrite that guide or the fix_sketch's role:= argument alternative should be used to keep the documented workflow; this raises the effort but not the risk.

### jetank_simulation-17 — ~90 lines of identical physics/plugin/scene/GUI boilerplate copied into each of five worlds

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:32` · package jetank_simulation · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: Lines 5-91 (physics, six system plugins, scene, 55-line <gui> block with WorldControl/WorldStats anchors) are byte-for-byte repeated in simple_test.sdf, obstacle_course.sdf, sock_arena.sdf and house.sdf (only camera_pose and ambient differ). Any tuning (findings 14-16) must be applied five times and already drifts (sock_arena ambient 0.5 vs 0.4). The GUI layout in particular does not belong in the world: gz sim takes a single `--gui-config <file>` and ignores the world <gui> block when given one.

Evidence detail: Diffed the headers of all five worlds by reading them; ~450 of the 1601 world lines are this duplicated header.

Estimated gain: -250 to -450 LOC across worlds; single place to tune physics/rendering

Fix sketch: Move the <gui> block to one worlds/gui.config and pass `--gui-config` in gz_args; optionally author worlds as .sdf.xacro with a shared header macro and expand at build time via a CMake custom command.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## jetank_moveit_config

Coverage: 25 files read; 15 measurements run. Notes: All source, config, build, launch and test files of jetank_moveit_config were read in full; .git/ internals, .pytest_cache metadata and hook samples were listed but not read as they are not package content. build/, install/, log/, .pixi/ were not browsed; the only .pixi access was targeted introspection via pixi-run python/ros2 commands (versions, moveit_configs_utils builder source, plugin XML lookup). No RViz or controller_manager process was running, so RViz-related and spawner-success-path claims are static. The 'strings' check of MoveIt shared libraries produced no output, so the response_adapters / kinematics_solver_attempts dead-parameter claims are based on MoveIt 2.5.9 API knowledge rather than binary confirmation. No planning benchmark was run, so collision-matrix and segment-fraction gains are estimates.

Measurements:
- mcp__ros2-mcp__get_node_list: /move_group, three spawner nodes and /robot_state_publisher present; no /controller_manager node
- mcp__ros2-mcp__get_node_info /move_group: publishers include /robot_description and /robot_description_semantic; action clients remapped to /controller_manager/*
- mcp__ros2-mcp__get_node_params /move_group: full parameter dump (pilz cartesian_limits, S5_joint limits, kinematics_solver_attempts, ompl.response_adapters, 33 ompl.planner_configs.* params, longest_valid_segment_fraction 0.005, publish_robot_description true)
- mcp__ros2-mcp__profile_node /move_group 10 s: CPU mean 1.97% / p95 9.9%, RSS 71.1 MB, 21 threads, 16 fds
- mcp__ros2-mcp__profile_node /arm_controller_spawner 5 s: CPU mean 0.2%, RSS 65.2 MB, 11 threads
- mcp__ros2-mcp__get_topic_hz /joint_states 5 s: 9.95 Hz (50 msgs)
- mcp__ros2-mcp__get_topic_hz /monitored_planning_scene 6 s: 0.0 Hz (0 msgs)
- mcp__ros2-mcp__get_topic_bw /monitored_planning_scene 6 s: 0 B/s
- mcp__ros2-mcp__get_topic_list: full topic inventory (used to confirm /robot_description, action topics)
- ps -o pid,etimes,rss for spawner pids 8286/8294/8296: 1681 s elapsed, RSS 65804/63340/63560 KB; ros2_control_node absent from process list; move_group pid 8298 RSS 69424 KB
- pixi run ros2 pkg xml: moveit_ros_move_group 2.5.9, moveit_configs_utils 2.5.9, controller_manager 2.54.0; inspect.getsource of MoveItConfigsBuilder.to_moveit_configs / planning_pipelines
- pixi run ros2 run controller_manager spawner --help (timeout flag names)
- pixi run: ros2 pkg xml gripper_controllers and grep for position_controllers/GripperActionController plugin owner; ros2 pkg xml ament_lint_common dependency list
- strings scan of libmoveit_planning_pipeline.so / libmoveit_kdl_kinematics_plugin.so for response_adapters / kinematics_solver_attempts returned no output (tool produced nothing, so findings 04/05 rest on version knowledge, marked static)
- python load of moveit_configs_utils default_configs/ompl_defaults.yaml FAILED (file not present in this install); only affects the note that defaults are not merged, which the builder source already shows

Findings: 20 (high 0, medium 1, low 19).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_moveit_config-01 | runtime | Controller spawners idle forever when controller_manager never appears | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:120` | medium | measured | ~190 MB RSS and 3 idle processes freed on failed bringups; visible failure instead of silent hang | S | confirmed |
| jetank_moveit_config-02 | runtime | move_group re-publishes /robot_description that the caller's robot_state_publisher already provides | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:60` | low | measured | one fewer latched 30 KB publisher; consistent with sim path | S | **UNVERIFIED** |
| jetank_moveit_config-03 | minimality | Pilz cartesian limits loaded into move_group although only the OMPL pipeline is enabled | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:63` | low | measured | -9 LOC file, -2 launch lines, 4 fewer params in two nodes | S | **UNVERIFIED** |
| jetank_moveit_config-04 | minimality | response_adapters block is not read by MoveIt 2.5.9 (Humble) | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:15` | low | measured | -4 LOC, one fewer unused parameter | S | **UNVERIFIED** |
| jetank_moveit_config-06 | minimality | joint_limits.yaml carries limits for S5_joint, which is in no planning group, plus default-valued has_jerk_limits lines | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/joint_limits.yaml:27` | low | measured | -11 LOC, 6 fewer parameters | S | **UNVERIFIED** |
| jetank_moveit_config-08 | minimality | Seven OMPL planner configs declared that nothing ever selects | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:29` | low | measured | -55 LOC, ~30 fewer parameters in two nodes | S | **UNVERIFIED** |
| jetank_moveit_config-12 | footprint | package.xml declares dependencies this package never uses and omits ones it does | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/package.xml:14` | low | measured | 2 fewer dependency edges, correct rosdep closure | S | **UNVERIFIED** |
| jetank_moveit_config-18 | minimality | SRDF named poses are mirrored by hand in jetank_manipulation and have already drifted | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/jetank.srdf:40` | low | measured | removes ~30 duplicated lines in the sibling package and a drift source | M | **UNVERIFIED** |
| jetank_moveit_config-05 | minimality | kinematics_solver_attempts is an ignored (MoveIt 1 era) parameter | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/kinematics.yaml:7` | low | static | -1 LOC, one fewer unused parameter | S | **UNVERIFIED** |
| jetank_moveit_config-07 | runtime | longest_valid_segment_fraction 0.005 doubles collision checks per OMPL motion vs the default | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:91` | low | static | ~2x fewer collision checks per planning call for the arm group | S | **UNVERIFIED** |
| jetank_moveit_config-11 | runtime | demo.launch.py runs xacro on the full robot description twice at startup | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:56` | low | static | one fewer xacro process at launch (~0.5-1 s startup on Jetson) | S | **UNVERIFIED** |
| jetank_moveit_config-13 | footprint | ament_lint_common pulls eight linters into colcon test for a config-only package, three of them stubbed out | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/CMakeLists.txt:24` | low | static | 5 fewer test processes per colcon test, smaller test_depend closure, -9 CMake lines | S | **UNVERIFIED** |
| jetank_moveit_config-14 | minimality | Compiler warning flags set in a package with no compiled targets | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/CMakeLists.txt:4` | low | static | -3 LOC | S | **UNVERIFIED** |
| jetank_moveit_config-15 | minimality | Redundant use_ros2_control xacro mapping in the sim builder | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_sim.launch.py:53` | low | static | -1 mapping entry | S | **UNVERIFIED** |
| jetank_moveit_config-16 | minimality | RViz node receives the full URDF as a parameter although its config reads the model from the topic | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:188` | low | static | -1 launch line, one fewer 30 KB parameter | S | **UNVERIFIED** |
| jetank_moveit_config-17 | runtime | RViz config renders TF names for all 20 frames at 30 FPS | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/moveit.rviz:32` | low | static | lower idle RViz CPU/GPU on the Jetson when the planning UI is used locally | S | **UNVERIFIED** |
| jetank_moveit_config-19 | minimality | README documents a 4-joint arm chain to S5_link and a test contract that no longer match the package | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/README.md:5` | low | static | documentation matches code; ~6 lines corrected | S | **UNVERIFIED** |
| jetank_moveit_config-20 | minimality | Test parametrization threads an unused expected_args through the entry-point test | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/test/test_launch_import.py:50` | low | static | -2 LOC, clearer test intent | S | **UNVERIFIED** |
| jetank_moveit_config-09 | runtime | SRDF disable_collisions matrix leaves most static link pairs enabled for self-collision checking | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/jetank.srdf:93` | low (orig medium) | static | roughly 3-5x fewer self-collision pairs per state check (120 -> ~25-40) | M | confirmed |
| jetank_moveit_config-10 | minimality | MoveItConfigs builder, ParameterValue wrapping, RViz node and rviz_config argument duplicated across two launch files | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_sim.launch.py:41` | low | static | ~-60 LOC, single point of change for MoveIt config | M | **UNVERIFIED** |

### jetank_moveit_config-01 — Controller spawners idle forever when controller_manager never appears

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:120` · package jetank_moveit_config · lens runtime · severity medium · evidence measured · effort S · verdict confirmed

Description: The three spawner Nodes are launched unconditionally alongside ros2_control_node with no --controller-manager-timeout and no event handler tying them to the controller_manager process. On the live stack (unified.launch.py hardware:=serial enable_moveit:=true) no /controller_manager node exists, yet all three spawner python processes are still alive after 1681 s, each holding ~64 MB RSS and 11 threads, spinning on a service-wait loop. That is ~190 MB of RAM and three python interpreters doing nothing on an 8 GB Jetson, and the failure is silent.

Evidence detail: get_node_list: /joint_state_broadcaster_spawner, /arm_controller_spawner, /gripper_controller_spawner present, no /controller_manager node. profile_node /arm_controller_spawner: rss mean 65,220,608 B, 11 threads, 0.2% CPU. ps -o etimes,rss for pids 8286/8294/8296: 1681 s elapsed, RSS 65804/63340/63560 KB. ros2_control_node absent from ps.

Estimated gain: ~190 MB RSS and 3 idle processes freed on failed bringups; visible failure instead of silent hang

Fix sketch: Add '--controller-manager-timeout', '30' (and '--service-call-timeout') to the spawner arguments, and/or wrap the spawners in RegisterEventHandler(OnProcessStart(target_action=ros2_control_node, ...)) so they only start once the CM process is up and exit on its death.

Verifier (confirmed, adjusted medium): moveit_bringup.launch.py:120-133 builds the three spawner Nodes with only `--controller-manager` and optional `--param-file`; no `--controller-manager-timeout` and no event handler, and they are returned unconditionally at line 198-203 alongside ros2_control_node. Live check this session: get_node_list shows /joint_state_broadcaster_spawner, /arm_controller_spawner, /gripper_controller_spawner but no /controller_manager; `ps` shows pids 8286/8294/8296 alive for 24978 s with RSS 59092/56668/56860 KB, 11 threads each, 0.2% CPU, and no ros2_control_node process. By contrast jetank_simulation/launch/gazebo.launch.py:159 already passes `--controller-manager-timeout 60` and uses OnProcessExit handlers, so the fix pattern exists in-repo. Severity stays medium: ~170 MB RSS wasted and a silent bringup failure, but negligible CPU.

### jetank_moveit_config-02 — move_group re-publishes /robot_description that the caller's robot_state_publisher already provides

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:60` · package jetank_moveit_config · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: planning_scene_monitor(publish_robot_description=True) makes move_group a second latched publisher of the ~30 KB URDF string on /robot_description, while the module docstring (lines 14-15) states the caller always runs robot_state_publisher. moveit_sim.launch.py already sets this False for exactly that reason (line 62). The bringup path keeps a duplicate transient-local publisher and every late-joining subscriber (RViz RobotModel, web UI) receives the string twice.

Evidence detail: get_node_params /move_group: publish_robot_description = true. get_node_info /move_group lists a /robot_description (std_msgs/String) publisher while /robot_state_publisher is also running (get_node_list). Duplicate delivery to subscribers is static reasoning.

Estimated gain: one fewer latched 30 KB publisher; consistent with sim path

Fix sketch: Set publish_robot_description=False in _build_moveit_configs (matching moveit_sim.launch.py:62); keep publish_robot_description_semantic=True since nobody else publishes the SRDF.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **cross-11** — same publish_robot_description=True issue; cross-11 adds the live get_topic_hz(/robot_description) count=2 evidence.

### jetank_moveit_config-03 — Pilz cartesian limits loaded into move_group although only the OMPL pipeline is enabled

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:63` · package jetank_moveit_config · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Both launch files call .pilz_cartesian_limits('config/pilz_cartesian_limits.yaml') while .planning_pipelines(pipelines=['ompl']) restricts planning to OMPL, and moveit_planners_pilz is not a dependency. moveit_configs_utils.to_moveit_configs only auto-loads this file when the pilz pipeline is active, so the explicit call is the only reason four robot_description_planning.cartesian_limits.* parameters are pushed to move_group (and to RViz via joint_limits). The 9-line YAML file, its install, and the two builder calls are dead weight.

Evidence detail: get_node_params /move_group shows robot_description_planning.cartesian_limits.max_trans_vel/acc/dec/max_rot_vel present while planning_pipelines = ['ompl'] only. moveit_configs_utils source (inspected via pixi python) confirms pilz limits are auto-loaded only if 'pilz_industrial_motion_planner' is in planning_pipelines.

Estimated gain: -9 LOC file, -2 launch lines, 4 fewer params in two nodes

Fix sketch: Delete config/pilz_cartesian_limits.yaml and remove the .pilz_cartesian_limits(...) call from moveit_bringup.launch.py:63 and moveit_sim.launch.py:65 (or add the pilz pipeline if LIN/CIRC is actually wanted).

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-33** — same pilz_cartesian_limits call; footprint-33 additionally notes config/joint_limits.yaml, kinematics.yaml, ompl_planning.yaml and pilz_cartesian_limits.yaml are mode 0600 in the source tree (symlink-install exposes that to other users) and asks for chmod 644.

### jetank_moveit_config-04 — response_adapters block is not read by MoveIt 2.5.9 (Humble)

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:15` · package jetank_moveit_config · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: The request/response adapter split (default_planner_response_adapters/*) was introduced in MoveIt 2.10; the installed MoveIt is 2.5.9 where PlanningPipeline reads only planning_plugin and request_adapters. Lines 15-18 are therefore dead config that is nevertheless serialized into move_group as the ompl.response_adapters string parameter. AddTimeOptimalParameterization is already in request_adapters (line 8), so nothing is lost by removing the block.

Evidence detail: ros2 pkg xml moveit_ros_move_group -> version 2.5.9. get_node_params /move_group shows ompl.response_adapters set but the Humble pipeline API has no such parameter (static reasoning from MoveIt version history; strings scan of libmoveit_planning_pipeline.so returned nothing, so not confirmed from the binary).

Estimated gain: -4 LOC, one fewer unused parameter

Fix sketch: Delete the response_adapters key (lines 15-18). If MoveIt is later upgraded to 2.10+, rename the request adapters to the new names at that time.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-06 — joint_limits.yaml carries limits for S5_joint, which is in no planning group, plus default-valued has_jerk_limits lines

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/joint_limits.yaml:27` · package jetank_moveit_config · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: The SRDF arm chain ends at S4_link (S5 intentionally excluded, jetank.srdf:14-16), and the gripper group is gripper_left_joint only. The S5_joint block (lines 27-32) is therefore never consulted by time parameterization, and the five has_jerk_limits: false lines restate the default. 11 lines and 6 params that do nothing.

Evidence detail: get_node_params /move_group: robot_description_planning.joint_limits.S5_joint.* present; robot_description_semantic shows arm chain base_link->S4_link and gripper = gripper_left_joint, so S5_joint belongs to no group.

Estimated gain: -11 LOC, 6 fewer parameters

Fix sketch: Delete the S5_joint block and the has_jerk_limits: false lines (default is false).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-08 — Seven OMPL planner configs declared that nothing ever selects

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:29` · package jetank_moveit_config · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Only RRTConnect is the default for both groups and the sole planner_id the grasp_server (jetank_manipulation) requests. RRT, RRTstar, EST, LBKPIECE, KPIECE, BiTRRT and FMT (lines 29-76 and list entries 83-89, 97) are never used; they add ~55 lines and ~30 ompl.planner_configs.* parameters to move_group and RViz. The projection_evaluator lines (90, 98) are only meaningful for the KPIECE/EST family and become dead once those go.

Evidence detail: get_node_params /move_group shows 8 arm planner_configs and 33 ompl.planner_configs.* parameters loaded; grasp_server.py:54 PLANNER_ID = "RRTConnect"; ompl_defaults are NOT merged by moveit_configs_utils when planner_configs is present (source inspected).

Estimated gain: -55 LOC, ~30 fewer parameters in two nodes

Fix sketch: Keep only the RRTConnect planner_config and list it for both groups; drop projection_evaluator lines. Re-add specific planners only when an experiment needs them.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-12 — package.xml declares dependencies this package never uses and omits ones it does

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/package.xml:14` · package jetank_moveit_config · lens footprint · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Unused: moveit_ros_planning_interface (no MoveGroupInterface client code here) and position_controllers (the gripper plugin position_controllers/GripperActionController is exported by the gripper_controllers package, and jetank_controllers.yaml uses no JointGroupPositionController). Missing: moveit_configs_utils (imported at moveit_bringup.launch.py:42 / moveit_sim.launch.py:43), gripper_controllers (actual owner of the gripper plugin), jetank_simulation (included by moveit_sim.launch.py:80), and launch/launch_ros/ament_index_python. The stray deps inflate rosdep/pixi resolution while the missing ones make the package non-installable standalone.

Evidence detail: pixi shell: ros2 pkg xml gripper_controllers exists and grep finds position_controllers/GripperActionController only in share/gripper_controllers/ros_control_plugins.xml. Unused/missing status of the others from reading package.xml, all three launch files and jetank_controllers.yaml.

Estimated gain: 2 fewer dependency edges, correct rosdep closure

Fix sketch: Remove moveit_ros_planning_interface and position_controllers; add exec_depend on moveit_configs_utils, gripper_controllers, jetank_simulation, launch, launch_ros, ament_index_python.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-18 — SRDF named poses are mirrored by hand in jetank_manipulation and have already drifted

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/jetank.srdf:40` · package jetank_moveit_config · lens minimality · severity low · evidence measured · effort M · verdict **UNVERIFIED**

Description: grasp_server.py (_SRDF_STATES, jetank_manipulation) re-declares home/ready/grasp_* joint values 'as a mirror of jetank.srdf' and still includes S5_joint, which the SRDF arm group no longer contains. Two sources of truth for the same numbers; move_group already publishes /robot_description_semantic (publish_robot_description_semantic=True) so clients can read the group_states from there.

Evidence detail: grasp_server.py:318-336 shows the mirror with S5_joint; get_node_params /move_group robot_description_semantic shows 3-joint arm group_states and publish_robot_description_semantic = true.

Estimated gain: removes ~30 duplicated lines in the sibling package and a drift source

Fix sketch: Have grasp_server parse group_state elements from the /robot_description_semantic topic (or MoveIt's get_named_target_values via moveit_py) and delete _SRDF_STATES.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-05 — kinematics_solver_attempts is an ignored (MoveIt 1 era) parameter

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/kinematics.yaml:7` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: MoveIt 2's KinematicsBase reads kinematics_solver_timeout and kinematics_solver_search_resolution; kinematics_solver_attempts was removed before MoveIt 2 existed (attempts are now driven by the timeout). The line only adds an unused parameter to move_group and RViz.

Evidence detail: get_node_params /move_group shows robot_description_kinematics.arm.kinematics_solver_attempts = 3 declared, but the KDL plugin API in MoveIt 2.5 has no such option (based on MoveIt 2 source knowledge; not verified from the binary).

Estimated gain: -1 LOC, one fewer unused parameter

Fix sketch: Remove line 7.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-07 — longest_valid_segment_fraction 0.005 doubles collision checks per OMPL motion vs the default

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:91` · package jetank_moveit_config · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: OMPL validates each candidate motion by sampling states every longest_valid_segment_fraction of the joint-space extent. 0.005 (vs MoveIt default 0.01) means 2x as many state validity/collision checks for every RRTConnect extension on the arm group. For a 3-DOF arm made of box primitives with ~4.7 rad joint ranges the default step already resolves ~2.7 deg, so the finer step costs planning CPU on the Jetson without a safety benefit at these link sizes.

Evidence detail: get_node_params /move_group confirms ompl.arm.longest_valid_segment_fraction = 0.005 is active. Cost scaling is static reasoning from OMPL's DiscreteMotionValidator; no planning benchmark was run.

Estimated gain: ~2x fewer collision checks per planning call for the arm group

Fix sketch: Raise to 0.01 (or drop the key to use the default) and verify in sim that planned paths still clear the chassis/camera.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-11 — demo.launch.py runs xacro on the full robot description twice at startup

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:56` · package jetank_moveit_config · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: demo.launch.py expands jetank_ros2_control.urdf.xacro via Command(['xacro', ...]) for robot_state_publisher, then includes moveit_bringup.launch.py whose MoveItConfigsBuilder.robot_description() runs xacro again on the same file with the same mapping. On the Jetson each xacro pass over the multi-file JeTank model costs several hundred ms of Python and is pure startup latency.

Evidence detail: Both xacro invocations are visible in the launch sources read this session (demo.launch.py:56 and moveit_bringup.launch.py:51-54); the per-run xacro time was not measured.

Estimated gain: one fewer xacro process at launch (~0.5-1 s startup on Jetson)

Fix sketch: Add a 'start_robot_state_publisher' arg to moveit_bringup.launch.py that launches RSP with moveit_config.robot_description (already expanded), and let demo.launch.py set it true instead of running its own Command(xacro).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-13 — ament_lint_common pulls eight linters into colcon test for a config-only package, three of them stubbed out

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/CMakeLists.txt:24` · package jetank_moveit_config · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: ament_lint_auto + ament_lint_common register copyright, cppcheck, cpplint, flake8, lint_cmake, pep257, uncrustify and xmllint. CMakeLists already neutralises three via set(<pkg>_FOUND TRUE) hacks (lines 26-31); cppcheck, uncrustify and lint_cmake still run on a package with no C++ or meaningful CMake. Each is a separate test process on the Jetson during colcon test, and the whole linter family is a test-time dependency to install.

Evidence detail: ros2 pkg xml ament_lint_common lists exec_depends on the eight ament_cmake_* linters; CMakeLists.txt lines 23-32 read this session.

Estimated gain: 5 fewer test processes per colcon test, smaller test_depend closure, -9 CMake lines

Fix sketch: Replace ament_lint_auto/ament_lint_common with explicit test_depend ament_cmake_flake8 + ament_cmake_pep257 (or only the pytest test) and drop the *_FOUND TRUE workarounds.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-14 — Compiler warning flags set in a package with no compiled targets

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/CMakeLists.txt:4` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: add_compile_options(-Wall -Wextra -Wpedantic) guarded by a GNU/Clang check is boilerplate for C++ packages; this package only installs directories. Three dead lines.

Evidence detail: CMakeLists.txt read in full; no add_executable/add_library present.

Estimated gain: -3 LOC

Fix sketch: Delete lines 4-6.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-15 — Redundant use_ros2_control xacro mapping in the sim builder

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_sim.launch.py:53` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: mappings passes 'use_ros2_control': 'true' but jetank_ros2_control.urdf.xacro declares that arg with default "true" (line 8 of the xacro). The mapping changes nothing.

Evidence detail: grep of jetank_description/urdf/jetank_ros2_control.urdf.xacro shows <xacro:arg name="use_ros2_control" default="true"/>.

Estimated gain: -1 mapping entry

Fix sketch: Reduce mappings to {'use_sim': 'true'}.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-16 — RViz node receives the full URDF as a parameter although its config reads the model from the topic

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_bringup.launch.py:188` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: moveit.rviz configures RobotModel with 'Description Source: Topic' (/robot_description), and the MotionPlanning plugin's RDFLoader also falls back to the topic. Passing moveit_config.robot_description (~30 KB string) as an rviz2 parameter is a redundant copy in the launch param file and the node's parameter server. The sim launch omits it already (moveit_sim.launch.py:110-116).

Evidence detail: moveit.rviz lines 23-25 and moveit_bringup.launch.py:188 read this session; RViz was not running during measurement.

Estimated gain: -1 launch line, one fewer 30 KB parameter

Fix sketch: Drop moveit_config.robot_description from the rviz2 parameters list, matching moveit_sim.launch.py.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-17 — RViz config renders TF names for all 20 frames at 30 FPS

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/moveit.rviz:32` · package jetank_moveit_config · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The TF display with Show Names: true draws a text billboard per frame (20 links including four optical/camera frames and wheels) plus axes, and Global Options Frame Rate is 30. When RViz runs on the Jetson itself (demo.launch.py defaults use_rviz:=true) this is continuous GPU/CPU load unrelated to planning; text rendering is one of RViz's more expensive per-frame items.

Evidence detail: moveit.rviz lines 32-37 and 82 read this session; RViz was not running, so no GPU/CPU measurement.

Estimated gain: lower idle RViz CPU/GPU on the Jetson when the planning UI is used locally

Fix sketch: Set TF 'Show Names: false' (or disable the TF display; RobotModel already shows the links) and Frame Rate: 15.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-19 — README documents a 4-joint arm chain to S5_link and a test contract that no longer match the package

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/README.md:5` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: README says the arm group is base_link->S5_link with S1,S2,S3,S5 (lines 5-7, 72, 75) while jetank.srdf:18 chains to S4_link with three joints and the EE parent is S4_link. The Tests table (line 121) says bringup declares {use_sim_time, use_rviz, hardware} but test_launch_import.py:36 expects rviz_config too, and the args table (line 110) lists rviz_config as moveit_sim-only. Stale prose costs reader time and hides the actual DOF count.

Evidence detail: README.md, jetank.srdf and test/test_launch_import.py read this session.

Estimated gain: documentation matches code; ~6 lines corrected

Fix sketch: Update the planning-group rows to base_link->S4_link (S1,S2,S3), EE parent S4_link, and list rviz_config for moveit_bringup in both tables.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-20 — Test parametrization threads an unused expected_args through the entry-point test

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/test/test_launch_import.py:50` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: test_launch_module_exposes_entry_point takes expected_args from the parametrize tuple but never uses it; the two tests could also share one module load. Minor, but it is dead parameter plumbing in the only test file.

Evidence detail: test_launch_import.py lines 50-54 read this session.

Estimated gain: -2 LOC, clearer test intent

Fix sketch: Parametrize the first test over (filename, module_name) only, or fold the entry-point assertion into test_generate_launch_description.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_moveit_config-09 — SRDF disable_collisions matrix leaves most static link pairs enabled for self-collision checking

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/jetank.srdf:93` · package jetank_moveit_config · lens runtime · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: The URDF loaded into move_group has 16 links with collision geometry (chassis, arm_bearing, S1-S5, gripper_base/left/right, camera_link, laser, 4 wheels) = 120 link pairs, but only 19 pairs are disabled. Pairs that can never move relative to each other (chassis vs laser, chassis vs each wheel, wheel vs wheel, laser vs wheels, laser vs arm_bearing, camera vs laser, gripper_base vs S5 fingers, etc.) and pairs that can never reach each other (wheels vs any arm/gripper link) are still evaluated by FCL on every state validity check during planning and every checkStateValidity call. On a Jetson this is wasted CPU in the hottest MoveIt loop.

Evidence detail: Link inventory taken from the live robot_description parameter (get_node_params /move_group) and the 19 disable_collisions entries in the SRDF; no per-pair timing was measured.

Estimated gain: roughly 3-5x fewer self-collision pairs per state check (120 -> ~25-40)

Fix sketch: Run the MoveIt Setup Assistant collision-matrix sampler (or moveit_setup_srdf_plugins compute_default_collisions) against the current URDF and paste the generated Default/Never/Adjacent pairs; at minimum add chassis<->{laser, 4 wheels}, laser<->wheels, wheel<->wheel, and wheels/laser<->all arm & gripper links as Never.

Verifier (confirmed, adjusted low): The SRDF (jetank.srdf:93-117) has exactly 19 disable_collisions entries while move_group loads jetank_ros2_control.urdf.xacro (moveit_bringup.launch.py:45-55), which composes arm, gripper, camera, 4 wheels and lidar with collision geometry (wheels.xacro:39, lidar.xacro:18), so chassis<->laser/wheels, wheel<->wheel and laser<->wheels are indeed never disabled despite being rigidly fixed (laser_joint fixed at lidar.xacro:31; wheels are continuous cylinders about their own axle, so chassis distance is invariant). The only SRDF consumers are the two moveit launch files; jetank_manipulation only mirrors group_state joint values (grasp_server.py:144,318; grasp_poses.yaml:3; test_import.py:169), so adding disable_collisions pairs breaks no consumer and touches no package boundary or perception abstraction. One caveat on the fix_sketch: hand-writing wheels/laser<->arm/gripper as "Never" without running the sampler could suppress a real collision (arm reaches ~0.15 m above floor, near the wheel envelope), so only the sampler-generated pairs should be pasted; also FCL broadphase already culls far-apart pairs, so the 3-5x gain estimate is likely overstated, hence severity lowered.

### jetank_moveit_config-10 — MoveItConfigs builder, ParameterValue wrapping, RViz node and rviz_config argument duplicated across two launch files

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/moveit_sim.launch.py:41` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: moveit_sim.launch.py:41-67, 88-93, 103-118, 144-151 are near-verbatim copies of moveit_bringup.launch.py:30-65, 135-140, 180-196, 224-231. The only real differences are the xacro mappings and publish_robot_description. About 60 duplicated lines that must be edited in lockstep (they already diverged: bringup wraps XML params with an isinstance check on moveit_params, sim uses params.get).

Evidence detail: Line-by-line comparison of the two launch files read this session.

Estimated gain: ~-60 LOC, single point of change for MoveIt config

Fix sketch: Add launch/_moveit_common.py with build_moveit_configs(mappings, publish_robot_description), wrap_xml_params(params) and make_rviz_node(moveit_config, use_sim_time, ...), loaded by both launch files via importlib.util.spec_from_file_location (or install a small python module and import it).

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **cross-36** — same moveit_bringup/moveit_sim duplication; cross-36 proposes a launch/jetank_moveit_configs.py helper.

## jetank_description

Coverage: 19 files read; 12 measurements run. Notes: All 14 git-tracked files of the package were read in full, plus LICENSE/.gitignore. Skipped per constraints: .git/, .pytest_cache/, test/__pycache__/. The directories config/, meshes/, src/ and include/jetank_description/ exist but are empty (no files to read). The package contains no C++, no Python module beyond the launch file and test, no YAML, no JS/HTML, no rviz config. Sibling files listed under files_read were read only to attribute consumers of this package (who launches joint_state_publisher, who re-expands the xacro, what ros2_control.xacro contributes). No Gazebo simulation was running, so findings -27/-28/-29 (sim-only sensor settings) are static. /tf rate could not be attributed to RSP alone because /robot_controller also publishes /tf and no per-publisher tool is available. No measurement tool failed.

Measurements:
- mcp__ros2-mcp__get_node_list: 16 nodes incl. /robot_state_publisher, /joint_state_publisher, /robot_controller; no /controller_manager present
- mcp__ros2-mcp__get_node_info /robot_state_publisher, /joint_state_publisher, /robot_controller (the latter also publishes /tf)
- mcp__ros2-mcp__get_node_params /robot_state_publisher (publish_frequency 20, full robot_description string, no <ros2_control> element) and /joint_state_publisher (rate 10, source_list [], use_mimic_tags true)
- mcp__ros2-mcp__get_topic_list (49 topics)
- mcp__ros2-mcp__profile_node /robot_state_publisher 10 s: cpu mean 0.59% p95 9.7%, RSS 29.2 MB, 11 threads, 16 fds
- mcp__ros2-mcp__profile_node /joint_state_publisher 10 s: cpu mean 1.86% p95 9.8%, RSS 69.7 MB, 11 threads, 16 fds
- mcp__ros2-mcp__measure_topic_perf /tf 10 s: 40.27 Hz, 34,578 B/s, jitter 11 ms, 0 drops (shared with /robot_controller odom TF)
- mcp__ros2-mcp__measure_topic_perf /joint_states 10 s: 9.999 Hz, 3,809 B/s, latency p50 2.2 ms p99 3.2 ms
- mcp__ros2-mcp__read_topic /tf_static: 2 msgs, 12 static transforms from RSP + world->base_footprint from static publisher
- mcp__ros2-mcp__read_topic /joint_states: 10 joint names (4 arm, 2 gripper, 4 wheels), all positions 0.0
- Bash (pixi run python3, in-process xacro.process_file): expansion 0.128 s; 21,566 B total, 15,849 B without comments (5,717 B comments); 15 Gazebo/* material tags, 9 material-only gazebo blocks; 23 links, 22 joints, 12 fixed; with ros2_control=true 28,447 B in 0.070 s
- Bash (time pixi run xacro <file> use_ros2_control:=false): real 1.703 s, user 1.261 s, sys 0.324 s

Findings: 30 (high 0, medium 0, low 30).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| jetank_description-03 | footprint | exec_depend joint_state_publisher not launched here; live JSP publishes constant zeros at 10 Hz | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:20` | low (orig medium) | measured | ~70 MB RSS + ~2% CPU on the Orin when JSP is dropped from hardware bringup; removes one exec_depend here | S | confirmed |
| jetank_description-11 | runtime | 26% of the published /robot_description string is XML comments | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:78` | low | measured | -5.7 KB per copy of robot_description (x ~6 consumers), ~60 LOC of prose moved out of the model | S | **UNVERIFIED** |
| jetank_description-20 | footprint | Default use_ros2_control=true couples the description package to jetank_motor_control | `/home/koen/workspaces/ros2_ws/src/jetank_description/launch/robot_description.launch.py:59` | low | measured | Removes one cross-package exec_depend from the description package; -6.9 KB robot_description for viz-only bringups | S | **UNVERIFIED** |
| jetank_description-21 | runtime | Four continuous wheel joints published as dynamic TF on hardware that has no wheel encoders | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/wheels.xacro:24` | low | measured | 4 fewer dynamic transforms per RSP /tf message (~40% of its payload) on hardware | S | **UNVERIFIED** |
| jetank_description-25 | minimality | Both tests expand the full model separately and re-parse XML twice | `/home/koen/workspaces/ros2_ws/src/jetank_description/test/test_urdf.py:60` | low | measured | ~50% test runtime, -10 LOC | S | **UNVERIFIED** |
| jetank_description-19 | runtime | xacro run as a subprocess costs ~1.7 s at launch vs 0.13 s in-process; siblings re-expand it again | `/home/koen/workspaces/ros2_ws/src/jetank_description/launch/robot_description.launch.py:38` | low (orig medium) | measured | ~1.5 s per bringup per expansion on the Orin Nano | M | confirmed |
| jetank_description-01 | footprint | exec_depend rviz2 is unused by this package | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:28` | low (orig medium) | static | Removes a ~100+ MB GUI dependency chain from this package's rosdep closure on headless Jetson installs | S | confirmed |
| jetank_description-02 | footprint | exec_depend joint_state_publisher_gui is unused (Qt dependency) | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:21` | low (orig medium) | static | Drops PyQt5 + joint_state_publisher_gui from this package's install/rosdep closure | S | confirmed |
| jetank_description-04 | footprint | exec_depend urdf is not used directly | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:18` | low | static | -1 dependency line | S | **UNVERIFIED** |
| jetank_description-05 | footprint | ament_lint_auto/ament_lint_common pull ~7 linter runners into a package with no C++ | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:30` | low (orig medium) | static | Removes 2 test deps and ~6 linter processes per colcon test; -8 LOC of template boilerplate in CMakeLists | S | confirmed |
| jetank_description-06 | footprint | test_depend xacro duplicates exec_depend xacro | `/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:33` | low | static | -1 LOC | S | **UNVERIFIED** |
| jetank_description-07 | footprint | add_compile_options for a package with no compiled targets | `/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:4` | low | static | -3 LOC | S | **UNVERIFIED** |
| jetank_description-08 | footprint | Empty template dirs (meshes/, config/, src/, include/) installed or left on disk | `/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:13` | low | static | -8 LOC in CMake, 4 dirs removed, 2 empty install dirs gone | S | **UNVERIFIED** |
| jetank_description-09 | minimality | Sibling launch references non-existent jetank_description/rviz/urdf.rviz | `/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:13` | low | static | Removes a broken launch path (or ~30 LOC in the sibling) | S | **UNVERIFIED** |
| jetank_description-10 | minimality | CMake template boilerplate comments and lint overrides | `/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:24` | low | static | -7 LOC | S | **UNVERIFIED** |
| jetank_description-12 | minimality | Classic-Gazebo <material>Gazebo/*</material> blocks are dead under Ignition Fortress | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:203` | low | static | ~27 LOC across 5 files; 9 fewer elements in every robot_description | S | **UNVERIFIED** |
| jetank_description-13 | minimality | wheel_inertia macro duplicates cylinder_inertia | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/wheels.xacro:14` | low | static | -8 LOC | S | **UNVERIFIED** |
| jetank_description-14 | minimality | Hand-typed inertia tensors where box/cylinder_inertia macros exist | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:107` | low | static | ~40 LOC across 5 files | S | **UNVERIFIED** |
| jetank_description-15 | minimality | Left/right stereo camera sensor blocks are verbatim duplicates | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:85` | low | static | ~45 LOC | S | **UNVERIFIED** |
| jetank_description-16 | minimality | Right gripper finger link and gazebo block duplicate the left | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:103` | low | static | ~30 LOC | S | **UNVERIFIED** |
| jetank_description-17 | minimality | use_ros2_control condition evaluated twice | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:25` | low | static | -3 LOC | S | **UNVERIFIED** |
| jetank_description-18 | minimality | $(find jetank_description) used for the package's own component includes | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:17` | low | static | 6 ament lookups per expansion removed; test becomes source-consistent | S | **UNVERIFIED** |
| jetank_description-22 | minimality | imu_link is an identity-offset link with a 3 mm visual and inertial | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/imu.xacro:13` | low | static | -15 LOC, one fewer material/visual in every consumer | S | **UNVERIFIED** |
| jetank_description-26 | minimality | README contains stale/contradictory facts about the model | `/home/koen/workspaces/ros2_ws/src/jetank_description/README.md:29` | low | static | Correct docs; ~5 lines fixed/removed | S | **UNVERIFIED** |
| jetank_description-27 | runtime | gpu_lidar <visualize>true</visualize> renders 360 rays per scan in the sim GUI | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/lidar.xacro:46` | low | static | Fewer GUI draw calls in simulation | S | **UNVERIFIED** |
| jetank_description-28 | runtime | Gaussian image noise on both sim cameras adds a per-pixel pass at 640x360x30 fps x2 | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:101` | low | static | Removes one full-frame shader pass per camera frame in simulation | S | **UNVERIFIED** |
| jetank_description-29 | minimality | Custom lens function on the stereo cameras is unexplained and possibly non-pinhole | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:106` | low | static | -20 LOC if dead; removes a projection-model mismatch risk | S | **UNVERIFIED** |
| jetank_description-30 | minimality | 14-line historical ORIGINAL_NOTE comment duplicates ros2_control.xacro prose | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:78` | low | static | -14 LOC and ~900 B of robot_description | S | **UNVERIFIED** |
| jetank_description-23 | minimality | base_link is an empty pass-through link between base_footprint and chassis | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:39` | low | static | -1 link/-1 joint/-1 static TF, ~8 LOC | M | **UNVERIFIED** |
| jetank_description-24 | minimality | S4_link is a separate fixed link that could be a second visual on S3_link | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:157` | low | static | -1 link/-1 joint/-1 static TF, ~20 LOC | M | **UNVERIFIED** |

### jetank_description-03 — exec_depend joint_state_publisher not launched here; live JSP publishes constant zeros at 10 Hz

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:20` · package jetank_description · lens footprint · severity low (orig medium) · evidence measured · effort S · verdict confirmed

Description: This package never starts joint_state_publisher (robot_description.launch.py only starts robot_state_publisher). It is started by jetank_ros_main/launch/urdf.launch.py:50-55, which declares its own dependency. On the live stack the JSP process costs 69.7 MB RSS and ~1.9% CPU to publish a JointState of 10 joints that are all permanently 0.0 (no source_list, no GUI), and it will double-publish /joint_states once ros2_control's joint_state_broadcaster is up. The dependency is dead weight here and the node is dead weight on hardware.

Evidence detail: profile_node /joint_state_publisher (10 s): cpu mean 1.86%, p95 9.8%, RSS 69,687,296 B, 11 threads. measure_topic_perf /joint_states: 9.999 Hz, 3,809 B/s. read_topic /joint_states: 10 names, all positions 0.0, velocity/effort empty. get_node_params: source_list=[], rate=10. get_node_list shows no /controller_manager, so JSP is currently the only /joint_states source.

Estimated gain: ~70 MB RSS + ~2% CPU on the Orin when JSP is dropped from hardware bringup; removes one exec_depend here

Fix sketch: Remove line 20 from this package.xml. In jetank_ros_main/urdf.launch.py gate the JSP node behind a launch arg (e.g. use_jsp:=false by default on hardware) so joint_state_broadcaster is the single /joint_states source.

Verifier (confirmed, adjusted low): jetank_description/package.xml:20 declares exec_depend joint_state_publisher but jetank_description/launch/robot_description.launch.py:73-75 only starts robot_state_publisher; the JSP node is started by jetank_ros_main/launch/urdf.launch.py:50-55, and jetank_ros_main/package.xml:39 already declares its own dependency, so the dep is dead weight here. Re-measured live this session: profile_node /joint_state_publisher (5 s) cpu mean 1.77%, p95 9.86%, RSS 62.3 MB, 11 threads, and get_node_list shows no /controller_manager (only stalled *_spawner nodes), so JSP is currently the sole /joint_states source. The gain is plausible but modest on a 6-core/8 GB Orin (~62-70 MB is <1% of RAM, ~1.8% of one core ≈ 0.3% aggregate), and the double-publish is prospective, not observed; severity is more honestly low.

Duplicate folded in: **footprint-26** — same unused joint_state_publisher exec_depend at package.xml:20; footprint-26 also covers joint_state_publisher_gui (jetank_description-02) and rviz2 (jetank_description-01).

### jetank_description-11 — 26% of the published /robot_description string is XML comments

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:78` · package jetank_description · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: xacro preserves XML comments, so every explanatory paragraph in the xacro sources (gripper.xacro:78-91 14-line ORIGINAL_NOTE, arm.xacro:81-83/117-119/152-156, camera.xacro:41-43/58-60/80-84/117, lidar.xacro:37-39/63-65, imu.xacro:4-9/37-38) ends up in the robot_description parameter and the transient_local /robot_description topic. Every consumer (robot_state_publisher, joint_state_publisher, both move_group nodes, MoveIt planning-scene monitors, RViz) receives, stores and re-parses it with urdfdom. The live parameter value confirms all comments are present.

Evidence detail: In-process xacro.process_file(use_ros2_control=false): 21,566 B total, 15,849 B without comments -> 5,717 B (26.5%) comments. get_node_params /robot_state_publisher shows the comments verbatim in the live robot_description value.

Estimated gain: -5.7 KB per copy of robot_description (x ~6 consumers), ~60 LOC of prose moved out of the model

Fix sketch: Move change-history prose (ORIGINAL_NOTE, 'was 0.04', 'raised from 0.05') to README/commit history; keep one-line comments in the xacro. Alternatively post-process with a comment-stripping xacro wrapper is not worth it — just trim.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-20 — Default use_ros2_control=true couples the description package to jetank_motor_control

`/home/koen/workspaces/ros2_ws/src/jetank_description/launch/robot_description.launch.py:59` · package jetank_description · lens footprint · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: The launch default (line 59-61) and xacro default (jetank_ros2_control.urdf.xacro:8) include $(find jetank_motor_control)/config/ros2_control.xacro, which forces <exec_depend>jetank_motor_control</exec_depend> (package.xml:17) on a leaf description package and adds 6.9 KB to robot_description. The only live consumer (jetank_ros_main/urdf.launch.py:35) overrides it to false, and the MoveIt launch expands its own copy. Meanwhile jetank_motor_control/launch/test_urdf.launch.py consumes jetank_description without declaring it — the dependency direction is inverted.

Evidence detail: Expansion sizes: 28,447 B with ros2_control vs 21,566 B without. Live robot_description (get_node_params) contains no <ros2_control> element, i.e. launched with use_ros2_control=false.

Estimated gain: Removes one cross-package exec_depend from the description package; -6.9 KB robot_description for viz-only bringups

Fix sketch: Default use_ros2_control to false in both launch and xacro; have the ros2_control-needing launches (moveit, sim, hardware) pass true explicitly. Consider moving ros2_control.xacro into this package's urdf/ so the description owns the whole model and motor_control depends on description, not vice versa.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-21 — Four continuous wheel joints published as dynamic TF on hardware that has no wheel encoders

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/wheels.xacro:24` · package jetank_description · lens runtime · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: Wheel joints are type=continuous (line 24). ros2_control.xacro:183 states wheels are 'Simulation only - hardware uses robot_controller.cpp', so on hardware nobody measures them; the live /joint_states carries all four at constant 0.0 and robot_state_publisher recomputes and republishes four wheel transforms in every /tf message. Making them fixed when use_sim=false moves them to tf_static (published once) and shrinks every /tf message.

Evidence detail: read_topic /joint_states: front/rear_left/right_wheel_joint present, positions 0.0. measure_topic_perf /tf: 40.3 Hz, 34,578 B/s total (shared with /robot_controller odom TF; RSP's share not separable with the available tools). read_topic /tf_static: 12 static transforms, none for wheels.

Estimated gain: 4 fewer dynamic transforms per RSP /tf message (~40% of its payload) on hardware

Fix sketch: Add xacro arg wheel_joint_type (continuous when use_sim, fixed otherwise) to the wheel macro; joint_state_publisher/JSB then stop listing wheels too.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-25 — Both tests expand the full model separately and re-parse XML twice

`/home/koen/workspaces/ros2_ws/src/jetank_description/test/test_urdf.py:60` · package jetank_description · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: _process() (lines 20-29) already calls ET.fromstring and checks the root tag, yet test_robot_name_is_jetank (60-66) calls _process a second time (a second full xacro expansion with six includes) and parses the string again. `assert len(xml) > 0` at line 42 is unreachable-false after a successful parse. A module-scoped fixture returning (root, links) halves test time and ~10 LOC.

Evidence detail: In-process expansion measured at 0.128 s per call this session; the test does it twice. test_urdf.py lines 20-66 read.

Estimated gain: ~50% test runtime, -10 LOC

Fix sketch: @pytest.fixture(scope='module') def model(): return root, links; both tests consume it; drop the len(xml) assert and the xml return value.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-19 — xacro run as a subprocess costs ~1.7 s at launch vs 0.13 s in-process; siblings re-expand it again

`/home/koen/workspaces/ros2_ws/src/jetank_description/launch/robot_description.launch.py:38` · package jetank_description · lens runtime · severity low (orig medium) · evidence measured · effort M · verdict confirmed

Description: Command([FindExecutable('xacro'), ...]) spawns a fresh Python interpreter + xacro import per launch. jetank_moveit_config/launch/demo.launch.py:56 and jetank_simulation/launch/jetank_sim_description.py:29 expand the same file again independently, so a full bringup runs xacro 2-3 times. In-process xacro.process_file (as jetank_simulation already does) is ~13x faster on the Orin.

Evidence detail: time pixi run xacro <file> use_ros2_control:=false: real 1.703 s, user 1.261 s. In-process xacro.process_file: 0.128 s (0.070 s for the ros2_control=true variant, warm). Measured on the Jetson host this session.

Estimated gain: ~1.5 s per bringup per expansion on the Orin Nano

Fix sketch: Wrap the RSP node in an OpaqueFunction that reads the LaunchConfigurations and calls xacro.process_file(..., mappings=...) directly; have moveit/sim launches include this file (its docstring already asks for that) instead of re-expanding.

Verifier (confirmed, adjusted low): Fix is safe and in-scope: jetank_description/package.xml:13 already exec_depends on xacro, jetank_simulation/launch/jetank_sim_description.py:29 already uses xacro.process_file in-process, and the only IncludeLaunchDescription consumer (jetank_ros_main/launch/urdf.launch.py:30) passes launch_arguments that an OpaqueFunction reads identically; demo.launch.py:56 and test_urdf.launch.py:41 re-run the subprocess independently and would keep working untouched. However the 1.7 s number is inflated by `pixi run` startup: measured this session, bare `xacro` inside the pixi env is 0.40-0.43 s vs 0.08-0.17 s in-process, so the real gain is ~0.3 s per expansion, not 1.5 s. Note moveit_bringup.launch.py:51 expands via MoveItConfigsBuilder (in-process) regardless, so including robot_description.launch.py from demo.launch.py removes one expansion, not two.

### jetank_description-01 — exec_depend rviz2 is unused by this package

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:28` · package jetank_description · lens footprint · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: package.xml declares <exec_depend>rviz2</exec_depend> but no launch file in this package starts rviz2 and there is no rviz/ config directory (CMakeLists installs only urdf/meshes/launch/config). rviz2 is one of the heaviest ROS packages (Qt5, Ogre, OpenGL stack); declaring it here forces rosdep/pixi to pull it onto the Jetson for a pure description package even for headless bringups.

Evidence detail: grep across src/ shows the only rviz reference involving this package is jetank_motor_control/launch/test_urdf.launch.py:75, which looks for jetank_description/rviz/urdf.rviz (a file that does not exist). No file in jetank_description references rviz2.

Estimated gain: Removes a ~100+ MB GUI dependency chain from this package's rosdep closure on headless Jetson installs

Fix sketch: Delete line 28. If RViz visualisation is wanted, keep it in the package that actually launches rviz (jetank_ros_main / jetank_motor_control).

Verifier (confirmed, adjusted low): jetank_description/package.xml:28 declares exec_depend rviz2, but the package's only launch file (launch/robot_description.launch.py:73) starts just robot_state_publisher, no rviz/ directory exists, and CMakeLists.txt:13 installs only urdf/meshes/launch/config. RViz is launched elsewhere (jetank_ros_main/launch/sim_demo.launch.py:114 via its own rviz.launch.py); the sole reference to jetank_description/rviz/urdf.rviz (jetank_motor_control/launch/test_urdf.launch.py:75) points to a nonexistent file. Severity adjusted to low because rviz2 is still pulled in by jetank_ros_main on a full install, so the closure gain only materialises for a standalone/headless description install.

### jetank_description-02 — exec_depend joint_state_publisher_gui is unused (Qt dependency)

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:21` · package jetank_description · lens footprint · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: joint_state_publisher_gui (PyQt5) is declared as an exec dependency but nothing in this package launches it. The only user in the workspace is jetank_motor_control/launch/test_urdf.launch.py:61-64, and that package does not declare it itself, so the dep is in the wrong package and drags python-qt5 into a description-only package's runtime closure.

Evidence detail: grep -rn joint_state_publisher_gui src/ hits only jetank_description/package.xml:21 and jetank_motor_control/launch/test_urdf.launch.py:61-64.

Estimated gain: Drops PyQt5 + joint_state_publisher_gui from this package's install/rosdep closure

Fix sketch: Remove line 21; add <exec_depend>joint_state_publisher_gui</exec_depend> to jetank_motor_control/package.xml where test_urdf.launch.py lives.

Verifier (confirmed, adjusted low): jetank_description/package.xml:21 declares joint_state_publisher_gui but no launch file in jetank_description references it; the only consumer is jetank_motor_control/launch/test_urdf.launch.py:61-64, and jetank_motor_control/package.xml:51 declares only joint_state_publisher, not the gui variant. The fix (move the exec_depend to jetank_motor_control/package.xml) is metadata-only, breaks no consumer (the launching package gains the dep it already needs), creates no dependency cycle (motor_control does not depend on description, though description already exec_depends on motor_control at line 17), and touches no package boundaries or perception abstractions. Note the runtime gain is limited: pixi.toml:106 installs ros-humble-joint-state-publisher-gui unconditionally, so the env still ships PyQt5 regardless; this only corrects package-level rosdep/dependency accuracy.

### jetank_description-04 — exec_depend urdf is not used directly

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:18` · package jetank_description · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Nothing in this package calls the urdf/urdfdom library; robot_state_publisher (already an exec_depend) depends on it transitively. The explicit dep adds nothing but another rosdep edge to resolve.

Evidence detail: No .py/.cmake/.xacro file in the package imports or links urdf; only robot_state_publisher consumes it.

Estimated gain: -1 dependency line

Fix sketch: Delete line 18.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-05 — ament_lint_auto/ament_lint_common pull ~7 linter runners into a package with no C++

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:30` · package jetank_description · lens footprint · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: test_depend ament_lint_auto + ament_lint_common (package.xml:30-31, CMakeLists.txt:23-31) installs and runs ament_cmake_copyright, cppcheck, cpplint, uncrustify, flake8, pep257, xmllint and lint_cmake as separate colcon test processes. The package has zero C++ sources; CMakeLists.txt:26 and :30 already hand-disable two of the runners with template boilerplate. On the Jetson each `colcon test` spawns these interpreters/binaries for no value, and the build env must carry cppcheck/uncrustify.

Evidence detail: CMakeLists.txt lines 22-31 read this session: ament_lint_auto_find_test_dependencies() with copyright/cpplint forced FOUND; the only real test is ament_add_pytest_test(test_urdf ...) at line 35. No *.cpp/*.hpp exist (src/ and include/ are empty).

Estimated gain: Removes 2 test deps and ~6 linter processes per colcon test; -8 LOC of template boilerplate in CMakeLists

Fix sketch: Drop ament_lint_auto/ament_lint_common from package.xml; in CMakeLists keep only find_package(ament_cmake_pytest) + ament_add_pytest_test (optionally ament_cmake_xmllint for package.xml). Delete CMakeLists lines 23-31.

Verifier (confirmed, adjusted low): package.xml:30-31 declares ament_lint_auto/ament_lint_common and CMakeLists.txt:23-31 pulls the full ament_lint_common runner set (with copyright/cpplint hand-disabled at :26/:30), while the only real test is ament_add_pytest_test at CMakeLists.txt:35. find shows no .cpp/.hpp anywhere in the package; src/ and include/jetank_description/ are empty directories, so cppcheck/uncrustify/cpplint runners and their binaries are pure build-env and colcon-test weight. Minor caveat: flake8/pep257/xmllint/lint_cmake would still lint test/test_urdf.py, launch/robot_description.launch.py, package.xml and CMakeLists.txt, so the gain is 'mostly wasted' rather than 'zero value'; severity low because it only affects colcon test time and test deps, not runtime.

### jetank_description-06 — test_depend xacro duplicates exec_depend xacro

`/home/koen/workspaces/ros2_ws/src/jetank_description/package.xml:33` · package jetank_description · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: xacro is already an exec_depend (line 13); tests run in the install environment where exec deps are present, so the extra test_depend is redundant.

Evidence detail: package.xml lines 13 and 33 both declare xacro.

Estimated gain: -1 LOC

Fix sketch: Delete line 33.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-07 — add_compile_options for a package with no compiled targets

`/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:4` · package jetank_description · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 4-6 set -Wall -Wextra -Wpedantic for GCC/Clang, but the CMakeLists defines no add_executable/add_library. Dead template code.

Evidence detail: CMakeLists.txt contains only install(DIRECTORY ...) rules and a pytest test; no compile targets.

Estimated gain: -3 LOC

Fix sketch: Delete lines 4-6.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-08 — Empty template dirs (meshes/, config/, src/, include/) installed or left on disk

`/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:13` · package jetank_description · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The foreach at line 13 installs meshes/ and config/ 'only if present' — both are present but empty (untracked, from the ament template), so colcon creates empty share/jetank_description/meshes and config dirs on every build. src/ and include/jetank_description/ are likewise empty leftovers. The EXISTS guard and loop exist only to tolerate this.

Evidence detail: ls -laR config include meshes src shows all four empty; git ls-files does not track them; README.md:3 says 'no meshes'.

Estimated gain: -8 LOC in CMake, 4 dirs removed, 2 empty install dirs gone

Fix sketch: rmdir config include/jetank_description include meshes src; replace the foreach with install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME}).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-09 — Sibling launch references non-existent jetank_description/rviz/urdf.rviz

`/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:13` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_motor_control/launch/test_urdf.launch.py:75 loads [FindPackageShare('jetank_description'), 'rviz', 'urdf.rviz'], but this package has no rviz/ directory and the install loop (line 13) never installs one. That launch is therefore broken dead code from this package's point of view; either the rviz config should exist and be installed, or the sibling launch should be deleted.

Evidence detail: ls src/jetank_description/rviz -> No such file or directory; grep urdf.rviz hits only jetank_motor_control/launch/test_urdf.launch.py:75.

Estimated gain: Removes a broken launch path (or ~30 LOC in the sibling)

Fix sketch: Prefer deleting jetank_motor_control/launch/test_urdf.launch.py (jetank_ros_main/urdf.launch.py already covers RSP+JSP bringup); otherwise add rviz/urdf.rviz here and 'rviz' to the install list.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-10 — CMake template boilerplate comments and lint overrides

`/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:24` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 24-30 are the stock ament template comments ('comment the line when a copyright and license is added...') plus set(ament_cmake_copyright_FOUND TRUE)/set(ament_cmake_cpplint_FOUND TRUE). The package is in a git repo and has a LICENSE, so the comments are false, and the overrides only exist to suppress linters that should not be enabled at all (see -05).

Evidence detail: CMakeLists.txt:24-30 read this session.

Estimated gain: -7 LOC

Fix sketch: Delete with the ament_lint_auto block.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-12 — Classic-Gazebo <material>Gazebo/*</material> blocks are dead under Ignition Fortress

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:203` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: 15 <material>Gazebo/Grey|Blue|Black|DarkGrey</material> tags (arm.xacro:203-221, gripper.xacro:128/131/142, camera.xacro:76-78, wheels.xacro:48, jetank_ros2_control.urdf.xacro:85-87) reference classic Gazebo OGRE material scripts. The project targets Ignition/gz-sim Fortress (ign_ros2_control, gpu_lidar, ignition_frame_id), where these script names are not resolved; colours come from the URDF <material><color> already present on each visual. 9 of the <gazebo reference> blocks contain nothing else and are pure dead output in every expansion.

Evidence detail: Count from the in-session expansion script: gazebo_material_tags=15, gazebo_material_only_blocks=9. Ignition target confirmed by ros2_control.xacro:31 (ign_ros2_control/IgnitionSystem) and README.md:59 ('Gazebo (Fortress/Ignition)'). Rendering claim is static reasoning, not observed in sim this session.

Estimated gain: ~27 LOC across 5 files; 9 fewer elements in every robot_description

Fix sketch: Delete the material-only <gazebo reference> blocks; drop the <material> line from the blocks that also carry kp/kd/mu (gripper fingers, wheels).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-13 — wheel_inertia macro duplicates cylinder_inertia

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/wheels.xacro:14` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: wheels.xacro:14-21 defines wheel_inertia(m,r,w) with exactly the same formula as arm.xacro:22-29 cylinder_inertia(m,r,l). All component files are included into one xacro namespace by the top file, so one macro suffices.

Evidence detail: Both macros compute ixx=iyy=m(3r²+l²)/12, izz=mr²/2.

Estimated gain: -8 LOC

Fix sketch: Delete wheel_inertia; call <xacro:cylinder_inertia m=... r=... l=${wheel_width}/> at wheels.xacro:44 (or move both inertia macros to a shared inertials.xacro).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-14 — Hand-typed inertia tensors where box/cylinder_inertia macros exist

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:107` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: S2_link (arm.xacro:107-113), S3_link (143-149), camera_link (camera.xacro:33-38), gripper_base/left/right (gripper.xacro:31-36, 68-74, 117-123), laser (lidar.xacro:24-28) and imu_link (imu.xacro:23-28) each spell out a 7-line <inertial> with rounded constants, although box_inertia/cylinder_inertia macros are defined in the same include set and used for other links. Using the macros removes ~40 LOC and makes the values derive from the geometry.

Evidence detail: For S2_link (0.12x0.055x0.025 m, 0.04 kg) the macro gives ixx=1.2e-5, iyy=izz=5.0e-5 vs the hand-typed 1e-5/5e-5/5e-5 — equivalent within rounding, so behaviour is preserved.

Estimated gain: ~40 LOC across 5 files

Fix sketch: Replace each hand-written <inertial> with <xacro:box_inertia .../> or <xacro:cylinder_inertia .../> plus <origin> where the visual is offset.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-15 — Left/right stereo camera sensor blocks are verbatim duplicates

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:85` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: camera.xacro:85-119 and 123-157 are identical 35-line <gazebo><sensor> blocks differing only in link, sensor name, topic and frame id. The camera body/optical joints (44-73) are likewise mirrored pairs. A macro with a `side` and `y` parameter halves the file.

Evidence detail: Diff of the two sensor blocks: only 'left'/'right' substitutions in 4 attributes.

Estimated gain: ~45 LOC

Fix sketch: <xacro:macro name="stereo_eye" params="side y"> containing the body joint, optical joint and sensor; instantiate twice with side=left y=0.03 / side=right y=-0.03.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-16 — Right gripper finger link and gazebo block duplicate the left

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:103` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gripper_right_link (gripper.xacro:103-124) is a copy of gripper_left_link (54-75) and its <gazebo> contact block (141-147) copies 130-140. A finger macro with `side`, `y` and an optional mimic flag removes ~30 LOC.

Evidence detail: Lines compared this session; only the link name differs in the link bodies.

Estimated gain: ~30 LOC

Fix sketch: <xacro:macro name="finger" params="side y axis_y mimic:=false"> emitting joint+link+gazebo; conditional <mimic> via xacro:if.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-17 — use_ros2_control condition evaluated twice

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:25` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 25-27 include ros2_control.xacro under xacro:if use_ros2_control and lines 80-82 instantiate the macro under the same condition. The include can sit inside the second block, removing one conditional and 3 LOC.

Evidence detail: Top xacro read this session; both blocks guard on $(arg use_ros2_control).

Estimated gain: -3 LOC

Fix sketch: Move the <xacro:include> into the xacro:if at line 80.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-18 — $(find jetank_description) used for the package's own component includes

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:17` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Lines 17-22 resolve six sibling files through the ament index instead of a relative path. Each expansion performs six package lookups, and — more importantly — test/test_urdf.py expands the SOURCE top file while its includes resolve to the INSTALLED copies, so the test silently validates stale component files unless the package was rebuilt first.

Evidence detail: Live robot_description banner shows expansion from install/jetank_description/share/...; test_urdf.py:17 points URDF_DIR at the source tree. xacro resolves relative include filenames relative to the including file.

Estimated gain: 6 ament lookups per expansion removed; test becomes source-consistent

Fix sketch: Use filename="components/arm.xacro" etc.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-22 — imu_link is an identity-offset link with a 3 mm visual and inertial

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/imu.xacro:13` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: imu_link is mounted at xyz=0 0 0 rpy=0 0 0 on camera_link (top xacro:77) and carries a 3x3x1 mm visual box with its own material plus a 3 g inertial. The visual is never discernible in RViz/gz and the inertial is lumped into camera_link by sdformat anyway. Keeping the frame name (used by the icm20948_imu driver) with an empty <link name="imu_link"/> drops 15 LOC and one material from robot_description.

Evidence detail: tf_static shows camera_link->imu_link translation (0,0,0) rotation identity. imu.xacro:13-29 read this session.

Estimated gain: -15 LOC, one fewer material/visual in every consumer

Fix sketch: Replace lines 13-29 with <link name="imu_link"/>.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-26 — README contains stale/contradictory facts about the model

`/home/koen/workspaces/ros2_ws/src/jetank_description/README.md:29` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: README.md:29 lists lidar range 0.05–12 m but lidar.xacro:66 sets min 0.18 (and README:86 says 0.18 — internal contradiction). README:17-18 says use_sim=false ⇒ JetankSerialHardware, but the xacro default hardware=mock ⇒ GenericSystem (README:78 says so). README:38-39 cites 'jetank_ros2_control.urdf.xacro:71' and 'line 66' — actual lines are 77 and 72. README:57 says there is no src/ directory while empty src/ and include/ dirs exist on disk. Stale docs cost reader time and mislead the sim/hardware switch.

Evidence detail: Cross-checked against lidar.xacro:66, jetank_ros2_control.urdf.xacro:14/72/77 and ls output this session.

Estimated gain: Correct docs; ~5 lines fixed/removed

Fix sketch: Fix the range, the hardware default sentence and the line references; delete the empty dirs so line 57 becomes true.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-27 — gpu_lidar <visualize>true</visualize> renders 360 rays per scan in the sim GUI

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/lidar.xacro:46` · package jetank_description · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: <visualize>true</visualize> makes gz-sim draw the lidar rays every update (10 Hz x 360 samples) in the GUI render loop. Sim-only cost, but it is on by default in the canonical model and has no functional effect on /scan.

Evidence detail: lidar.xacro:46 read; sim not running this session, so no GPU measurement.

Estimated gain: Fewer GUI draw calls in simulation

Fix sketch: Set <visualize>false</visualize> (or drop the tag; default is false).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-28 — Gaussian image noise on both sim cameras adds a per-pixel pass at 640x360x30 fps x2

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:101` · package jetank_description · lens runtime · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: <noise type gaussian stddev 0.007> on both camera sensors (camera.xacro:101-105, 139-143) makes gz-sim run a noise shader over every rendered frame (2 cameras x 30 Hz x 230 kpx). Useful for robustness testing but unconditional in the canonical model; a xacro arg (camera_noise:=false default) would keep sim frame time down on the Jetson when the sim is run there.

Evidence detail: camera.xacro lines 101-105 and 139-143 read; no sim running this session to measure.

Estimated gain: Removes one full-frame shader pass per camera frame in simulation

Fix sketch: Wrap the <noise> block in <xacro:if value="$(arg camera_noise)"/> with default false.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-29 — Custom lens function on the stereo cameras is unexplained and possibly non-pinhole

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:106` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Both cameras declare <lens><type>custom</type><custom_function c1=1.05 c2=4 f=1.0 fun=tan> with scale_to_hfov. The downstream stereo pipeline (jetank_perception, reprojectImageTo3D) assumes a pinhole model; a custom tan-based mapping with c1=1.05 is not identical to pinhole. If this block is a leftover from a wide-angle template it is 10 dead lines per camera and a possible source of sim-vs-hardware disparity error. NOT verified in sim this session — flagged for the verification stage.

Evidence detail: camera.xacro:106-115 and 144-153 read; no in-sim comparison of projected vs pinhole intrinsics was run.

Estimated gain: -20 LOC if dead; removes a projection-model mismatch risk

Fix sketch: Delete the <lens> block (gz default is pinhole with the given horizontal_fov) unless a measured reason for the custom mapping exists.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-30 — 14-line historical ORIGINAL_NOTE comment duplicates ros2_control.xacro prose

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/gripper.xacro:78` · package jetank_description · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gripper.xacro:78-91 narrates a past change ('RESTORED', 'REMOVED', 'may or may not') and the same explanation is repeated in jetank_motor_control/config/ros2_control.xacro:123-146. Change history belongs in git; the comment is also shipped inside robot_description (see -11).

Evidence detail: Both comment blocks read this session; content overlaps on mimic/_mimic-suffix rationale.

Estimated gain: -14 LOC and ~900 B of robot_description

Fix sketch: Replace with a one-liner: '<mimic> kept for RSP/MoveIt; ros2_control mimic params omitted (see ros2_control.xacro).'

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-23 — base_link is an empty pass-through link between base_footprint and chassis

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:39` · package jetank_description · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: base_footprint -> base_link (z 0.03) -> chassis (z 0.0275) uses two empty links and two fixed joints. base_link is REP-105 required, but chassis adds nothing: its visual/collision/inertial can live on base_link with an <origin z=0.0275>. That removes one link, one joint and one tf_static frame from every consumer (MoveIt planning scene, RViz, tf buffers). Check jetank_moveit_config SRDF/ACM references to 'chassis' before renaming.

Evidence detail: Top xacro lines 30-67; tf_static shows base_footprint->base_link and base_link->chassis as separate static transforms.

Estimated gain: -1 link/-1 joint/-1 static TF, ~8 LOC

Fix sketch: Move the chassis <visual>/<collision>/<inertial> under base_link with origin z=0.0275; set parent="base_link" for arm/wheels/lidar; update SRDF if it names chassis.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### jetank_description-24 — S4_link is a separate fixed link that could be a second visual on S3_link

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/arm.xacro:157` · package jetank_description · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: S4_joint is fixed (line 157) and S4_link (163-176) is just a 2x2x3 cm box. URDF allows multiple <visual>/<collision> per link, so the box could be appended to S3_link at x=0.09 and S5_joint parented to S3_link with origin (0.09,0,0.015). Saves a link, a joint and a static transform, and the ros2_control.xacro:246-249 gazebo block for S4_joint becomes unnecessary. Only if jetank_moveit_config does not name S4_link in its SRDF.

Evidence detail: arm.xacro:152-176 read; tf_static contains S3_link->S4_link (0.09,0,0) identity rotation.

Estimated gain: -1 link/-1 joint/-1 static TF, ~20 LOC

Fix sketch: Fold S4 geometry into S3_link; grep the moveit SRDF for S4_link first.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## cross-duplication

Coverage: 113 files read; 7 measurements run. Notes: Compared every launch/ directory (37 launch files across 10 packages, all read in full except none skipped), every config/ directory (all YAML/xacro/SRDF/rviz files read or key-line grepped; nav2_params.yaml and the large rviz files were grepped for displays/topics rather than read line-by-line), every Python helper module (action_utils, node_runner, grasp_math, backends, capture_frames, topics.py, cmd_vel_bridge, gripper_mimic_relay) and the C++ headers/entry points relevant to cross-package coupling (quality_monitor*.hpp, sock_reproject.hpp, sock_segmentation_server.cpp head, stereo_camera_node.cpp parameter block, robot_controller.cpp parameter block). Not read in full: stereo_camera_node.cpp (2019 lines), web_control_node.py (1761 lines; ~600 lines read), grasp_server.py body, base_approach_node/mobile_grasp_coordinator bodies, feetech_bus/jetank_serial_hardware, icm20948_node.cpp, camera_interface.hpp, stereo_processing_strategy.hpp, world SDFs, static/app.js - intra-package logic outside the cross-package remit. Measured evidence covers only what the currently running stack (unified.launch.py with enable_moveit, no lidar, no detector) exposes; controller_manager was not present so the /joint_states double-publisher condition (cross-10) was inferred from JSP being live at 10 Hz plus the launch graph. No measurement tool failed. Baseline exclusions were not applied; findings previously touched by the July 2026 audit (topics.yaml contract, node_runner/action_utils consolidation, per-controller YAML split) are re-reported where residual duplication remains.

Measurements:
- mcp__ros2-mcp__get_node_list (ROS_DOMAIN_ID=42): 15 nodes incl. /robot_state_publisher, /joint_state_publisher, /world_to_base_footprint_tf, /robot_controller, /icm20948_imu, /web_control_node, /stereo_camera/stereo_camera_node, three controller spawners, /move_group (x2 entries); no /controller_manager, no rplidar, no sock_detector
- mcp__ros2-mcp__get_topic_list: 49 topics; /robot_description, /robot_description_semantic, /joint_states, /stereo_camera/{disparity,points,left|right/image_raw(+compressed),image_rect(+compressed),camera_info}, /detections/socks, /grasp_object action, /controller_manager/{follow_joint_trajectory,gripper_cmd} action topics; no /scan
- mcp__ros2-mcp__get_node_info(/joint_state_publisher): publishes /joint_states, subscribes /robot_description
- mcp__ros2-mcp__get_node_info(/robot_state_publisher): publishes /robot_description, /tf, /tf_static; subscribes /joint_states
- mcp__ros2-mcp__get_node_info(/web_control_node): publishes /cmd_vel,/initialpose; subscribes /amcl_pose,/detections/socks,/map,/stereo_camera/left/image_raw/compressed, grasp_object + navigate_to_pose action feedback/status
- mcp__ros2-mcp__get_topic_hz(/joint_states, 5 s): 10.001 Hz, 50 msgs (joint_state_publisher rate; joint_state_broadcaster not yet up)
- mcp__ros2-mcp__get_topic_hz(/robot_description, 3 s): count=2 latched messages received (two publishers: robot_state_publisher + move_group)

Findings: 31 (high 0, medium 5, low 26).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| cross-02 | minimality | sock_detector lifecycle auto-activation copy-pasted three times in three launch styles | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/sim_demo.launch.py:220` | medium | static | ~30 launch lines removed; 3-6 fewer ros2 CLI subprocesses per bring-up; removes 22-46 s fixed sleeps in the mobile_grasp launches | S | confirmed |
| cross-06 | minimality | motor_params.yaml keys do not match robot_controller.cpp parameter names (config silently ignored) | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/config/motor_params.yaml:3` | medium | static | Config actually applied; 4 dead YAML keys removed; config co-located with its node | S | confirmed |
| cross-07 | minimality | Base geometry and velocity limits duplicated with conflicting values across five configs | `/home/koen/workspaces/ros2_ws/src/jetank_motor_control/config/jetank_controllers.yaml:123` | medium | static | Sim/hardware odometry consistency; one geometry source; removes 4 redundant limit declarations | S | confirmed |
| cross-01 | minimality | navigation_full.launch.py re-implements unified.launch.py's hardware bring-up layer | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/navigation_full.launch.py:71` | medium | static | ~120 lines removed; one bring-up graph to maintain; no double-spawn risk when both files are combined by sim_demo | M | confirmed |
| cross-09 | minimality | SRDF named states and arm joint list mirrored by hand in grasp_server.py and joint_limits.yaml (already drifted) | `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:320` | medium | static | Removes a 30-line mirror that is already inconsistent; MoveIt joint constraints always match the SRDF | M | confirmed |
| cross-03 | minimality | Mobile-grasp pipeline node set duplicated between mobile_grasp.launch.py and mobile_grasp_hw.launch.py | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:135` | low (orig medium) | static | ~40 lines removed; sim and hw run the same pipeline definition; grasp_poses.yaml applied in sim too | S | confirmed |
| cross-04 | minimality | SetParameter+GroupAction wrappers repeated in five places because web_control/detect launch files lack topic args | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:225` | low | static | ~35 launch lines removed across 4 files; no parameter leakage to sibling nodes | S | **UNVERIFIED** |
| cross-08 | minimality | arm/gripper controller parameters maintained in two files marked 'keep in sync' | `/home/koen/workspaces/ros2_ws/src/jetank_motor_control/config/controllers/arm_controller.yaml:10` | low | static | ~45 duplicated YAML lines removed; single tuning point | S | **UNVERIFIED** |
| cross-12 | minimality | world->base_footprint static TF for the MoveIt virtual joint declared in two packages | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:71` | low | static | ~10 lines removed; MoveIt bring-up self-contained | S | **UNVERIFIED** |
| cross-16 | minimality | stereo_camera_config.yaml is bound to the literal 'stereo_camera' namespace that topics.yaml is supposed to own | `/home/koen/workspaces/ros2_ws/src/jetank_perception/config/stereo_camera_config.yaml:4` | low | static | Config applies under any namespace; one calibration-URL source | S | **UNVERIFIED** |
| cross-19 | minimality | stereo_camera_sim.launch.py re-declares the stereo node instead of parameterising stereo_camera.launch.py | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/stereo_camera_sim.launch.py:17` | low | static | ~35 lines removed; single stereo node definition | S | **UNVERIFIED** |
| cross-20 | minimality | gazebo_remote/robot_remote launch pair is a stale fork of gazebo.launch.py | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/robot_remote.launch.py:60` | low | static | ~190 lines and two stale entry points removed | S | **UNVERIFIED** |
| cross-21 | minimality | jetank_motor_control installs a launch/ dir containing a deprecated stub and a broken URDF test launch | `/home/koen/workspaces/ros2_ws/src/jetank_motor_control/launch/test_urdf.launch.py:74` | low | static | 114 lines, one install rule and two rosdep keys removed | S | **UNVERIFIED** |
| cross-23 | minimality | single_camera/simple_camera launch files and the camera_node executable are unused template leftovers | `/home/koen/workspaces/ros2_ws/src/jetank_perception/launch/single_camera.launch.py:112` | low | static | 214 launch lines removed; one fewer C++ target (OpenCV link) per build on the Orin | S | **UNVERIFIED** |
| cross-24 | minimality | navigation.rviz is a strict subset of jetank_ros_main's unified.rviz | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/rviz/navigation.rviz:1` | low | static | 79 lines removed; one RViz layout to update on topic changes | S | **UNVERIFIED** |
| cross-26 | minimality | Training-image capture implemented twice with incompatible on-disk layouts | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/capture_frames.py:46` | low | static | One capture implementation (~140 lines removed) and one dataset layout | S | **UNVERIFIED** |
| cross-27 | minimality | Map saving wrapped twice (save_map.sh and web_control_node.save_map) | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/scripts/save_map.sh:37` | low | static | 57 lines and one installed script removed | S | **UNVERIFIED** |
| cross-29 | minimality | Boolean launch-arg idioms and use_sim_time declarations re-implemented across 23 launch files | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:209` | low | static | ~40 lines simplified; consistent use_sim_time default spelling | S | **UNVERIFIED** |
| cross-30 | footprint | pixi.toml and 1.2 MB pixi.lock duplicated verbatim in jetank_ros_main/workspace_template; pixi deps unused by any source | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:1` | low | static | 1.2 MB removed from the package repo; three conda packages fewer in the ~6 GB env | S | **UNVERIFIED** |
| cross-31 | footprint | package.xml exec_depends declared for tools no launch or source uses | `/home/koen/workspaces/ros2_ws/src/jetank_perception/package.xml:46` | low | static | ~10 rosdep keys fewer on a clean install; honest package graph | S | **UNVERIFIED** |
| cross-32 | minimality | Executor/QoS boilerplate copied between packages instead of one shared helper | `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:490` | low | static | ~40 lines removed; consistent shutdown semantics across nodes | S | **UNVERIFIED** |
| cross-33 | minimality | Installed test_drive/test_cameras console scripts target a control interface the sim no longer exposes | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:21` | low | static | 419 lines and two console entry points removed (or one working smoke test) | S | **UNVERIFIED** |
| cross-34 | minimality | Docs and root CLAUDE.md describe a PointCloud2->LaserScan node and laser_data.yaml that no longer exist | `/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:10` | low | static | Accurate navigation docs; no dead references for tooling | S | **UNVERIFIED** |
| cross-35 | footprint | ros2_control.xacro lives in jetank_motor_control, forcing jetank_description to depend on the C++ hardware package | `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:26` | low | static | jetank_description buildable/usable standalone; removes a description->hardware package edge | S | **UNVERIFIED** |
| cross-37 | minimality | World-name to SDF mapping owned by jetank_ros_main instead of the simulation package that ships the worlds | `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/gazebo_sim.launch.py:39` | low | static | 71 lines and one wrapper removed; worlds registered where they live | S | **UNVERIFIED** |
| cross-38 | footprint | Empty config/model/map directories installed and documented as configuration locations | `/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:8` | low | static | Fewer misleading paths for readers/agents; trivial install cleanup | S | **UNVERIFIED** |
| cross-39 | minimality | gazebo_headless.launch.py is a compatibility wrapper that re-declares three arguments to pass gui:=false | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_headless.launch.py:40` | low | static | 72 lines and one entry point removed | S | **UNVERIFIED** |
| cross-05 | minimality | robot_description xacro expansion implemented four different ways across four packages | `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:52` | low | static | ~60 lines removed; one place for xacro path/args; drops the sys.path import hack in jetank_simulation | M | **UNVERIFIED** |
| cross-15 | minimality | Topic contract (topics.yaml) covers 4 topics; a dozen cross-package topic/action names remain hardcoded, some not even parameters | `/home/koen/workspaces/ros2_ws/src/jetank_perception/src/sock_segmentation_server.cpp:134` | low (orig medium) | static | Camera/controller renames become one-file edits; segmentation server configurable; removes ~10 duplicated literals | M | confirmed |
| cross-25 | minimality | DetectSocks action has no client in the workspace; every integration launch runs the detector in continuous mode | `/home/koen/workspaces/ros2_ws/src/jetank_detection/action/DetectSocks.action:1` | low | static | One rosidl interface fewer (build time), ~130 node lines removable if on-demand mode is confirmed dead | M | **UNVERIFIED** |
| cross-28 | minimality | web_control_node hardcodes jetank_navigation's lifecycle node names and launch files for pkill/launch management | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1014` | low | static | Removes 11 subprocess spawns + 1 s per nav mode switch; one owner for the node list | M | **UNVERIFIED** |

### cross-02 — sock_detector lifecycle auto-activation copy-pasted three times in three launch styles

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/sim_demo.launch.py:220` · package jetank_detection · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: The configure->activate sequence for /sock_detector is implemented three different ways: sim_demo.launch.py 220-235 (bash poll loop `ros2 node list | grep` up to 180 s then two `ros2 lifecycle set`), mobile_grasp.launch.py 110-113 (two TimerActions at 40 s/46 s) and mobile_grasp_hw.launch.py 152-155 (TimerActions at 22 s/28 s). Each spawns 2-3 extra `ros2` CLI processes (each ~1 s of Python startup + daemon discovery on the Orin) and the fixed timers either fire too early under load (goal rejected) or waste tens of seconds. The knowledge of how to activate the detector belongs to jetank_detection, not to three integration launch files.

Evidence detail: Read all three launch files; ExecuteProcess blocks quoted above. detect.launch.py (jetank_detection) declares no autostart argument.

Estimated gain: ~30 launch lines removed; 3-6 fewer ros2 CLI subprocesses per bring-up; removes 22-46 s fixed sleeps in the mobile_grasp launches

Fix sketch: Add an `autostart` parameter to sock_detector_node (self-transition configure->activate from a one-shot timer after construction, or use launch_ros LifecycleNode + EmitEvent/RegisterEventHandler in detect.launch.py). Delete the three ExecuteProcess/TimerAction blocks and pass autostart:=true.

Verifier (confirmed, adjusted medium): The three copies exist as described (sim_demo.launch.py:219-227 bash poll + 2 CLI calls; mobile_grasp.launch.py:110-113 and mobile_grasp_hw.launch.py:152-155 fixed TimerActions), and the fix is in-scope: detect_sim/detect_real.launch.py are thin pass-through wrappers over detect.launch.py (detect_sim.launch.py:19-20, detect_real.launch.py:20-22), so an `autostart` arg added once in jetank_detection propagates to all three includes without touching package boundaries. Grep of the workspace shows no other consumer drives /sock_detector lifecycle transitions (only README/docstring text at jetank_web_control/README.md:214 and detect*.launch.py docstrings), so defaulting autostart to false keeps the manual `ros2 lifecycle set` workflow intact. One implementation caveat: parameters are currently declared in on_configure (sock_detector_node.py:88-99), so `autostart` must be declared in __init__ (line 62-79) and configure failure (missing model, on_configure returns FAILURE) must stay non-fatal as sim_demo.launch.py:218-219 already expects.

Duplicate folded in: **jetank_ros_main-23** — same sim_demo bash poll loop for /sock_detector activation; jetank_ros_main-23 (low) frames it as up to ~90 ros2 CLI spawns during bring-up and suggests launch_ros LifecycleNode + EmitEvent/OnStateTransition.

### cross-06 — motor_params.yaml keys do not match robot_controller.cpp parameter names (config silently ignored)

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/config/motor_params.yaml:3` · package jetank_motor_control · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: jetank_ros_main/config/motor_params.yaml sets left_motor_forward_channel/left_motor_reverse_channel/right_motor_forward_channel/right_motor_reverse_channel (lines 3-9), but robot_controller.cpp declares only `left_motor` and `right_motor` ints (lines 21-22, defaults 0/1). rclcpp ignores undeclared YAML keys, so the channel mapping in the config has no effect and the node always runs with 0/1. The node's config lives in a different package (jetank_ros_main) than the node (jetank_motor_control), which is how the drift went unnoticed; motor_controller.launch.py (jetank_ros_main line 11-15) is the only loader.

Evidence detail: grep of declare_parameter in robot_controller.cpp lines 21-37 shows no *_channel keys; motor_params.yaml lines 3-9 use *_channel keys.

Estimated gain: Config actually applied; 4 dead YAML keys removed; config co-located with its node

Fix sketch: Move motor_params.yaml to jetank_motor_control/config, rename keys to left_motor/right_motor (or declare the *_channel params in robot_controller.cpp), and have jetank_ros_main/motor_controller.launch.py load it from jetank_motor_control's share.

Verifier (confirmed, adjusted medium): motor_params.yaml:3,4,8,9 set left/right_motor_forward/reverse_channel, but robot_controller.cpp:21-22 declares only `left_motor`/`right_motor` ints (README.md:42-43 documents these as the real names) and no `*_channel` key appears anywhere in jetank_motor_control. motor_controller.launch.py:11-22 is the only loader and passes this yaml, so the four channel keys are silently ignored by rclcpp and the node always uses channels 0/1. Also notable: the yaml's channel values (1/0, 2/3) conflict with the single-index model the node actually uses, so the intent behind the config cannot even be expressed with current declarations.

Duplicate folded in: **jetank_ros_main-03** — same motor_params.yaml *_channel keys never declared by robot_controller; verifier of jetank_ros_main-03 (adjusted low) warns that the fix_sketch "delete motor_params.yaml" is wrong because alpha/beta/track_width ARE declared and loaded, so only the four channel lines are dead and should be renamed.

### cross-07 — Base geometry and velocity limits duplicated with conflicting values across five configs

`/home/koen/workspaces/ros2_ws/src/jetank_motor_control/config/jetank_controllers.yaml:123` · package workspace · lens minimality · severity medium · evidence static · effort S · verdict confirmed

Description: Track width is 0.14 m in jetank_controllers.yaml line 123 (sim diff_drive, derived from wheels.xacro 2x0.07) but 0.11 m in jetank_ros_main/config/motor_params.yaml line 13 and robot_controller.cpp default line 27 (hardware open-loop odometry). Angular odometry therefore differs by 27% between sim and hardware for the same /cmd_vel, so Nav2/base_approach tuned in sim (nav2_params robot_radius 0.12, max_vel_x 0.3, lines 138-146,189) transfer wrong. Velocity caps are also set independently: motor_params 1.0 m/s / 2.0 rad/s, diff_drive 0.5/2.0 (lines 137,144), web_control.launch.py 0.5/1.0 (lines 76-77), nav2 0.3/1.0, base_approach 0.15/0.8 (base_approach_node.py 152-153).

Evidence detail: Values quoted from the files read this session; wheels.xacro line 10 wheel_radius 0.03 matches controllers yaml line 124 but no shared source for separation exists.

Estimated gain: Sim/hardware odometry consistency; one geometry source; removes 4 redundant limit declarations

Fix sketch: Define track width once (xacro property in jetank_description/urdf/components/wheels.xacro or a shared jetank_description/config/base_geometry.yaml) and load it in both motor_params.yaml and jetank_controllers.yaml; pick one hardware track width after measuring; make web/base_approach caps <= diff_drive caps.

Verifier (confirmed, adjusted medium): Facts verified: wheel_separation 0.14 at jetank_motor_control/config/jetank_controllers.yaml:123 (matches wheels.xacro:57-60 y=±0.07) vs track_width 0.11 at jetank_ros_main/config/motor_params.yaml:13 and robot_controller.cpp:27 (used in kinematics at :118-119), so sim and hardware odometry disagree. The fix is in-scope (no package merge, perception untouched) and safe for consumers: only gazebo.launch.py/moveit_bringup.launch.py:94 load the controllers yaml and motor_controller.launch.py:14 loads motor_params, all via file paths, so a single source needs launch-side substitution rather than yaml includes and adds a jetank_description dependency to jetank_motor_control (jetank_ros_main already has it, package.xml:29). Two caveats: unifying deliberately changes one path's kinematics (must measure the real robot first, as the sketch says), and the "web/base_approach caps <= diff_drive" part is already satisfied (web 0.5/1.0, base_approach 0.15/0.8 vs diff_drive 0.5/2.0), so that piece of the sketch is a no-op.

### cross-01 — navigation_full.launch.py re-implements unified.launch.py's hardware bring-up layer

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/navigation_full.launch.py:71` · package jetank_navigation · lens minimality · severity medium · evidence static · effort M · verdict confirmed

Description: navigation_full.launch.py (lines 71-120) includes urdf.launch.py, motor_controller.launch.py, imu.launch.py, lidar.launch.py, slam.launch.py and nav2_bringup.launch.py with its own mode/map/use_sim_time plumbing. unified.launch.py (jetank_ros_main, lines 194-341) includes exactly the same six files with a second, divergent argument set (enable_navigation/navigation_mode vs mode; unified adds camera + web + moveit; navigation_full gates hardware nodes on use_sim_time, unified does not). Two bring-up graphs for one robot means every hardware change (e.g. the IMU add) must be patched twice, and pixi.toml tasks `slam`/`nav2` (root pixi.toml lines 41,45) go through navigation_full while docs point at unified/main. sim_demo.launch.py (line 133-136) even includes navigation_full only for its SLAM branch, pulling a whole hardware bring-up file in to get one node.

Evidence detail: Read both files in full: same six IncludeLaunchDescription targets; unified.launch.py has no UnlessCondition(use_sim_time) gating while navigation_full has it on lines 76,84,93,101. Root pixi.toml lines 41 and 45 reference navigation_full.

Estimated gain: ~120 lines removed; one bring-up graph to maintain; no double-spawn risk when both files are combined by sim_demo

Fix sketch: Give unified.launch.py an `enable_hardware`/`use_sim_time` gate for the hardware layer (and an `enable_perception` flag), then reduce navigation_full.launch.py to an include of unified with enable_navigation:=true, enable_perception:=false and mode mapping, or delete it and repoint pixi.toml tasks slam/nav2 at unified.launch.py.

Verifier (confirmed, adjusted medium): navigation_full.launch.py:71-120 and unified.launch.py:193-341 include the same six launch files (urdf, motor_controller, imu, lidar, slam, nav2_bringup) with divergent argument plumbing: navigation_full gates the four hardware includes with UnlessCondition(use_sim_time) at lines 76/84/93/101, while unified.launch.py:193-284 has no such gating and uses enable_navigation/navigation_mode/map_file instead of mode/map. Root pixi.toml:41,45 route slam/nav2 tasks through navigation_full, while main.launch.py:28-33 and mobile_grasp_hw.launch.py:81-83 wrap unified, and sim_demo.launch.py:126-135 includes navigation_full solely to obtain the slam branch. Duplication is real; a residual concern is that unified's hardware layer would need the use_sim_time gate before navigation_full could be collapsed into it, which the fix sketch already covers.

### cross-09 — SRDF named states and arm joint list mirrored by hand in grasp_server.py and joint_limits.yaml (already drifted)

`/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:320` · package jetank_manipulation · lens minimality · severity medium · evidence static · effort M · verdict confirmed

Description: grasp_server.py `_SRDF_STATES` (lines 318-350, comment 'mirror of jetank_moveit_config/config/jetank.srdf. Update here if the SRDF changes') hardcodes home/ready/grasp_pre/grasp_reach joint values including S5_joint, while jetank.srdf defines the `arm` group as chain base_link->S4_link (line 18) whose group_states list only S1/S2/S3 (lines 40-73); grasp_server EE_LINK is 'S5_link' (line 56) though the SRDF end_effector parent is S4_link (line 33). jetank_moveit_config/config/joint_limits.yaml lines 27-32 also carries S5_joint limits for a joint outside the planning group. The S1/S2/S3 joint list is repeated in jetank_controllers.yaml, arm_controller.yaml, moveit_controllers.yaml, joint_limits.yaml, the SRDF and grasp_server.

Evidence detail: Quoted lines from grasp_server.py, jetank.srdf, joint_limits.yaml; /robot_description_semantic is published on the live graph (get_topic_list result) so the SRDF is available at runtime.

Estimated gain: Removes a 30-line mirror that is already inconsistent; MoveIt joint constraints always match the SRDF

Fix sketch: At grasp_server start, read the SRDF via get_package_share_directory('jetank_moveit_config')/config/jetank.srdf (xml.etree) or from the latched /robot_description_semantic topic, and build the named-target JointConstraints from its group_state elements; drop _SRDF_STATES and the S5 entries in joint_limits.yaml.

Verifier (confirmed, adjusted medium): Drift is real: grasp_server.py:320-352 hardcodes S5_joint=0.0 in every named state and EE_LINK="S5_link" (line 58), while jetank.srdf:18 defines `arm` as chain base_link->S4_link, its group_states (lines 40-73) list only S1/S2/S3, and the EE parent is S4_link (line 33); joint_limits.yaml:27-32 also carries S5_joint outside the group, and test_import.py:159-184 asserts the stale S5-including mirror, so the fix must touch the test too. The est_gain is honest as a maintenance/correctness gain (~30 LOC removed, constraints derived from the SRDF at startup via a one-time XML parse that is negligible on the Orin Nano); there is no CPU/latency gain and none is claimed. Medium severity is fair because the mismatch is a latent functional risk (JointConstraint on a joint outside the planning group, pose targets on a link MoveIt no longer considers part of `arm`), not just cosmetic; effort is closer to S than M since parsing group_state elements is trivial.

### cross-03 — Mobile-grasp pipeline node set duplicated between mobile_grasp.launch.py and mobile_grasp_hw.launch.py

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp_hw.launch.py:135` · package jetank_manipulation · lens minimality · severity low (orig medium) · evidence static · effort S · verdict confirmed

Description: The four pipeline Node() definitions (sock_segmentation_server, grasp_server, base_approach_node, mobile_grasp_coordinator) plus the staggered TimerAction and the detector GroupAction appear verbatim in mobile_grasp.launch.py (lines 99-107, 87-96) and mobile_grasp_hw.launch.py (lines 135-149, 116-132). The only differences are use_sim_time, the grasp_poses.yaml param file (only passed on hw, so sim runs grasp_server WITHOUT grasp_poses.yaml and relies on code defaults) and cmd_vel_topic. Any pipeline change (new node, renamed executable) must be applied in two integration files, and the sim/hw config skew already exists.

Evidence detail: Read both files. mobile_grasp.launch.py line 101-102 `grasp = Node(... parameters=[sim_time])` vs mobile_grasp_hw.launch.py line 137-139 `parameters=[grasp_poses_yaml, not_sim]`. grasp.launch.py (jetank_manipulation) already exists for grasp_server only.

Estimated gain: ~40 lines removed; sim and hw run the same pipeline definition; grasp_poses.yaml applied in sim too

Fix sketch: Add jetank_manipulation/launch/mobile_grasp_pipeline.launch.py declaring use_sim_time and cmd_vel_topic args, starting the four nodes with grasp_poses.yaml; include it from both jetank_ros_main launches (TimerAction wrapper stays in the caller).

Verifier (confirmed, adjusted low): Duplication is real: mobile_grasp.launch.py:99-107 and mobile_grasp_hw.launch.py:135-149 define the same four Node()s, and only the hw side passes grasp_poses.yaml (hw:138 vs sim:101-102). However the claimed skew is currently harmless: every value in jetank_manipulation/config/grasp_poses.yaml matches the declare_parameter defaults in grasp_server.py:393-420, so sim behaves identically today. The est_gain is overstated: the duplicated block is ~9 (sim) + ~14 (hw) lines, and a new mobile_grasp_pipeline.launch.py with args would add roughly as many lines as it removes (net ≈ 0-15 lines), and there is zero runtime/CPU gain on the Jetson since launch descriptions execute once at startup. Honest severity is low: a maintainability item with a latent (not active) config-drift hazard.

### cross-04 — SetParameter+GroupAction wrappers repeated in five places because web_control/detect launch files lack topic args

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:225` · package workspace · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: Because web_control.launch.py declares no `detections_topic` arg and detect.launch.py declares no `detections_topic`/`debug_image_topic` args, every consumer wraps the include in GroupAction([SetParameter(...), IncludeLaunchDescription(...)]): unified.launch.py 225-241, sim_demo.launch.py 162-176 (web) and 188-204 (detector), mobile_grasp.launch.py 87-96, mobile_grasp_hw.launch.py 116-132. Scoped SetParameter also leaks the parameter onto every node inside the include (cmd_vel_bridge gets a `detections_topic` it never declared). Five copies of a 10-line idiom exist to work around two missing DeclareLaunchArguments.

Evidence detail: web_control.launch.py lines 70-84 declare web_port,image_topic,cmd_vel_topic,max_linear,max_angular,sim,output_cmd_vel,nav_cmd_vel only; detect.launch.py lines 34-78 declare no detections_topic/debug_image_topic. Each caller's comment says 'the included launch file declares no launch arg for it'.

Estimated gain: ~35 launch lines removed across 4 files; no parameter leakage to sibling nodes

Fix sketch: Declare `detections_topic` in web_control.launch.py and `detections_topic`,`debug_image_topic` in detect.launch.py (default = node default), forward as node parameters, and replace the five GroupAction/SetParameter wrappers with plain launch_arguments.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-08 — arm/gripper controller parameters maintained in two files marked 'keep in sync'

`/home/koen/workspaces/ros2_ws/src/jetank_motor_control/config/controllers/arm_controller.yaml:10` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: controllers/arm_controller.yaml (lines 12-38) and controllers/gripper_controller.yaml (6-13) are byte-for-byte copies of the arm_controller/gripper_controller sections of jetank_controllers.yaml (lines 41-98), with header comments 'Keep in sync with jetank_controllers.yaml'. The standalone controller_manager path (moveit_bringup.launch.py lines 114-119) loads the per-controller files, the gz_ros2_control path loads the monolithic one. Tolerances/joint lists drift silently.

Evidence detail: Read all three YAMLs; sections identical; moveit_bringup.launch.py 91-119 loads both files (controllers_file + --param-file).

Estimated gain: ~45 duplicated YAML lines removed; single tuning point

Fix sketch: Keep only the `/**`-keyed per-controller files; strip the arm_controller/gripper_controller sections from jetank_controllers.yaml and pass `--param-file` to the spawners in jetank_simulation/gazebo.launch.py (spawner supports it there too), so both paths load the same files.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-12 — world->base_footprint static TF for the MoveIt virtual joint declared in two packages

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:71` · package jetank_moveit_config · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The static_transform_publisher for the SRDF virtual joint (world->base_footprint) is defined in demo.launch.py lines 71-77 and again in jetank_ros_main/unified.launch.py lines 204-213 (with a different node name). moveit_bringup.launch.py docstring (line 14-15) pushes the responsibility to callers, so each caller re-implements it.

Evidence detail: Grep for 'world', 'base_footprint' in launch files returned exactly these two definitions.

Estimated gain: ~10 lines removed; MoveIt bring-up self-contained

Fix sketch: Move the static_transform_publisher into moveit_bringup.launch.py behind a `publish_virtual_joint_tf` arg (default true) and delete both caller copies.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-16 — stereo_camera_config.yaml is bound to the literal 'stereo_camera' namespace that topics.yaml is supposed to own

`/home/koen/workspaces/ros2_ws/src/jetank_perception/config/stereo_camera_config.yaml:4` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The params file is keyed `stereo_camera: stereo_camera_node: ros__parameters:` (lines 4-6). jetank_ros_main passes namespace=camera_namespace() from topics.yaml (unified.launch.py 264, stereo_camera_sim.launch.py 21) to make renames a one-file edit, but a different namespace makes rcl skip the whole YAML (node-name matching) and the node silently falls back to C++ defaults (e.g. GPU_BM, 640x360). The calibration URLs are also declared twice: the YAML uses $(find-pkg-share) (lines 50-52) and stereo_camera.launch.py rebuilds file:// URLs and overrides them (lines 20-23, 154-156), whereas stereo_camera_sim.launch.py does not, so the two launch paths resolve calibration differently.

Evidence detail: YAML header lines 4-6; unified.launch.py line 264 and stereo_camera_sim.launch.py 21 use camera_namespace(); stereo_camera.launch.py 20-23/154-156 vs stereo_camera_sim.launch.py 22-33 read.

Estimated gain: Config applies under any namespace; one calibration-URL source

Fix sketch: Change the YAML root to `/**:` (or `/**/stereo_camera_node:`), drop the launch-side file:// override block and rely on $(find-pkg-share) in the YAML for both real and sim launches.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-19 — stereo_camera_sim.launch.py re-declares the stereo node instead of parameterising stereo_camera.launch.py

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/stereo_camera_sim.launch.py:17` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_ros_main/launch/stereo_camera_sim.launch.py (lines 17-35) duplicates the Node definition from jetank_perception/launch/stereo_camera.launch.py (138-170): same package/executable/name/config path, differing only in five parameter overrides (use_sim_time, camera.use_hardware_acceleration=false, input_source=ros_topics, optical frame ids). The real launch already has left/right_frame_id args; it lacks input_source/use_sim_time/use_hardware_acceleration args, so the sim path forked the whole node block into another package.

Evidence detail: Both files read; parameter sets compared.

Estimated gain: ~35 lines removed; single stereo node definition

Fix sketch: Add `input_source`, `use_sim_time` and `use_hardware_acceleration` launch args to jetank_perception/stereo_camera.launch.py; make stereo_camera_sim.launch.py a 10-line include (or delete it and let mobile_grasp.launch.py include the perception launch with input_source:=ros_topics).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-20 — gazebo_remote/robot_remote launch pair is a stale fork of gazebo.launch.py

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/robot_remote.launch.py:60` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gazebo_remote.launch.py copies the gz_sim include and the 5-line ros_gz_bridge camera argument list from gazebo.launch.py (lines 43-64 vs 86-141); robot_remote.launch.py copies robot_state_publisher, spawn and three spawners (lines 47-98 vs gazebo.launch.py 104-205) but without the OnProcessExit sequencing, the --controller-manager-timeout 60 fix, the gripper controllers and the mimic relay that gazebo.launch.py gained later. Referenced only by the package README and its own import test. Anyone using the remote pair gets the pre-fix startup race.

Evidence detail: Read both remote files and gazebo.launch.py; launch-reference grep shows no other consumer.

Estimated gain: ~190 lines and two stale entry points removed

Fix sketch: Delete gazebo_remote/robot_remote (and their test rows) or reduce them to `gazebo.launch.py spawn_robot:=false` / `gazebo.launch.py start_gz:=false` style flags on the single launch file.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-21 — jetank_motor_control installs a launch/ dir containing a deprecated stub and a broken URDF test launch

`/home/koen/workspaces/ros2_ws/src/jetank_motor_control/launch/test_urdf.launch.py:74` · package jetank_motor_control · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: launch/gazebo_sim.launch.py is a 30-line LogInfo-only deprecation stub (Gazebo Classic). launch/test_urdf.launch.py re-expands the xacro (see cross-05), starts joint_state_publisher_gui (not declared in motor_control package.xml) and loads jetank_description/rviz/urdf.rviz (line 74-75) which does not exist (jetank_description/rviz is empty). CMakeLists installs the whole launch dir (`install(DIRECTORY config launch ...)`). jetank_description/package.xml keeps exec_depends on joint_state_publisher_gui and rviz2 (lines 21,28) only for this file.

Evidence detail: ls jetank_description/rviz returned no files; both launch files and motor_control CMakeLists read; package.xml deps listed.

Estimated gain: 114 lines, one install rule and two rosdep keys removed

Fix sketch: Delete jetank_motor_control/launch entirely, drop `launch` from the install(DIRECTORY ...) line, and remove joint_state_publisher_gui/rviz2 from jetank_description/package.xml (or move a working urdf.rviz there if RViz preview is wanted).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-23 — single_camera/simple_camera launch files and the camera_node executable are unused template leftovers

`/home/koen/workspaces/ros2_ws/src/jetank_perception/launch/single_camera.launch.py:112` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: single_camera.launch.py still contains template text ('Replace with your actual package name', line 76-77) and an RViz node pointing at '/path/to/your/rviz/config.rviz' (line 112); simple_camera.launch.py is a second copy of the same node with different hardcoded values (lines 22-38). Neither is included by any bring-up (launch-reference grep: only jetank_perception/README.md). They exist for the `camera_node` executable (src/single_camera.cpp, 162 lines) which CMake builds and links against OpenCV+Threads (CMakeLists add_executable(camera_node ...)) on every build although no launch or task uses it.

Evidence detail: Both launch files read; grep for single_camera/simple_camera across .py/.md/.toml found only the perception README.

Estimated gain: 214 launch lines removed; one fewer C++ target (OpenCV link) per build on the Orin

Fix sketch: Delete simple_camera.launch.py; either fix single_camera.launch.py (real rviz path, drop template comments) or remove it together with the camera_node target if single-CSI capture is not a supported mode.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-24 — navigation.rviz is a strict subset of jetank_ros_main's unified.rviz

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/rviz/navigation.rviz:1` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: navigation.rviz (79 lines) contains RobotModel, TF, LaserScan /scan, Map, PointCloud2 /stereo_camera/points, two Paths, two Polygons and /particle_cloud; unified.rviz (280 lines, jetank_ros_main) contains every one of those displays plus images/odometry. navigation_full.launch.py loads navigation.rviz through nav2_bringup's rviz_launch (lines 123-135) while sim_demo/rviz.launch.py load unified.rviz, so two RViz layouts must be kept aligned whenever a topic is renamed.

Evidence detail: Display/topic extraction from both files (grep counts in this session) shows navigation.rviz's display set is contained in unified.rviz's.

Estimated gain: 79 lines removed; one RViz layout to update on topic changes

Fix sketch: Delete navigation.rviz and have navigation_full.launch.py include jetank_ros_main/launch/rviz.launch.py (unified.rviz), or move unified.rviz to jetank_navigation if the dependency direction is preferred.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-26 — Training-image capture implemented twice with incompatible on-disk layouts

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/capture_frames.py:46` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_detection/capture_frames.py (142 lines) subscribes to the left image and writes ~/datasets/detection/sim/sock_<domain>_NNNNNN.jpg (lines 46-61, 105-118). jetank_web_control/web_control_node.py implements a second capture path (save_capture lines 624-660, list/label helpers 511-537, 662-700) writing <timestamp>_NNNN.jpg plus YOLO .txt sidecars and classes.txt into ~/datasets/detection. jetank_detection/scripts/prepare_dataset.py (docstring lines 4-6) only understands the web-labeller layout and skips images without sidecars, so capture_frames output is unusable by the package's own dataset tool without manual relabelling.

Evidence detail: Read capture_frames.py, web_control_node.py 300-700 and prepare_dataset.py header.

Estimated gain: One capture implementation (~140 lines removed) and one dataset layout

Fix sketch: Drop capture_frames.py (and its install rule) in favour of the web capture, or make capture_frames write the web-labeller layout (flat dir, same naming, empty .txt optional) and have web_control call a shared jetank_detection.dataset helper for naming.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-27 — Map saving wrapped twice (save_map.sh and web_control_node.save_map)

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/scripts/save_map.sh:37` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: scripts/save_map.sh (installed to lib/jetank_navigation, CMakeLists install(PROGRAMS scripts/save_map.sh)) and web_control_node.save_map (lines 1103-1121) both shell out to `ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>`; the shell version lacks the use_sim_time/save_map_timeout handling the Python one needed. Two wrappers for one upstream CLI, in two packages.

Evidence detail: Both read this session.

Estimated gain: 57 lines and one installed script removed

Fix sketch: Delete save_map.sh and document the map_saver_cli command (or the web Save Map button) in the navigation README.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-29 — Boolean launch-arg idioms and use_sim_time declarations re-implemented across 23 launch files

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/unified.launch.py:209` · package workspace · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: `IfCondition(PythonExpression(["'", cfg, "' == 'true'"]))` appears 8 times (unified.launch.py 209-212, 225-227, 319-323, 332-336; sim_demo 118-119; stereo_camera.launch.py 188-190, 205-207) although IfCondition(LaunchConfiguration(x)) already accepts 'true'/'false'; `.perform(context).lower() in ('true','1')` is copied in web_control.launch.py 24, moveit_sim.launch.py 72-73 and nav2_bringup.launch.py 33-35. DeclareLaunchArgument('use_sim_time') is written 18 times with three different defaults ('false', 'False', 'true'). Each PythonExpression is an eval() at launch-parse time.

Evidence detail: grep counts in this session: 8 '== \'true\'' matches, 3 lower()-in matches, 18 use_sim_time declarations across 23 launch files.

Estimated gain: ~40 lines simplified; consistent use_sim_time default spelling

Fix sketch: Replace the PythonExpression equality idiom with IfCondition(LaunchConfiguration(...)) where the arg is already a bool string; normalise use_sim_time defaults to 'false'; for the combined slam/nav2 conditions use LaunchConfigurationEquals.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **jetank_ros_main-21** — same PythonExpression == true idiom at unified.launch.py:209-211/225-227 where IfCondition(LaunchConfiguration) suffices.

### cross-30 — pixi.toml and 1.2 MB pixi.lock duplicated verbatim in jetank_ros_main/workspace_template; pixi deps unused by any source

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/workspace_template/pixi.toml:1` · package jetank_ros_main · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: workspace_template/pixi.toml and pixi.lock are byte-identical to the workspace root copies (diff empty; both locks 1,235,882 bytes), so every environment change must be applied twice and the lock file is versioned inside a ROS package repo. The shared dependency list also pulls packages no source uses: `libgpiod` (root pixi.toml line 132; grep for gpiod in all .cpp/.hpp/CMakeLists/package.xml returns nothing - motor.cpp drives a PCA9685 over I2C), `ros-humble-moveit-servo` (line 123; no reference in src), `ros-humble-joint-state-publisher-gui` (line 106; only the stale test_urdf launch).

Evidence detail: `diff` of the two pixi.toml files produced no output; `ls -la` sizes identical; greps quoted.

Estimated gain: 1.2 MB removed from the package repo; three conda packages fewer in the ~6 GB env

Fix sketch: Keep pixi.toml/pixi.lock only at the workspace root (have install.sh copy or symlink from a single source), and drop libgpiod, moveit-servo and joint-state-publisher-gui from [dependencies] unless a concrete consumer is added.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-31 — package.xml exec_depends declared for tools no launch or source uses

`/home/koen/workspaces/ros2_ws/src/jetank_perception/package.xml:46` · package workspace · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_perception declares launch_xml/launch_yaml (lines 46-47; all launches are .py) and gstreamer1.0-plugins-* (40-41; GStreamer comes through the system OpenCV build, not rosdep); jetank_simulation declares ros_gz_image (line 15; unused, only ros_gz_bridge is launched); jetank_description declares joint_state_publisher_gui and rviz2 (21,28) used only by motor_control's stale test_urdf; jetank_motor_control declares ign_ros2_control (47), xacro, joint_state_publisher, robot_state_publisher (50-52) which belong to simulation/description launches, not to the motor package; jetank_ros_main declares launch_xml/launch_yaml (22-23). rosdep resolves each on a fresh Jetson install.

Evidence detail: package.xml dependency listing for all 10 packages compared with the launch/source files read this session.

Estimated gain: ~10 rosdep keys fewer on a clean install; honest package graph

Fix sketch: Remove the listed exec_depends; keep ign_ros2_control only in jetank_simulation, xacro/robot_state_publisher only in jetank_description.

Verifier (unverified, adjusted low): low severity, not sent to verifier

Duplicate folded in: **footprint-20** — same launch_xml/launch_yaml exec_depends at jetank_perception/package.xml:46-47 and jetank_ros_main/package.xml:22-23.

### cross-32 — Executor/QoS boilerplate copied between packages instead of one shared helper

`/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:490` · package jetank_detection · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: sock_detector_node.py main() (lines 490-502) is the same rclpy.init/MultiThreadedExecutor/spin/destroy/shutdown block that jetank_manipulation extracted into node_runner.spin_node (39 lines); cmd_vel_bridge.py 193-203 and capture_frames.py 125-140 carry single-threaded variants with different shutdown guards. The transient-local 'latched' QoSProfile is built inline in web_control_node.py 439-445 and again in jetank_manipulation/action_utils.latched_qos (33-44). Because jetank_manipulation depends on jetank_detection (not the reverse), the helper sits in the wrong package to be reused.

Evidence detail: main() grep output across all Python nodes and TRANSIENT_LOCAL grep, this session.

Estimated gain: ~40 lines removed; consistent shutdown semantics across nodes

Fix sketch: Move node_runner.spin_node and latched_qos into jetank_detection (already the lowest common Python dependency: manipulation and web_control can import it) and use them from sock_detector_node, cmd_vel_bridge, capture_frames and web_control_node.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-33 — Installed test_drive/test_cameras console scripts target a control interface the sim no longer exposes

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:21` · package jetank_ros_main · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: setup.py installs `test_drive` and `test_cameras` console_scripts. test_drive.py publishes geometry_msgs/Twist on /cmd_vel 'to verify diff_drive_controller works' (docstring line 5, publisher line 21), but the sim controller subscribes TwistStamped on /diff_drive_controller/cmd_vel (jetank_controllers.yaml, sim_demo.launch.py docstring 30-34), so the script cannot drive the sim; on hardware it duplicates what web control does. test_cameras.py hardcodes the four stereo topics (21-41) outside the topic contract. 419 lines of stale tooling shipped in the integration package.

Evidence detail: Files read (heads) and setup.py entry_points; controller topic from jetank_controllers.yaml and sim_demo docstring.

Estimated gain: 419 lines and two console entry points removed (or one working smoke test)

Fix sketch: Delete both scripts and their entry points, or rewrite test_drive to publish through cmd_vel_bridge's output type and read topic names from jetank_ros_main.topics.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-34 — Docs and root CLAUDE.md describe a PointCloud2->LaserScan node and laser_data.yaml that no longer exist

`/home/koen/workspaces/ros2_ws/src/jetank_navigation/CMakeLists.txt:10` · package jetank_navigation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_navigation/src contains only icm20948_node.cpp and config/ has no laser_data.yaml (ls/wc this session), yet the workspace CLAUDE.md ('PointCloud2 to LaserScan conversion node', 'config/laser_data.yaml'), PROGRESS.md, TROUBLESHOOTING.md and LEARNING_PLAN.md still document that node (grep hits). Agents and readers are steered to non-existent files.

Evidence detail: grep for laser_geometry|pointcloud_to_laserscan|laser_data.yaml matched only the three navigation docs and ../CLAUDE.md; src listing shows one .cpp.

Estimated gain: Accurate navigation docs; no dead references for tooling

Fix sketch: Update CLAUDE.md's jetank_navigation section and the three docs to describe the actual contents (IMU driver, RPLidar/Nav2/SLAM launches and configs).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-35 — ros2_control.xacro lives in jetank_motor_control, forcing jetank_description to depend on the C++ hardware package

`/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/jetank_ros2_control.urdf.xacro:26` · package jetank_description · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: The canonical URDF includes `$(find jetank_motor_control)/config/ros2_control.xacro` (line 26), so jetank_description/package.xml must exec_depend on jetank_motor_control (line 17) and any consumer that only wants geometry (RViz, sim, MoveIt on a laptop) must build the hardware package (rclcpp_lifecycle, pluginlib, hardware_interface, the serial plugin) first. The xacro only references plugin names as strings; it is description data, not motor code.

Evidence detail: URDF line 26 and jetank_description/package.xml line 17 read; motor_control CMakeLists shows the C++ library targets.

Estimated gain: jetank_description buildable/usable standalone; removes a description->hardware package edge

Fix sketch: Move config/ros2_control.xacro to jetank_description/urdf/components/ros2_control.xacro, update the include and drop the jetank_motor_control exec_depend (motor_control keeps its plugin XML).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-37 — World-name to SDF mapping owned by jetank_ros_main instead of the simulation package that ships the worlds

`/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/gazebo_sim.launch.py:39` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_ros_main/launch/gazebo_sim.launch.py exists only to map 'empty|simple_test|obstacle_course|sock_arena|house' to files under jetank_simulation/worlds (lines 39-45) and forward three args through five conditional includes (57-69); its comment even says 'the sim package owns worlds'. sim_demo and mobile_grasp go through this wrapper to reach jetank_simulation/gazebo.launch.py, so adding a world requires editing two packages.

Evidence detail: Read gazebo_sim.launch.py, gazebo.launch.py, worlds/ listing (5 sdf files matching the map).

Estimated gain: 71 lines and one wrapper removed; worlds registered where they live

Fix sketch: Add a `world_name` arg to jetank_simulation/gazebo.launch.py that resolves to `<share>/worlds/<name>.sdf` (PathJoinSubstitution) when `world` is not given, then delete jetank_ros_main/gazebo_sim.launch.py and include gazebo.launch.py directly.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-38 — Empty config/model/map directories installed and documented as configuration locations

`/home/koen/workspaces/ros2_ws/src/jetank_description/CMakeLists.txt:8` · package workspace · lens footprint · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: jetank_description/config, jetank_simulation/config, jetank_simulation/models and jetank_navigation/maps are empty directories (ls this session) that the CMake install loops (jetank_description CMakeLists foreach urdf meshes launch config; jetank_simulation foreach worlds launch config models; jetank_navigation install maps OPTIONAL) still install, and the workspace CLAUDE.md lists src/jetank_description/config and src/jetank_simulation/config as main configuration files.

Evidence detail: Directory listings returned no files; CMakeLists install loops read.

Estimated gain: Fewer misleading paths for readers/agents; trivial install cleanup

Fix sketch: Remove the empty directories (git does not track them anyway) and the corresponding CLAUDE.md bullets, or add the intended files.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-39 — gazebo_headless.launch.py is a compatibility wrapper that re-declares three arguments to pass gui:=false

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_headless.launch.py:40` · package jetank_simulation · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: gazebo_headless.launch.py (72 lines) re-declares use_sim_time/world/start_arm_active with copied defaults (40-55, including the world path computed again) and includes gazebo.launch.py with gui:=false. Its only non-doc consumer is moveit_sim.launch.py (line 77) which already computes `headless` as a bool and could pass gui directly. jetank_ros_main/gazebo_sim.launch.py already forwards `gui` to gazebo.launch.py.

Evidence detail: Launch-reference grep: gazebo_headless referenced by moveit_sim.launch.py, docs, the deprecated motor_control stub and its own test.

Estimated gain: 72 lines and one entry point removed

Fix sketch: In moveit_sim.launch.py include gazebo.launch.py with gui:=<not headless>; delete gazebo_headless.launch.py and update the README/test rows.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-05 — robot_description xacro expansion implemented four different ways across four packages

`/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:52` · package workspace · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: jetank_description/launch/robot_description.launch.py is documented as the canonical include (its docstring lines 1-16), yet: jetank_moveit_config/demo.launch.py lines 52-68 runs `Command(['xacro ', file, ' hardware:=', hardware])` itself (and omits use_sim/use_ros2_control mappings); jetank_motor_control/launch/test_urdf.launch.py lines 35-58 runs its own Command with use_sim:=true; jetank_simulation/launch/jetank_sim_description.py lines 18-33 does an in-process xacro.process_file plus a sys.path hack (gazebo.launch.py 17-21, robot_remote.launch.py 24-28) to share it. Each copy encodes the xacro path and arg names again; a renamed xacro arg breaks three packages silently. Every extra xacro expansion is ~0.5-1 s of Python on the Orin at launch time.

Evidence detail: Read robot_description.launch.py, demo.launch.py, test_urdf.launch.py, jetank_sim_description.py, gazebo.launch.py, robot_remote.launch.py. Only jetank_ros_main/urdf.launch.py (line 30-38) uses the canonical include.

Estimated gain: ~60 lines removed; one place for xacro path/args; drops the sys.path import hack in jetank_simulation

Fix sketch: demo.launch.py and gazebo.launch.py/robot_remote.launch.py include jetank_description/robot_description.launch.py with use_sim:=true (it already starts robot_state_publisher); delete jetank_sim_description.py and test_urdf.launch.py. MoveItConfigsBuilder keeps its own expansion (needed for move_group params).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-15 — Topic contract (topics.yaml) covers 4 topics; a dozen cross-package topic/action names remain hardcoded, some not even parameters

`/home/koen/workspaces/ros2_ws/src/jetank_perception/src/sock_segmentation_server.cpp:134` · package workspace · lens minimality · severity low (orig medium) · evidence static · effort M · verdict confirmed

Description: topics.yaml/topics.py centralise only the left image, its compressed variant and /detections/socks(+debug). Other producer/consumer pairs are string literals: sock_segmentation_server.cpp lines 134,140,146 subscribe '/stereo_camera/disparity', '/stereo_camera/left/camera_info', '/detections/socks' as non-parameter literals (renaming camera_namespace in topics.yaml breaks segmentation silently); stereo_camera_node.cpp 321-324 ros_input defaults; web_control.launch.py 27-28 re-hardcodes both camera topics that topics.yaml and web_control_node.py 336 already define (three copies); the sim controller topic '/diff_drive_controller/cmd_vel' is in base_approach_node.py 149, cmd_vel_bridge.py 61 and web_control.launch.py 80; '/gripper_controller/gripper_cmd' in grasp_server.py 51 and grasp_poses.yaml 15; action names in mobile_grasp_coordinator.py 94-96; test_cameras.py 21-41. topics.py itself re-parses the YAML file on every accessor call (_load() at lines 27-46 called by each of 6 helpers) instead of caching.

Evidence detail: grep of topic-like string literals across all node sources (this session) listed above; topics.yaml lines 29-45 read.

Estimated gain: Camera/controller renames become one-file edits; segmentation server configurable; removes ~10 duplicated literals

Fix sketch: Make the three segmentation-server topics declared parameters (defaults from the contract); extend topics.yaml with disparity, camera_info, sim cmd_vel and the three manipulation action names, and add matching accessors; have web_control.launch.py fall back to the node default (empty image_topic -> not passed) instead of re-hardcoding; cache _load() with functools.lru_cache.

Verifier (confirmed, adjusted low): Facts verified: sock_segmentation_server.cpp:134/140/146 subscribe '/stereo_camera/disparity', '/stereo_camera/left/camera_info', '/detections/socks' as non-parameter literals; '/diff_drive_controller/cmd_vel' is duplicated in cmd_vel_bridge.py:61, base_approach_node.py:149, web_control.launch.py:80 and sim_demo.launch.py:34; web_control.launch.py:27-28 re-hardcodes both camera topics; topics.py:51-76 calls _load() (YAML re-parse) in every accessor. However the est_gain is purely maintainability (rename safety, ~10 fewer literals), not a Jetson CPU/memory/latency gain: topics.py accessors are only imported by five launch files (unified/mobile_grasp/sim_demo/etc.), so the redundant YAML parse costs a few ms once at launch, never in a node hot path. Severity honest at low given zero runtime impact; the segmentation-server non-parameter literals are the only functional risk (silent break on namespace rename).

### cross-25 — DetectSocks action has no client in the workspace; every integration launch runs the detector in continuous mode

`/home/koen/workspaces/ros2_ws/src/jetank_detection/action/DetectSocks.action:1` · package jetank_detection · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: DetectSocks.action is generated by rosidl (CMakeLists rosidl_generate_interfaces) and served by sock_detector_node.py (server + goal/cancel/execute code, lines 173-430), but grep finds no client anywhere (only docstrings and the server). sim_demo.launch.py 199, mobile_grasp.launch.py 93 and mobile_grasp_hw.launch.py 126 all pass continuous:=true; the 3D path uses SegmentSocks instead. The action still costs C++/Python typesupport generation on every build and ~130 lines of server code in the node.

Evidence detail: grep -i detect_socks/DetectSocks across .py/.cpp/.js/.html: matches only in sock_detector_node.py and launch docstrings.

Estimated gain: One rosidl interface fewer (build time), ~130 node lines removable if on-demand mode is confirmed dead

Fix sketch: Confirm no external client is planned; if so remove DetectSocks from rosidl_generate_interfaces and the action-server code paths, leaving continuous mode as the single behaviour (or keep the action but drop the `continuous` toggle so there is one code path).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### cross-28 — web_control_node hardcodes jetank_navigation's lifecycle node names and launch files for pkill/launch management

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1014` · package jetank_web_control · lens minimality · severity low · evidence static · effort M · verdict **UNVERIFIED**

Description: _NAV_PROC_PATTERNS (lines 1014-1018) duplicates the lifecycle node list that jetank_navigation/launch/nav2_bringup.launch.py owns (lines 63-81) and _launch_nav (1020-1032) issues `pkill -9 -f <name>` for each of 11 patterns plus a 1 s sleep before every start, then spawns `ros2 launch jetank_navigation <file>`. Adding/renaming a Nav2 node in jetank_navigation requires editing jetank_web_control; `pkill -f amcl`/`map_server` also matches unrelated processes. Eleven pkill process spawns per mode switch on the Orin.

Evidence detail: Lines quoted from web_control_node.py; nav2_bringup.launch.py lifecycle_nodes lists read.

Estimated gain: Removes 11 subprocess spawns + 1 s per nav mode switch; one owner for the node list

Fix sketch: Rely on stop_nav's process-group SIGINT/SIGKILL (already implemented, lines 1071-1093) and drop the pattern list; if lingering nodes remain, call /lifecycle_manager_navigation/manage_nodes (shutdown) before killing, keeping node names inside jetank_navigation.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## workspace-footprint

Coverage: 58 files read; 12 measurements run. Notes: All 10 package.xml files, all 8 CMakeLists.txt, both setup.py/setup.cfg pairs and pixi.toml were read in full; pixi.lock was only grepped/aggregated (never dumped). Cross-checks used #include lists of every .cpp/.hpp in the three C++ packages, import lists of every node/launch .py, and launch package=/executable= greps. build/, install/, log/, .pixi/ were not opened, so no ldd/size measurements of built binaries exist; findings 12/13 (over-linking) are therefore static. Not reported as findings but noted: ghostscript (66 MB) arrives via nav2-map-server->graphicsmagick and vtk-base+viskores (88 MB) via conda pcl — both unavoidable while nav2/PCL are used from RoboStack; pybullet/bullet (105 MB) via moveit-core is likewise structural. No merges/deletions of packages or of the perception strategy/factory abstractions were proposed. The 2m03s/10-package build was not re-run (read-only constraint), so per-finding build-time gains are estimates except where the Jan-2026 build_output.log number is cited.

Measurements:
- du -sh src/*/ (with .git): description 1.7M, detection 1.6M, manipulation 1.6M, motor_control 1.9M, moveit_config 1.4M, navigation 2.2M, perception 3.0M, ros_main 4.8M, simulation 1.8M, web_control 3.7M; excluding .git: 164K/408K/772K/188K/188K/272K/380K/2.0M/336K/852K
- pixi.lock (Python aggregation, aarch64+noarch): 915 packages, 1324.7 MB compressed; dependency closure of pixi.toml top-level deps: 906 packages / 1274.2 MB compressed
- pixi.lock top packages by compressed size: ogre 116.2 MB, gcc_impl 68.6, ghostscript 66.4 (via graphicsmagick<-nav2-map-server), vtk-base 64.7 (via pcl<-pcl-conversions), pybullet 63.1 + bullet-cpp 42.3 (via bullet<-moveit-core), qt6-main 59.4, qt-main 51.9, libllvm21 43.1 + libllvm22 43.1 (via qt), viskores 22.9, libopencv 22.7, pcl 17.3, ruby 17.8 + dartsim 13.5 (via libignition)
- pixi.lock closure deltas: drop ros-gz-sim/bridge/image + ign-ros2-control = -25 pkgs / -175.0 MB; swap ros-humble-desktop -> ros-base + rviz2 = -72 pkgs / -10.3 MB; drop gstreamer+gst-plugins-* = -4 pkgs / -3.5 MB; drop moveit-servo = -1.0 MB; drop libgpiod = -0.1 MB; drop ninja = -0.2 MB; drop navigation2+nav2-bringup = -38 pkgs / -12.0 MB (not proposed; nav2 is used)
- pixi.lock: libopencv-4.13.0-qt6_py312h2034ceb_606 depends (39 entries) contains no gstreamer; 0 packages matching cuda/ultralytics/torch/ccache; aiohttp reachable only via wslink<-vtk-base<-pcl; pillow only via matplotlib-base<-rqt-plot<-rqt-common-plugins<-desktop
- grep -rn -i gpiod src/ (excl .git): 0 hits; grep moveit_servo: 0; grep ros_gz_image/image_bridge in launches: 0; grep 'image_transport::' stereo_camera_node.cpp: 1 hit (unused member at line 120); grep 'package://' in xacro: 0; meshes/, models/, config/ (desc+sim), maps/ directories: empty
- wc -l: stereo_camera_node.cpp 2019, sock_segmentation_server.cpp 741, stereo_processing_strategy.hpp 655, camera_interface.hpp 442; C++ total 6823 lines across 3 compiled packages
- build_output.log (ws root, Jan 12 2026, system-ROS build of one package): jetank_perception finished in 36.5 s
- __pycache__ inside installable launch dirs: navigation/launch 64K, simulation/launch 48K, moveit_config/launch 32K, detection/launch 28K, manipulation/launch 8K (cpython-310 and cpython-312 .pyc both present); test/__pycache__ (not installed): web_control 328K, manipulation 308K, detection 108K
- Host: nproc = 6; free -m total 7619 MB, available 3548 MB
- git -C src/jetank_ros_main ls-files: workspace_template/pixi.lock (1,235,882 B, same size as root pixi.lock), 12 plans/*.md, docs/images/jetank_real.jpg (125,582 B) are tracked
- Live ROS MCP measurement tools were not invoked: this lens (build/dependency footprint) needed no topic/node measurements; no tool failures to report

Findings: 21 (high 1, medium 5, low 15).

| id | lens | title | file:line | severity | evidence | gain | effort | verdict |
|---|---|---|---|---|---|---|---|---|
| footprint-01 | footprint | Gazebo/Ignition sim stack installed unconditionally on the Jetson (175 MB compressed, 25 pkgs) | `/home/koen/workspaces/ros2_ws/pixi.toml:111` | high | measured | -175 MB compressed download (~500+ MB installed) and 25 fewer packages on the Jetson; faster pixi install/solve | M | confirmed |
| footprint-02 | footprint | ros-humble-desktop metapackage pulls demos, turtlesim, rqt, tutorials never used | `/home/koen/workspaces/ros2_ws/pixi.toml:73` | medium | measured | -72 packages / -10 MB compressed; fewer pixi solve constraints | S | confirmed |
| footprint-09 | minimality | Runtime deps reach the env only transitively; pixi.toml under-declares what src/ imports | `/home/koen/workspaces/ros2_ws/pixi.toml:127` | medium | measured | No size change; prevents breakage when trimming and makes install reproducible | S | confirmed |
| footprint-28 | minimality | exec_depend on jetank_mission (absent from workspace) while aiohttp/PIL/numpy/cv2 go undeclared | `/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:20` | medium | measured | Correct rosdep; prevents silent feature loss when trimming pixi env | S | confirmed |
| footprint-10 | footprint | Every `pixi run build` compiles and links the gtest test binaries (BUILD_TESTING defaults ON) | `/home/koen/workspaces/ros2_ws/pixi.toml:24` | medium | static | Roughly 3 of 10 heavy C++ TUs and 3 link steps skipped per full build; estimated 15-30% of the 2m03s wall time | S | confirmed |
| footprint-12 | footprint | find_package(PCL) without COMPONENTS links every PCL module (incl. visualization/VTK) into all perception targets | `/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:35` | medium | static | Fewer shared objects mapped at node start (tens of libs), shorter link step; no source change | S | confirmed |
| footprint-03 | footprint | gstreamer / gst-plugins-base / gst-plugins-good in pixi env cannot be used by the pixi OpenCV | `/home/koen/workspaces/ros2_ws/pixi.toml:118` | low (orig medium) | measured | -3.5 MB compressed, 4 packages; removes a misleading 'GStreamer works in pixi' assumption | S | confirmed |
| footprint-04 | minimality | libgpiod declared in pixi.toml but no source uses GPIO | `/home/koen/workspaces/ros2_ws/pixi.toml:117` | low | measured | -1 package / 0.1 MB; removes a stale claim | S | **UNVERIFIED** |
| footprint-05 | minimality | ros-humble-moveit-servo unused anywhere in src/ | `/home/koen/workspaces/ros2_ws/pixi.toml:108` | low | measured | -1.0 MB compressed, fewer solve constraints | S | **UNVERIFIED** |
| footprint-06 | minimality | ros-humble-joint-state-publisher-gui only used by a dev-only launch (pulls pyqt/Qt5 python binding) | `/home/koen/workspaces/ros2_ws/pixi.toml:82` | low | measured | -1 package alone; with footprint-02, drops pyqt (6.4 MB) and python-qt-binding | S | **UNVERIFIED** |
| footprint-11 | footprint | No parallel-worker / job cap or ccache for a 6-core, 7.6 GB Jetson | `/home/koen/workspaces/ros2_ws/pixi.toml:24` | low (orig medium) | measured | Avoids swap stalls on full rebuilds; ccache makes header-touch rebuilds seconds instead of ~30 s+ | S | **UNCERTAIN** |
| footprint-19 | minimality | Raw calibration dump ost.txt shipped in share/ | `/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:201` | low | measured | ~1 KB; hygiene | S | **UNVERIFIED** |
| footprint-22 | minimality | gripper_controllers exec_depend unused; ign_ros2_control exec_depend drags Gazebo onto a hardware package | `/home/koen/workspaces/ros2_ws/src/jetank_motor_control/package.xml:39` | low | measured | One fewer rosdep; sim stack no longer required to satisfy the hardware package | S | **UNVERIFIED** |
| footprint-23 | minimality | Deprecated Gazebo-Classic stub launch still installed | `/home/koen/workspaces/ros2_ws/src/jetank_motor_control/launch/gazebo_sim.launch.py:21` | low | measured | 1 KB; removes a misleading entry point | S | **UNVERIFIED** |
| footprint-24 | minimality | install(DIRECTORY launch) without __pycache__ EXCLUDE ships stale .pyc files (7 packages) | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:18` | low | measured | ~180 KB and deterministic install trees | S | **UNVERIFIED** |
| footprint-25 | minimality | Empty models/, config/, meshes/, maps/ directories installed as share subdirs | `/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:16` | low | measured | Hygiene only; removes phantom share dirs and false docs | S | **UNVERIFIED** |
| footprint-34 | minimality | C++17 requested three ways and CUDA path keyed on OPENCV_ENABLE_NONFREE | `/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:5` | low | measured | Clarity; no size change | S | **UNVERIFIED** |
| footprint-35 | minimality | Template scaffold launch file installed (simple_camera.launch.py) | `/home/koen/workspaces/ros2_ws/src/jetank_perception/launch/simple_camera.launch.py:23` | low | measured | 1.4 KB; one fewer confusing entry point | S | **UNVERIFIED** |
| footprint-14 | footprint | opencv.hpp umbrella header in 5 files plus a 2019-line node TU dominate compile time | `/home/koen/workspaces/ros2_ws/src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:3` | low (orig medium) | measured | Estimated 20-40% shorter perception compile (the package is the longest of the 10) | M | confirmed |
| footprint-07 | minimality | ninja installed but colcon build tasks never select the Ninja generator | `/home/koen/workspaces/ros2_ws/pixi.toml:56` | low | static | Either -0.2 MB or faster incremental rebuild checks (seconds per package) | S | **UNVERIFIED** |
| footprint-15 | minimality | Directory-wide include_directories/link_directories/add_definitions apply PCL+yaml-cpp flags to camera_node | `/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:59` | low | static | Marginal compile-time (shorter include search), cleaner target graph | S | **UNVERIFIED** |

### footprint-01 — Gazebo/Ignition sim stack installed unconditionally on the Jetson (175 MB compressed, 25 pkgs)

`/home/koen/workspaces/ros2_ws/pixi.toml:111` · package workspace · lens footprint · severity high · evidence measured · effort M · verdict confirmed

Description: pixi.toml:111-114 lists ros-humble-ros-gz-sim, ros-gz-bridge, ros-gz-image and ign-ros2-control in the single default [dependencies] table, and platforms (line 13) includes linux-aarch64, so every `pixi install` on the Orin Nano downloads and unpacks the full Ignition Fortress stack (ogre 116 MB, ruby 18 MB, dartsim 13.5 MB, libignition-gazebo6 12.6 MB, qt5 chain). Nothing in src/ that runs on the robot needs it: only jetank_simulation launches reference ros_gz_sim/ros_gz_bridge. This is the single largest removable chunk of the env on the robot (disk, install time, and pixi solve time).

Evidence detail: Python dependency-closure over pixi.lock (aarch64+noarch): removing the four ros-gz/ign top-level deps drops 25 packages / 175.0 MB compressed; biggest: ogre 116.2, ruby 17.8, dartsim 13.5, libignition-gazebo6 12.6, libignition-rendering6 4.5. grep of launch files: only package='ros_gz_sim' (2) and 'ros_gz_bridge' (3) in jetank_simulation/jetank_ros_main launches.

Estimated gain: -175 MB compressed download (~500+ MB installed) and 25 fewer packages on the Jetson; faster pixi install/solve

Fix sketch: Move the four ros-gz/ign deps into `[feature.sim.dependencies]` and add `[environments] default = [] ; sim = ["sim"]` (or `sim = { features = ["sim"], solve-group = "default" }`). Keep `pixi run gazebo` bound to the sim env (`[tasks.gazebo] env = "sim"` style) and install only the default env on the Jetson.

Verifier (confirmed, adjusted high): pixi.toml:13 lists platforms ["linux-aarch64","linux-64"] and pixi.toml:126-129 put ros-humble-ros-gz-sim/-bridge/-image and ign-ros2-control in the single default [dependencies] table with no [feature]/[environments] split (grep found none). pixi.lock sizes for aarch64 match the claim: ogre 116.2 MB, ruby 17.8 MB, dartsim 13.5 MB, libignition-gazebo6 12.6 MB. Usages are sim-only: package.xml exec_depends in jetank_simulation:13-15,36 and jetank_motor_control:47 (gated by use_sim:=true in config/ros2_control.xacro:31,228), plus ros_gz launches in jetank_simulation and gazebo_sim.launch.py; nothing on the hardware path needs them. Severity kept high since it is the largest removable chunk on the Jetson.

### footprint-02 — ros-humble-desktop metapackage pulls demos, turtlesim, rqt, tutorials never used

`/home/koen/workspaces/ros2_ws/pixi.toml:73` · package workspace · lens footprint · severity medium · evidence measured · effort S · verdict confirmed

Description: ros-humble-desktop (line 73) expands to 56 direct deps including demo-nodes-cpp/py, turtlesim, action-tutorials, examples-rclcpp-*, pendulum-control, intra-process-demo, image-tools, rqt-common-plugins (→ rqt-plot → matplotlib → pillow; rqt-graph → pydot → graphviz → gtk3/cairo/icu), dummy-robot-bringup, depthimage-to-laserscan, joy. Nothing in src/ references any of those. What the workspace actually uses from desktop is ros-base + rviz2 (5 launch files start rviz2).

Evidence detail: pixi.lock closure: swapping ros-humble-desktop for ros-humble-ros-base + ros-humble-rviz2 removes 72 packages / 10.3 MB compressed (demo-nodes-cpp 0.9, turtlesim 0.7, qos-demo 0.6, examples-* , rqt-image-view, pendulum-control, intra-process-demo...). desktop depends list read from lock. grep for rviz2 in launches: 5 files. Caveat measured too: pillow reaches the env only via matplotlib←rqt-plot←desktop, and web_control_node.py:72-76 silently disables map PNG rendering when PIL is missing.

Estimated gain: -72 packages / -10 MB compressed; fewer pixi solve constraints

Fix sketch: Replace `ros-humble-desktop = "*"` with `ros-humble-ros-base = "*"` + `ros-humble-rviz2 = "*"` (+ `ros-humble-rviz-default-plugins` if not implied), and add `pillow`, `numpy`, `pyyaml` explicitly (see footprint-09) so web_control keeps working.

Verifier (confirmed, adjusted medium): The fix is in-scope (only pixi.toml:80, no package merge or perception abstraction change) and safe for every build/exec dep declared in src/*/package.xml: pixi.lock shows all transitively-needed packages (tf2-sensor-msgs, message-filters, laser-geometry, interactive-markers, rviz-default-plugins, sensor-msgs-py, vision-msgs) are still pulled by ros-base/geometry2, nav2, slam-toolbox, moveit, ros-gz-bridge or the explicit perception deps, and the lock confirms pillow reaches the env only via matplotlib-base/rqt-bag-plugins (so the fix_sketch's explicit `pillow` addition is required for web_control_node.py:72-76). One gap in the fix_sketch: `teleop_twist_keyboard` is a desktop-only dep that is documented as a user workflow (src/jetank_ros_main/SIM_CONTROL.md:55, src/jetank_simulation/SIM_TESTING.md:97, sim_demo.launch.py:32 docstring), so `ros-humble-teleop-twist-keyboard` should be added explicitly or that documented command breaks. Note the finding's cited line 73 is wrong; `ros-humble-desktop` is at pixi.toml:80. The `image_view` node in single_camera.launch.py:124 is not in the lock at all today, so it is unaffected.

### footprint-09 — Runtime deps reach the env only transitively; pixi.toml under-declares what src/ imports

`/home/koen/workspaces/ros2_ws/pixi.toml:127` · package workspace · lens minimality · severity medium · evidence measured · effort S · verdict confirmed

Description: Sources import packages that pixi.toml never names: aiohttp (web_control_node.py:86; arrives only via wslink←vtk-base←pcl), pillow (web_control_node.py:73; only via matplotlib←rqt-plot←desktop), numpy, pyyaml (topics.py), py-opencv (via cv-bridge), ros-humble-vision-msgs, nav2-msgs, sensor-msgs-py, control-msgs, moveit-msgs, shape-msgs, tf2-geometry-msgs, tf2-sensor-msgs, message-filters, moveit-configs-utils, nav2-common, ament-cmake-gtest/pytest. Any of the trims in footprint-01/02 can silently drop these (web_control degrades to 'PIL unavailable', aiohttp ImportError kills the node). Also `[pypi-dependencies]` (line 127) is empty while jetank_detection/README.md:41 instructs `pixi run python -m pip install ultralytics` into the env by hand, so the env is not reproducible from the lock.

Evidence detail: Import aggregation over all node .py files (grep of import/from lines) vs pixi.toml [dependencies]. pixi.lock reverse-dependency chains: aiohttp <- wslink <- vtk-base <- pcl <- ros-humble-pcl-conversions; pillow <- matplotlib-base <- ros-humble-rqt-plot <- ros-humble-rqt-bag-plugins <- ros-humble-rqt-common-plugins <- ros-humble-desktop. ultralytics/torch: 0 entries in pixi.lock.

Estimated gain: No size change; prevents breakage when trimming and makes install reproducible

Fix sketch: Declare aiohttp, pillow, numpy, pyyaml and the ros-humble-*-msgs / sensor-msgs-py / moveit-configs-utils / nav2-common packages explicitly; put ultralytics under `[pypi-dependencies]` (or a `detect` feature) so `pixi install` reproduces the runtime.

Verifier (confirmed, adjusted medium): pixi.toml `[dependencies]` (lines 70-136) names no aiohttp, pillow, numpy, pyyaml, or any ros-humble-*-msgs / nav2-common / moveit-configs-utils, and `[pypi-dependencies]` at line 138 is empty by comment ("Empty for now"); yet web_control_node.py:73-86 imports PIL/numpy/aiohttp (aiohttp raises SystemExit on ImportError), topics.py imports yaml, grasp_server.py imports moveit_msgs/control_msgs/shape_msgs, and nav2_bringup.launch.py imports nav2_common. pixi.lock confirms aiohttp is required only by wslink (lock line 13852-13856, itself pulled by vtk-base←pcl) and pillow only by matplotlib-base (line 5723-5740, pulled by ros-humble-rqt-plot), and ultralytics/torch have 0 lock entries while jetank_detection/README.md:41 instructs a manual `pip install ultralytics`. Neither package.xml declares aiohttp/pillow either, so the transitive-only exposure is real; severity stays medium since it is a reproducibility/trim-safety hazard rather than a runtime cost.

### footprint-28 — exec_depend on jetank_mission (absent from workspace) while aiohttp/PIL/numpy/cv2 go undeclared

`/home/koen/workspaces/ros2_ws/src/jetank_web_control/package.xml:20` · package jetank_web_control · lens minimality · severity medium · evidence measured · effort S · verdict confirmed

Description: package.xml:20 exec_depends on jetank_mission, which is not in src/ (jetank.repos lists it but it was never cloned), so `rosdep install` cannot resolve the key. Meanwhile the node hard-requires aiohttp (web_control_node.py:86-87), uses numpy+PIL for the map PNG (:72-76, silently disabled when missing), cv2 (:255) and optionally jetank_manipulation (:79) — none declared as python3-aiohttp / python3-numpy / python3-pil / python3-opencv / jetank_manipulation. On a trimmed env (footprint-02) the PIL feature vanishes without any dependency warning.

Evidence detail: ls src/: no jetank_mission; grep jetank_mission: package.xml:20, README, conftest, web_control_node.py:65; grep python3- in package.xml: only python3-pytest.

Estimated gain: Correct rosdep; prevents silent feature loss when trimming pixi env

Fix sketch: Make jetank_mission conditional or drop it until cloned; add python3-aiohttp, python3-numpy, python3-pil, python3-opencv as <exec_depend> and jetank_manipulation as an optional exec_depend.

Verifier (confirmed, adjusted medium): package.xml:20 declares `<exec_depend>jetank_mission</exec_depend>` but no package with that name exists under src/ (ls shows 10 packages, none named jetank_mission; jetank.repos:51 lists it as a git remote only), while web_control_node.py:64-68 treats it as optional via try/except. Conversely web_control_node.py:84-89 hard-fails (`SystemExit`) without aiohttp, and numpy/PIL (:71-75), cv2 (:255, :738) and jetank_manipulation (:78-82) are imported but package.xml declares no python3-* runtime dependency other than python3-pytest as test_depend, and setup.py has no install_requires for them. The dependency metadata is inverted relative to actual usage exactly as the finding describes.

Duplicate folded in: **cross-13** — same jetank_mission exec_depend / undeclared imports; IMPORTANT verifier note from cross-13: jetank_mission is a real sibling repo pinned in jetank_ros_main/jetank.repos:51-54 and cloned by install.sh, so do NOT delete the exec_depend; only add the missing jetank_manipulation, jetank_navigation, nav2_map_server, ament_index_python exec_depends.

Duplicate folded in: **jetank_web_control-32** — same jetank_mission/jetank_manipulation exec_depend inconsistency at package.xml:20.

### footprint-10 — Every `pixi run build` compiles and links the gtest test binaries (BUILD_TESTING defaults ON)

`/home/koen/workspaces/ros2_ws/pixi.toml:24` · package workspace · lens footprint · severity medium · evidence static · effort S · verdict confirmed

Description: The `build` task (pixi.toml:24) never passes -DBUILD_TESTING=OFF, and colcon/ament default it to ON. So each build also compiles test_stereo_math.cpp (which includes stereo_processing_strategy.hpp: opencv.hpp umbrella + PCL VoxelGrid/SOR/PassThrough templates), test_reproject.cpp, test_motor_header.cpp, links three gtest executables against ${OpenCV_LIBS}/${PCL_LIBRARIES}, and runs ament_lint_auto_find_test_dependencies() find_package fan-out in 9 packages plus ament_add_pytest_test registration. On the Jetson this is pure overhead unless `pixi run test` is being invoked.

Evidence detail: jetank_perception/CMakeLists.txt:65-94 (two ament_add_gtest with OpenCV/PCL links), jetank_motor_control/CMakeLists.txt:88-94, plus BUILD_TESTING blocks in all other CMakeLists. Task definitions at pixi.toml:24-29 contain no BUILD_TESTING flag. Not timed in this session (no build allowed).

Estimated gain: Roughly 3 of 10 heavy C++ TUs and 3 link steps skipped per full build; estimated 15-30% of the 2m03s wall time

Fix sketch: Add `-DBUILD_TESTING=OFF` to `build`/`build-*` tasks and create `build-tests = colcon build ... -DBUILD_TESTING=ON`; make `test` depend on `build-tests` instead of `build`.

Verifier (confirmed, adjusted medium): pixi.toml:22-27 shows no BUILD_TESTING flag on any build task and `test` depends on `build`, so every build compiles the gtest/pytest registrations at jetank_perception/CMakeLists.txt:65-94 (two gtests linked to OpenCV/PCL) and jetank_motor_control/CMakeLists.txt:88-94, plus ament_lint_auto fan-out in all 9 ament_cmake packages. Grepping every `if(BUILD_TESTING)` block (9 files) found no install(), add_library or add_executable inside them, so -DBUILD_TESTING=OFF changes no installed/runtime artifact and no launch file, script or CI consumes the test binaries; the fix touches only pixi.toml and leaves package boundaries and the perception strategy/factory abstractions untouched. Two caveats the fix_sketch should note: toggling the cached BUILD_TESTING value between `build` and `build-tests` forces a full CMake reconfigure of the shared build/ dir, and per-package READMEs (e.g. src/jetank_motor_control/README.md:90) that document bare `colcon test --packages-select` will silently find no tests after an OFF build. Speedup is unmeasured (static reasoning only).

### footprint-12 — find_package(PCL) without COMPONENTS links every PCL module (incl. visualization/VTK) into all perception targets

`/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:35` · package jetank_perception · lens footprint · severity medium · evidence static · effort S · verdict confirmed

Description: CMakeLists.txt:35 `find_package(PCL REQUIRED)` and lines 81, 88, 149, 162 link `${PCL_LIBRARIES}`, which with no COMPONENTS list expands to all PCL libraries (io, visualization→VTK, surface, registration, features, recognition, tracking, ...). Actual use: stereo_camera_node uses pcl::toROSMsg + PassThrough/StatisticalOutlierRemoval/VoxelGrid (common, filters); sock_segmentation_server uses SACSegmentation, ExtractIndices, KdTree, EuclideanClusterExtraction (common, filters, segmentation, search, kdtree, sample_consensus). Over-linking lengthens link time, inflates DT_NEEDED lists (dozens of libpcl_*.so + libvtk*.so loaded at process start), and raises RSS/startup latency of both nodes on the Jetson.

Evidence detail: grep 'pcl::' over perception sources: stereo_camera_node.cpp only pcl::toROSMsg; stereo_processing_strategy.hpp:630-648 PassThrough/SOR/VoxelGrid; sock_segmentation_server.cpp:548-644 SACSegmentation/ExtractIndices/KdTree/EuclideanClusterExtraction. pcl-1.15.1 in lock depends on vtk-base (64.7 MB). ldd on installed binaries not run (install/ is off-limits).

Estimated gain: Fewer shared objects mapped at node start (tens of libs), shorter link step; no source change

Fix sketch: `find_package(PCL REQUIRED COMPONENTS common filters)` for stereo_camera_node/test_stereo_math and `COMPONENTS common filters segmentation search kdtree sample_consensus` for sock_segmentation_server; link the per-component targets instead of ${PCL_LIBRARIES}.

Verifier (confirmed, adjusted medium): src/jetank_perception/CMakeLists.txt:35 is `find_package(PCL REQUIRED)` with no COMPONENTS, and lines 81, 88, 149, 162 link `${PCL_LIBRARIES}` into test_stereo_math, test_reproject, sock_segmentation_server and stereo_camera_node; PCLConfig.cmake (system copy at /usr/lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake:456,488) confirms the no-COMPONENTS default is `pcl_all_components` including visualization, which pulls in vtk. Source grep shows actual use is only common/filters (stereo_processing_strategy.hpp:8-10, stereo_camera_node.cpp:1284 toROSMsg) and common/filters/segmentation/search/sample_consensus (sock_segmentation_server.cpp:54-61); pixi.lock:1249/1300 shows pcl-1.15.1 with vtk-base-9.5.2 as a dependency, so the over-link is real. Not measured with ldd (install/ off-limits), so runtime cost is static reasoning; severity kept medium since fix is a one-line CMake change with no source impact.

### footprint-03 — gstreamer / gst-plugins-base / gst-plugins-good in pixi env cannot be used by the pixi OpenCV

`/home/koen/workspaces/ros2_ws/pixi.toml:118` · package workspace · lens footprint · severity low (orig medium) · evidence measured · effort S · verdict confirmed

Description: pixi.toml:118-120 declares gstreamer, gst-plugins-base, gst-plugins-good 'for JeTank packages', but no source uses the GStreamer C API (perception CMakeLists.txt:37-38 says so explicitly), and the conda-forge libopencv 4.13 build in the lock has no gstreamer dependency at all, so cv::CAP_GSTREAMER pipelines in camera_interface.hpp cannot work in the pixi env regardless (memory note: CSI camera node runs under system /opt/ros/humble). The three packages plus their glib-networking/libsoup/libpsl chain are dead weight.

Evidence detail: pixi.lock: libopencv-4.13.0-qt6_py312h2034ceb_606 depends list (39 entries) contains no gstreamer/gst-plugins entry. grep -rn 'gst_\|gstreamer' src C++ sources: 0 hits outside pipeline strings. Closure removal: -4 packages / -3.5 MB compressed (gst-plugins-good 2.8, libsoup 0.4, glib-networking 0.2, libpsl 0.1). Only qt-main also lists gstreamer as a dep.

Estimated gain: -3.5 MB compressed, 4 packages; removes a misleading 'GStreamer works in pixi' assumption

Fix sketch: Delete the three gst* lines (and the comment) from pixi.toml; document that CSI capture requires the system OpenCV/GStreamer under /opt/ros/humble (pixi-activate.sh:16-23 already only exports GST_PLUGIN_SYSTEM_PATH_1_0).

Verifier (confirmed, adjusted low): Verified: pixi.toml:133-135 declares gstreamer/gst-plugins-base/gst-plugins-good; the aarch64 libopencv-4.13.0 record (pixi.lock:10316+, depends block) lists no gst* entry, and the only C++ GStreamer references are cv::CAP_GSTREAMER pipeline strings (camera_interface.hpp:104,364; CMakeLists.txt:37-38 confirms no dev dep), so the pixi OpenCV cannot open them. Removal gain is exactly as stated: gstreamer+gst-plugins-base stay pinned by qt-main (pixi.lock:11863-11864), so only gst-plugins-good (2.83 MB) + libsoup (0.44) + glib-networking (0.16) + libpsl (0.08) ≈ 3.5 MB / 4 packages drop. However the gain is install-size only (~0.06% of a ~6 GB env) with zero CPU/latency/hot-path impact on the Orin, so medium overstates it; the real value is removing the misleading assumption, which is low severity.

### footprint-04 — libgpiod declared in pixi.toml but no source uses GPIO

`/home/koen/workspaces/ros2_ws/pixi.toml:117` · package workspace · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: pixi.toml:117 adds libgpiod as a 'native lib needed by JeTank packages'. grep -rn -i gpiod over all src/ (cpp, hpp, CMakeLists, package.xml, py) returns zero hits; motor.cpp:5-9 drives the motor HAT over I2C (linux/i2c-dev.h), and feetech_bus.cpp uses termios serial. The dependency (and the CLAUDE.md claim 'Uses libgpiod') is stale.

Evidence detail: grep -rn -i 'gpiod' src/ (excluding .git): 0 matches. pixi.lock closure: libgpiod-2.2.1 is a leaf, 0.1 MB compressed.

Estimated gain: -1 package / 0.1 MB; removes a stale claim

Fix sketch: Remove `libgpiod = "*"` from pixi.toml and fix the CLAUDE.md line 'Uses libgpiod for GPIO control'.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-05 — ros-humble-moveit-servo unused anywhere in src/

`/home/koen/workspaces/ros2_ws/pixi.toml:108` · package workspace · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: pixi.toml:108 pulls ros-humble-moveit-servo. No launch file, yaml, package.xml or Python file references moveit_servo (only 'servo' in the sense of Feetech bus servos). It also drags ros-humble-joy, control-toolbox and tf2-eigen into the solve.

Evidence detail: grep -rn 'moveit_servo' src/ (all file types): 0 hits. pixi.lock: moveit-servo depends on 29 packages (joy, control-toolbox, tf2-eigen, ...); closure removal saves 1 package / 1.0 MB compressed since the rest is shared.

Estimated gain: -1.0 MB compressed, fewer solve constraints

Fix sketch: Delete `ros-humble-moveit-servo = "*"` from pixi.toml.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-06 — ros-humble-joint-state-publisher-gui only used by a dev-only launch (pulls pyqt/Qt5 python binding)

`/home/koen/workspaces/ros2_ws/pixi.toml:82` · package workspace · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: pixi.toml:82 installs joint-state-publisher-gui; the only user is jetank_motor_control/launch/test_urdf.launch.py:61-64 (a manual URDF sanity launch with RViz). It is the direct parent of ros-humble-python-qt-binding → pyqt (6.4 MB). On the Jetson this is a desktop debugging tool, not a robot dependency. jetank_description/package.xml:21 also lists it as exec_depend although robot_description.launch.py only starts robot_state_publisher.

Evidence detail: grep joint_state_publisher_gui --include=*.py src/: only test_urdf.launch.py. pixi.lock reverse deps: pyqt <- ros-humble-python-qt-binding <- ros-humble-joint-state-publisher-gui (pyqt is also reachable via rqt from ros-humble-desktop, so standalone removal saves 1 pkg; combined with footprint-02 it removes the pyqt chain).

Estimated gain: -1 package alone; with footprint-02, drops pyqt (6.4 MB) and python-qt-binding

Fix sketch: Move joint-state-publisher-gui into a `dev`/`sim` pixi feature; drop the exec_depend from jetank_description/package.xml:21.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-11 — No parallel-worker / job cap or ccache for a 6-core, 7.6 GB Jetson

`/home/koen/workspaces/ros2_ws/pixi.toml:24` · package workspace · lens footprint · severity low (orig medium) · evidence measured · effort S · verdict **UNCERTAIN**

Description: pixi.toml:24 runs plain `colcon build --symlink-install`. colcon defaults to N=nproc packages in parallel and colcon-cmake passes make -j nproc per package, so up to 6x6 compiler jobs can coexist. The perception TUs each instantiate PCL filter templates plus the full OpenCV header set (multi-hundred-MB compilers), so with 3.5 GB available the Orin Nano can page. There is also no ccache, so touching stereo_camera_node.cpp or a header recompiles from scratch every time.

Evidence detail: nproc = 6; free -m: total 7619 MB, available 3548 MB at time of measurement. Task text read at pixi.toml:24-28 (no --parallel-workers, no MAKEFLAGS, no CMAKE_CXX_COMPILER_LAUNCHER). ccache: 0 entries in pixi.lock. Peak build RSS not measured.

Estimated gain: Avoids swap stalls on full rebuilds; ccache makes header-touch rebuilds seconds instead of ~30 s+

Fix sketch: Add `ccache` to pixi deps and `-DCMAKE_CXX_COMPILER_LAUNCHER=ccache` to --cmake-args; set `--parallel-workers 2` (or `MAKEFLAGS=-j4` in [activation.env]) for the Jetson profile.

Verifier (uncertain, adjusted low): The premise is only partly right: pixi.toml:21-25 indeed sets no --parallel-workers/MAKEFLAGS/ccache and pixi.lock has 0 ccache entries, but colcon-cmake (build.py:314) passes `-j6 -l6`, so the load-average limit caps the 36-compiler scenario, and the workspace has only 9 C++ TUs across 3 packages (perception 4, motor 4, nav 1) with the other 7 packages being Python/URDF-only — so full-build memory pressure is far smaller than described (3.8 GB zram swap also exists, 0 used). The est_gain is also overstated: ccache cannot make "header-touch rebuilds seconds" because a changed included header changes the preprocessed hash and misses the cache; it only helps clean rebuilds/branch switches. Peak build RSS was not measured, so a real swap stall cannot be confirmed or excluded; the effort-S ccache/-j cap is harmless but the gain is speculative.

### footprint-19 — Raw calibration dump ost.txt shipped in share/

`/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:201` · package jetank_perception · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: CMakeLists.txt:201-202 installs the whole config/ tree, which includes config/calibration/ost.txt (the camera_calibration text dump). Only left_camera.yaml, right_camera.yaml and stereo_calibration.yaml are referenced (stereo_camera_config.yaml:50-52, stereo_camera.launch.py:21-23). ost.txt is a source artifact, not a runtime file.

Evidence detail: ls config/calibration: left_camera.yaml, ost.txt, right_camera.yaml, stereo_calibration.yaml. grep -rn 'ost.txt' src/: only a mention in jetank_ros_main/plans/sim2real-gap-analysis.md.

Estimated gain: ~1 KB; hygiene

Fix sketch: Add `PATTERN "*.txt" EXCLUDE` to the config install, or move ost.txt under docs/.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-22 — gripper_controllers exec_depend unused; ign_ros2_control exec_depend drags Gazebo onto a hardware package

`/home/koen/workspaces/ros2_ws/src/jetank_motor_control/package.xml:39` · package jetank_motor_control · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: package.xml:39 lists gripper_controllers, but config/jetank_controllers.yaml:20 uses `position_controllers/GripperActionController` (position_controllers, already declared at :38); no yaml or launch references gripper_controllers/*. package.xml:47 exec_depends on ign_ros2_control because ros2_control.xacro has a use_sim branch — this makes the hardware-facing motor package require the entire Ignition stack via rosdep on the Jetson.

Evidence detail: grep 'type:' jetank_controllers.yaml: joint_state_broadcaster, joint_trajectory_controller, position_controllers/GripperActionController, forward_command_controller, diff_drive_controller. grep 'gripper_controllers' src/ --include=*.yaml --include=*.py: 0 hits.

Estimated gain: One fewer rosdep; sim stack no longer required to satisfy the hardware package

Fix sketch: Drop line 39; make line 47 conditional (`<exec_depend condition="$JETANK_SIM == 1">ign_ros2_control</exec_depend>`) or move it to jetank_simulation, which already declares it.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-23 — Deprecated Gazebo-Classic stub launch still installed

`/home/koen/workspaces/ros2_ws/src/jetank_motor_control/launch/gazebo_sim.launch.py:21` · package jetank_motor_control · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: launch/gazebo_sim.launch.py only emits a deprecation message (line 21: 'DEPRECATED (Gazebo Classic)') and README.md:77 marks it deprecated, yet CMakeLists.txt:76-77 installs the whole launch dir so it ships to share/jetank_motor_control/launch and shows up in `ros2 launch` tab completion.

Evidence detail: grep 'DEPRECATED' launch/gazebo_sim.launch.py:21; README.md:77.

Estimated gain: 1 KB; removes a misleading entry point

Fix sketch: Delete the file and the README row.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-24 — install(DIRECTORY launch) without __pycache__ EXCLUDE ships stale .pyc files (7 packages)

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:18` · package jetank_simulation · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: install(DIRECTORY ...) for launch/config has no `PATTERN "__pycache__" EXCLUDE` in jetank_simulation/CMakeLists.txt:18, jetank_detection:56, jetank_manipulation:72, jetank_moveit_config:19, jetank_motor_control:76, jetank_navigation:33 and jetank_perception:190 (only jetank_description:17 and the two python-package installs exclude it). The source launch dirs currently hold cpython-310 and cpython-312 .pyc files from earlier runs, so those get copied/symlinked into share/ and make install content depend on what was previously executed.

Evidence detail: du of __pycache__ inside installable dirs: jetank_ros_main/launch 112K (setup.py glob, not affected), jetank_navigation/launch 64K, jetank_simulation/launch 48K, jetank_moveit_config/launch 32K, jetank_detection/launch 28K, jetank_manipulation/launch 8K; both 310 and 312 tags present.

Estimated gain: ~180 KB and deterministic install trees

Fix sketch: Add `PATTERN "__pycache__" EXCLUDE` to every install(DIRECTORY ...) that touches launch/config/scripts; run `find src -name __pycache__ -exec rm -r {} +` once.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-25 — Empty models/, config/, meshes/, maps/ directories installed as share subdirs

`/home/koen/workspaces/ros2_ws/src/jetank_simulation/CMakeLists.txt:16` · package jetank_simulation · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: jetank_simulation/CMakeLists.txt:16-22 installs models/ and config/ if they exist — both are empty untracked dirs (Nov 2025 leftovers), as are include/jetank_simulation and src/. jetank_description/CMakeLists.txt:13 similarly installs empty meshes/ and config/ (and has empty include/, src/). jetank_navigation/CMakeLists.txt:39-43 installs maps/ which is empty. The CLAUDE.md claim '3D mesh files for visualization' is false: there are no meshes and no package:// references in any xacro.

Evidence detail: ls -la of each dir shows only . and ..; git ls-files in jetank_description/jetank_simulation returns no entries for meshes|models|include|src|config; grep 'package://' urdf: 0 hits.

Estimated gain: Hygiene only; removes phantom share dirs and false docs

Fix sketch: rmdir the empty dirs (they are untracked), trim the foreach lists to dirs that exist, and fix the CLAUDE.md package description.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-34 — C++17 requested three ways and CUDA path keyed on OPENCV_ENABLE_NONFREE

`/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:5` · package jetank_perception · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: CMakeLists.txt:5-6 sets CMAKE_CXX_STANDARD 17 globally and :151/:158/:173 repeat target_compile_features(cxx_std_17) per target; :9 cmake_policy(SET CMP0074 NEW) is already the default with cmake_minimum_required ≥3.12. Lines 52-54 map 'OpenCV has CUDA' to `-DOPENCV_ENABLE_NONFREE`, and stereo_processing_strategy.hpp:17-21 uses that same macro as the guard for cudaimgproc/cudastereo. The pixi libopencv has no CUDA, so the GPU strategy is compiled out in the pixi env; the macro name misdescribes what it does and would also switch on OpenCV non-free algorithms if a CUDA build were used.

Evidence detail: pixi.lock: 0 packages matching cuda; libopencv build string qt6_py312. Header guard read at stereo_processing_strategy.hpp:17-21.

Estimated gain: Clarity; no size change

Fix sketch: Keep one C++17 setting, drop the CMP0074 line, and rename the define to JETANK_OPENCV_CUDA (set from OpenCV_CUDA_VERSION) in both CMake and the header.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-35 — Template scaffold launch file installed (simple_camera.launch.py)

`/home/koen/workspaces/ros2_ws/src/jetank_perception/launch/simple_camera.launch.py:23` · package jetank_perception · lens minimality · severity low · evidence measured · effort S · verdict **UNVERIFIED**

Description: launch/simple_camera.launch.py still carries template comments ('Change this to your package name', 'Replace with your executable name' at line 23-24, 'This is a bit advanced' at :19-20) and hard-codes parameters that single_camera.launch.py already exposes as arguments. It is installed by CMakeLists.txt:190-192 and adds a third camera launch entry point that no pixi task or README references.

Evidence detail: cat of the file (43 lines) shows the template comments; grep 'simple_camera' src/ outside the file: 0 hits.

Estimated gain: 1.4 KB; one fewer confusing entry point

Fix sketch: Delete simple_camera.launch.py (single_camera.launch.py covers the use case).

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-14 — opencv.hpp umbrella header in 5 files plus a 2019-line node TU dominate compile time

`/home/koen/workspaces/ros2_ws/src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:3` · package jetank_perception · lens footprint · severity low (orig medium) · evidence measured · effort M · verdict confirmed

Description: `#include <opencv2/opencv.hpp>` at stereo_processing_strategy.hpp:3, camera_interface.hpp:3, quality_monitor.hpp:3, quality_monitor.cpp:3 and stereo_camera_node.cpp:23 parses every OpenCV module header in each TU. stereo_camera_node.cpp is 2019 lines and additionally includes PCL filter templates via the strategy header, yaml-cpp, message_filters, image_transport, camera_info_manager. test_stereo_math.cpp includes the same strategy header, so the heavy template set is compiled twice per build. This TU is the long pole of the perception package build.

Evidence detail: wc -l: stereo_camera_node.cpp 2019, stereo_processing_strategy.hpp 655, camera_interface.hpp 442. build_output.log (ws root, Jan 2026, system-ROS build) shows jetank_perception alone at 36.5 s. Per-TU timing not measured this session.

Estimated gain: Estimated 20-40% shorter perception compile (the package is the longest of the 10)

Fix sketch: Replace opencv.hpp with core.hpp/imgproc.hpp/calib3d.hpp/videoio.hpp; add `target_precompile_headers(stereo_camera_node PRIVATE <opencv2/core.hpp> <pcl/point_cloud.h> <rclcpp/rclcpp.hpp>)`; optionally move the pcl filter implementations out of the header into a .cpp compiled once (stays inside the strategy pattern).

Verifier (confirmed, adjusted low): Measured on this Jetson (g++ -fsyntax-only, /usr/include/opencv4): a TU with `<opencv2/opencv.hpp>` parses in 4.8-5.0 s vs 2.6-2.7 s with only core/imgproc/calib3d/videoio/imgcodecs, and stereo_processing_strategy.hpp:6-10's PCL filter headers alone cost 7.4 s — so the include set is genuinely the dominant cost of the 2019-line stereo_camera_node.cpp TU (CMakeLists.txt:97-99 also compiles quality_monitor.cpp into it, and test/test_stereo_math.cpp:12 re-parses the same strategy header). However the headline fix (dropping opencv.hpp, used at stereo_processing_strategy.hpp:3, camera_interface.hpp:3, quality_monitor.hpp:3, quality_monitor.cpp:3, stereo_camera_node.cpp:23) saves only ~2.2 s per TU, i.e. roughly 6% of the 36.5 s package wall time from build_output.log:74 since the long-pole TU sets the critical path on 6 cores; the 20-40% figure is only plausible if the PCH/PCL-out-of-header part of the fix_sketch is also done. Symbol usage confirms the narrow set suffices (only cv::Mat/Size/cuda/StereoBM/SGBM/remap/VideoCapture/imencode etc.; no dnn/ml/features2d/stitching). Real but modest developer-time gain on a rebuild-only path, no runtime impact, so severity is low.

### footprint-07 — ninja installed but colcon build tasks never select the Ninja generator

`/home/koen/workspaces/ros2_ws/pixi.toml:56` · package workspace · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: pixi.toml:56 adds ninja, but the build/build-* tasks (lines 24-28) pass only `--cmake-args -DCMAKE_BUILD_TYPE=Release`, so CMake uses Unix Makefiles. Either the package is dead weight or the build is leaving the faster no-op/incremental Ninja builds on the table.

Evidence detail: Task definitions read at pixi.toml:24-28 contain no `-G Ninja`/`CMAKE_GENERATOR`; ninja-1.13.2 present in lock (0.2 MB). Not measured which is faster for this small tree.

Estimated gain: Either -0.2 MB or faster incremental rebuild checks (seconds per package)

Fix sketch: Add `-G Ninja` to the `--cmake-args` of the build tasks (or set `CMAKE_GENERATOR=Ninja` in [activation.env]), or remove ninja.

Verifier (unverified, adjusted low): low severity, not sent to verifier

### footprint-15 — Directory-wide include_directories/link_directories/add_definitions apply PCL+yaml-cpp flags to camera_node

`/home/koen/workspaces/ros2_ws/src/jetank_perception/CMakeLists.txt:59` · package jetank_perception · lens minimality · severity low · evidence static · effort S · verdict **UNVERIFIED**

Description: CMakeLists.txt:46 link_directories(${PCL_LIBRARY_DIRS}), :49 add_definitions(${PCL_DEFINITIONS}) and :59-63 include_directories(include ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS}) are global, so camera_node (which the comment at :167-168 says uses no PCL or yaml-cpp) is compiled with the PCL/VTK/Boost/Eigen/FLANN include search path and PCL defines. The per-target target_include_directories at :144-146/154-156/169-171 already exist, making the global forms redundant.

Evidence detail: Read CMakeLists.txt:42-63 and :144-177; single_camera.cpp includes only rclcpp, sensor_msgs, cv_bridge, chrono and camera_interface.hpp.

Estimated gain: Marginal compile-time (shorter include search), cleaner target graph

Fix sketch: Delete lines 46, 49, 59-63; attach PCL/yaml-cpp includes and definitions per target via target_include_directories/target_compile_definitions on stereo_camera_node and sock_segmentation_server only.

Verifier (unverified, adjusted low): low severity, not sent to verifier

## Refuted findings

- **jetank_perception-35** — Whole-PCL find_package and ${PCL_LIBRARIES} linked into every target and test: The runtime claim does not hold: the pixi/RoboStack toolchain links with -Wl,--as-needed (.pixi/envs/default/etc/conda/activate.d/activate-gcc_linux-aarch64.sh LDFLAGS), so unused PCL modules from ${PCL_LIBRARIES} (CMakeLists.txt:35, 149, 162) are already dropped from NEEDED — the finding's own ldd shows only pcl_filters plus exactly its transitive NEEDED set (readelf libpcl_filters.so: sample_consensus, search, kdtree, octree, common), which a COMPONENTS-scoped find_package would produce identically. No fewer shared objects would be mapped at node start on the Orin, so est_gain is not plausible. What remains is minor build hygiene: find_package(PCL) without COMPONENTS resolves ~20 PCL modules (incl. visualization/VTK) at configure time and the linker scans them on each of the 4 links (81, 88, 149, 162) — a small incremental-link/configure cost, not a hot-path or footprint issue.
- **jetank_motor_control-28** — ign_ros2_control exec_depend pulls the Gazebo stack onto the hardware robot: The cited lines do not exist: jetank_motor_control/package.xml is 63 lines and the ign_ros2_control exec_depend is at line 47 (with a comment at 43-46 explaining the xacro use). More importantly the claimed gain is implausible: this workspace is pixi-managed with a single default environment and platforms = ["linux-aarch64","linux-64"] (pixi.toml:13), and pixi.toml:126-129 directly pins ros-humble-ros-gz-sim/bridge/image and ros-humble-ign-ros2-control at workspace level, so rosdep/package.xml never drives what lands on the Jetson; removing the exec_depend would install zero fewer bytes. jetank_simulation/package.xml:36-37 already declares ign_ros2_control and diff_drive_controller, so the "move" is redundant too. What remains is a package.xml hygiene nit (sim-only dep declared in the hardware driver package), not a footprint win.
- **footprint-13** — ${OpenCV_LIBS} (all modules) linked twice into camera_node and stereo_camera_node: The fix is in-scope (no ament_export_* in jetank_perception/CMakeLists.txt, no downstream package links it, strategy/factory untouched) but its claimed gain does not hold: the pixi toolchain links with `-Wl,--as-needed` (pixi run printenv LDFLAGS), so unused libopencv_*/openvino libs are already dropped from DT_NEEDED of camera_node/stereo_camera_node, and under system ROS (/opt/ros/humble, which the CSI camera path requires) cv_bridge's exported target already pulls the full OpenCV contrib set (dnn, highgui, viz, ...) transitively, so trimming COMPONENTS in this package changes nothing. The fix_sketch is also incomplete as written: stereo_camera_node.cpp:1145/1149 use cv::imencode (imgcodecs, not in the proposed list), and all sources `#include <opencv2/opencv.hpp>` (5 hits), so header weight is unchanged. Only the cosmetic double-link (CMakeLists.txt:122/128 vs 161/176) is real; it has no measurable cost.

## Measurement follow-ups

Static runtime findings whose node/path was not live during this audit. Each needs a measurement before its est_gain can be trusted.

### Node not live

- [ ] jetank_detection-01 (medium) — Action execute loop busy-polls _latest_image with 10 ms sleeps — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:363` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-02 (medium) — Continuous mode runs inference on every camera frame with no rate limit — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:275` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-04 (medium) — No warm-up inference after model load; first action goal pays CUDA/cuDNN init inside its timeout — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:72` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-05 (medium) — predict() called without imgsz/half/device; FP16 and fixed input size not exploited on Jetson — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:79` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_motor_control-17 (medium) — write() re-sends unchanged goal positions every cycle (including S5, which no controller commands) — `src/hardware/jetank_serial_hardware.cpp:378` — ros2_control_node/serial bus dead
- [ ] jetank_motor_control-18 (medium) — 12 ms reply timeout is ~10x longer than a 1 Mbps status frame needs; a single silent servo overruns the 20 ms cycle — `src/hardware/feetech_bus.cpp:142` — ros2_control_node/serial bus dead
- [ ] jetank_navigation-16 (medium) — DWB samples 800 trajectories per 20 Hz control cycle for a 0.3 m/s robot — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:150` — Nav2 (DWB) not running
- [ ] jetank_simulation-09 (medium) — robot_remote.launch.py spawners have no sequencing and no --controller-manager-timeout — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/robot_remote.launch.py:74` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-14 (medium) — 1 ms physics step in all five worlds is 20x the controller rate — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:7` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_web_control-05 (medium) — Browser send loop streams zero-velocity commands at 10 Hz forever, defeating the node's silent-when-idle watchdog — `/home/koen/workspaces/ros2_ws/static/app.js:211` — no browser client connected
- [ ] jetank_motor_control-16 (medium) — Hardware read()/write() perform 2 blocking serial transactions per servo per 50 Hz cycle (10 total) inside the controller_manager loop — `src/hardware/jetank_serial_hardware.cpp:346` — ros2_control_node/serial bus dead (servo no ping)
- [ ] jetank_navigation-17 (medium) — Nav2 lifecycle set launches waypoint_follower, velocity_smoother and (SLAM variant) smoother_server that no workflow uses — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/launch/nav2_bringup.launch.py:63` — Nav2 not running
- [ ] jetank_description-27 (low) — gpu_lidar <visualize>true</visualize> renders 360 rays per scan in the sim GUI — `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/lidar.xacro:46` — Gazebo not running
- [ ] jetank_description-28 (low) — Gaussian image noise on both sim cameras adds a per-pixel pass at 640x360x30 fps x2 — `/home/koen/workspaces/ros2_ws/src/jetank_description/urdf/components/camera.xacro:101` — Gazebo not running
- [ ] jetank_detection-03 (low) — Backend with no model loaded is non-None, so every frame is converted and logs a warning at camera rate — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:146` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-06 (low) — Three separate GPU->host transfers per inference (xyxy, conf, cls) — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/backends.py:93` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-07 (low) — MultiThreadedExecutor uses cpu_count() (6) threads for one subscription + one action server — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:494` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-08 (low) — Image subscriptions use RELIABLE QoS for 30 Hz raw frames — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:204` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-10 (low) — Per-goal subscription create/destroy adds DDS matching latency to every on-demand goal — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:345` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-11 (low) — _latest_image retains a full frame indefinitely after a goal finishes — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:258` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_detection-12 (low) — capture_frames: depth-10 reliable queue and blocking JPEG write in the subscription callback — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/capture_frames.py:75` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_manipulation-02 (low) — spin_until_complete busy-polls at 50 Hz for up to 60 s per action call — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:71` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-03 (low) — time.sleep() inside async execute callback blocks an executor thread — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:686` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-04 (low) — Coordinator always builds+TFs the world grasp pose even in default 'preset' mode where it is never used — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:179` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-05 (low) — TransformListener with spin_thread=True spawns an extra thread and hidden node in the coordinator — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:105` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-06 (low) — wait_for_server timeouts equal the result timeouts -> missing server blocks a Trigger service for up to 60 s — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py:325` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-07 (low) — MultiThreadedExecutor() defaults to cpu_count threads per node (6 on Orin Nano) for at most 2-3 concurrent callbacks — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/node_runner.py:28` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-11 (low) — _moveit_error_name reflects over vars(MoveItErrorCodes) on every call — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:91` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-12 (low) — Debug f-string for the pose request is formatted eagerly even when debug logging is off — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:301` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-18 (low) — cloud_to_xyz makes 3-4 intermediate copies to convert a structured array to (N,3) — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/action_utils.py:114` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-19 (low) — pca_long_axis_yaw allocates a per-point norm array only for a degeneracy test, then np.cov recentres again — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_math.py:52` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-20 (low) — Control loop re-imports time and re-reads a parameter every tick; hypot/atan2 computed twice — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/base_approach_node.py:397` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_manipulation-32 (low) — wait_for_server re-queried on every arm move and gripper command within one goal — `/home/koen/workspaces/ros2_ws/src/jetank_manipulation/jetank_manipulation/grasp_server.py:481` — no jetank_manipulation node running (grasp_server/base_approach/coordinator/grasp_pose_node)
- [ ] jetank_motor_control-19 (low) — tcflush(TCIFLUSH) syscall before every packet plus echo-compare on every RX chunk — `src/hardware/feetech_bus.cpp:250` — ros2_control_node/serial bus dead
- [ ] jetank_moveit_config-11 (low) — demo.launch.py runs xacro on the full robot description twice at startup — `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/launch/demo.launch.py:56` — demo.launch.py not used in the live bringup
- [ ] jetank_moveit_config-17 (low) — RViz config renders TF names for all 20 frames at 30 FPS — `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/moveit.rviz:32` — RViz not running
- [ ] jetank_navigation-14 (low) — Local costmap uses VoxelLayer with publish_voxel_map for a 2D-lidar-only robot — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:195` — Nav2 not running
- [ ] jetank_navigation-15 (low) — always_send_full_costmap: True on both costmaps forces full-grid publishes every cycle — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:218` — Nav2 not running
- [ ] jetank_navigation-18 (low) — slam_toolbox enable_interactive_mode: true publishes interactive markers for every graph node — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/slam/slam_toolbox.yaml:43` — slam_toolbox not running
- [ ] jetank_navigation-19 (low) — RViz config subscribes to /stereo_camera/points (dropped pipeline) and renders all TF frames — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/rviz/navigation.rviz:38` — RViz not running
- [ ] jetank_navigation-28 (low) — AMCL particle count and update thresholds set high for a small indoor robot — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/config/nav2/nav2_params.yaml:44` — AMCL not running
- [ ] jetank_perception-10 (low) — capture_loop clones every frame into latest_frame_ and sleeps 1 ms although camera_node only uses the async callback — `src/jetank_perception/include/jetank_perception/camera_interface.hpp:406` — camera_node (single_camera.cpp) not running
- [ ] jetank_perception-12 (low) — sock_segmentation_server scans the whole disparity image per goal only to log a pixel count, and logs INFO per detection stage — `src/jetank_perception/src/sock_segmentation_server.cpp:239` — sock_segmentation_server not running
- [ ] jetank_perception-13 (low) — Blob transform round-trips PCL->PointCloud2->tf->PCL and copies the result cloud twice — `src/jetank_perception/src/sock_segmentation_server.cpp:487` — sock_segmentation_server not running
- [ ] jetank_perception-40 (low) — ros_topics input path uses toCvCopy for both frames where toCvShare suffices — `src/jetank_perception/src/stereo_camera_node.cpp:917` — ros_topics (sim) input path not active
- [ ] jetank_perception-41 (low) — sock_segmentation_server spawns a detached thread per goal and blocks up to 0.4 s on TF lookups — `src/jetank_perception/src/sock_segmentation_server.cpp:196` — sock_segmentation_server not running
- [ ] jetank_ros_main-16 (low) — Diagnostic scripts use rclpy Rate.sleep() in the same thread as spin_once (blocking/deadlock-prone loop) — `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/scripts/test_drive.py:45` — diagnostic scripts not run
- [ ] jetank_ros_main-35 (low) — unified.rviz enables both compressed camera streams, the point cloud and Nav2/AMCL displays by default — `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/rviz/unified.rviz:66` — RViz not running
- [ ] jetank_simulation-01 (low) — Relay dedup threshold 1e-6 m republishes on every /joint_states message under physics jitter — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:91` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-02 (low) — Eager f-string formatting in the per-message debug log — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/scripts/gripper_mimic_relay:98` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-04 (low) — Relay is given use_sim_time although it never reads the clock — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:223` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-05 (low) — Two separate ros_gz_bridge parameter_bridge processes where one suffices — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:144` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-06 (low) — Gazebo launched with -v 4 (debug verbosity) in every launch path — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:89` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-07 (low) — Five separate spawner processes launched in parallel for the controllers — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo.launch.py:159` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-11 (low) — Remote launch bridges raw 640x360 RGB images over the network via parameter_bridge — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/launch/gazebo_remote.launch.py:53` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-15 (low) — Shadow mapping enabled in every world for two 30 Hz camera sensors — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:27` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-16 (low) — Contact system plugin loaded in all worlds but the robot has no contact sensor — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/empty_fortress.sdf:15` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_simulation-18 (low) — Semi-transparent marker models cost an extra ogre2 transparency pass per frame — `/home/koen/workspaces/ros2_ws/src/jetank_simulation/worlds/sock_arena.sdf:555` — Gazebo/ros_gz/gripper_mimic_relay not running
- [ ] jetank_web_control-10 (low) — Map PNG re-downloaded every second with a cache-busting query even when the map has not changed — `/home/koen/workspaces/ros2_ws/static/app.js:502` — no browser client / no map
- [ ] jetank_web_control-13 (low) — Labeller redraw performs 4 layout reads per box per mouse-move — `/home/koen/workspaces/ros2_ws/static/app.js:938` — no browser client (labeller)
- [ ] jetank_web_control-15 (low) — cmd_vel_bridge allocates a new TwistStamped and Time object on every 20 Hz tick — `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/cmd_vel_bridge.py:184` — cmd_vel_bridge (sim-only) not running
- [ ] jetank_detection-09 (low) — Debug image published as raw bgr8 Image; full-frame copy per debug frame — `/home/koen/workspaces/ros2_ws/src/jetank_detection/jetank_detection/sock_detector_node.py:293` — no /sock_detector or /frame_capture node running; ultralytics not in pixi env
- [ ] jetank_ros_main-24 (low) — Fixed worst-case TimerAction staggers (up to 46 s sim, 28 s hardware) instead of readiness events — `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/launch/mobile_grasp.launch.py:77` — mobile_grasp launches not used in the live bringup

### Node live, but the specific path was not exercised or timed

- [ ] jetank_perception-02 (high) — Rectification remap runs on 3-channel BGR, then converted to gray; rect images mislabeled mono8 — `src/jetank_perception/src/stereo_camera_node.cpp:1058`
- [ ] jetank_perception-09 (medium) — StatisticalOutlierRemoval (k=30) on every frame is the heaviest filter in the chain — `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:637`
- [ ] jetank_motor_control-02 (low) — No change detection: identical cmd_vel values re-write the PCA9685 every message — `src/motor/motor.cpp:108`
- [ ] jetank_motor_control-06 (low) — cmd_vel subscription QoS depth 10 lets stale commands queue behind blocking I2C — `src/motor/robot_controller.cpp:63`
- [ ] jetank_motor_control-07 (low) — Missing I2C device causes RCLCPP_ERROR on every register write in the hot path — `src/motor/motor.cpp:170`
- [ ] jetank_motor_control-36 (low) — Throttled WARN every 5 s forever while the robot is legitimately idle — `src/motor/robot_controller.cpp:206`
- [ ] jetank_moveit_config-07 (low) — longest_valid_segment_fraction 0.005 doubles collision checks per OMPL motion vs the default — `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/ompl_planning.yaml:91`
- [ ] jetank_navigation-04 (low) — Per-tick heap allocation from exception construction on I2C read failure path — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:165`
- [ ] jetank_navigation-05 (low) — Messages published by const-ref (copy into middleware) instead of unique_ptr / loaned — `/home/koen/workspaces/ros2_ws/src/jetank_navigation/src/icm20948_node.cpp:339`
- [ ] jetank_perception-08 (low) — All large messages published by copy (publish(*msg) / by value) instead of unique_ptr — `src/jetank_perception/src/stereo_camera_node.cpp:1108`
- [ ] jetank_perception-11 (low) — Each CSI camera is opened twice at startup (compatibility test then real open) plus test frames — `src/jetank_perception/include/jetank_perception/camera_interface.hpp:305`
- [ ] jetank_perception-18 (low) — Quality metrics use per-pixel .at<>() loops and full-size value copies (three double vectors for cloud stats) — `src/jetank_perception/src/quality_monitor.cpp:100`
- [ ] jetank_perception-19 (low) — GPU strategy uses pageable host Mats, synchronous stream, and reserves 2x64 MB buffer pool — `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:204`
- [ ] jetank_ros_main-08 (low) — topics.py re-opens and re-validates topics.yaml on every accessor call — `/home/koen/workspaces/ros2_ws/src/jetank_ros_main/jetank_ros_main/topics.py:27`
- [ ] jetank_web_control-12 (low) — GET /captures opens and parses every label sidecar in the dataset on each request — `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:675`
- [ ] jetank_web_control-14 (low) — _launch_nav forks 11 sequential pkill processes and then sleeps a fixed 1 s before every nav start — `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1024`
- [ ] jetank_web_control-16 (low) — nav_status performs blocking stat + waitpid syscalls on the asyncio loop each poll — `/home/koen/workspaces/ros2_ws/src/jetank_web_control/jetank_web_control/web_control_node.py:1003`
- [ ] jetank_motor_control-08 (low) — Two Motor instances each open /dev/i2c-7 and re-run the PCA9685 reset/prescale sequence — `src/motor/motor.cpp:90`
- [ ] jetank_moveit_config-09 (low) — SRDF disable_collisions matrix leaves most static link pairs enabled for self-collision checking — `/home/koen/workspaces/ros2_ws/src/jetank_moveit_config/config/jetank.srdf:93`
- [ ] jetank_perception-06 (low) — Point cloud path allocates and copies the full frame 4-5 times per frame before filtering — `src/jetank_perception/include/jetank_perception/stereo_processing_strategy.hpp:86`

## Constraint check

No kept finding proposes merging packages or collapsing the jetank_perception strategy/factory abstractions (CameraFactory/CameraInterface, StereoProcessingStrategy/create_strategy). Borderline items reviewed and kept as-is: jetank_perception-23/-24/-25 trim private pipeline machinery, unused virtuals and never-instantiated subclasses but explicitly keep the factory and strategy interfaces; jetank_detection-17 removes jetank_detection's own DetectorBackend ABC/make_backend factory (not a perception abstraction); jetank_detection-28, cross-35 and cross-32 move individual files between packages without merging any package. Nothing is EXCLUDED.

<!-- counts: total=359 high=7 medium=42 low=310 dropped_as_duplicate=25 followups_not_live=63 followups_live_unmeasured=20 top5=jetank_perception-01,jetank_perception-05,jetank_ros_main-01,footprint-01,jetank_perception-22 -->

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

## Addendum 2026-09-27 — Measurement pass 1: Nav2 (navigation-only) + slam_toolbox with the real lidar

Stack: navigation_full.launch.py mode:=slam rviz:=False plus navigation_only.launch.py use_sim_time:=False. No goal sent; all numbers are idle-stack measurements (profile_node 10 s, measure_topic_perf 10 s).

| id | status | key numbers | live verdict | severity now |
|---|---|---|---|---|
| jetank_navigation-16 | measured (idle only; sampling cost not exercised) | vx_samples=20, vtheta_samples=20 (Nav2 default), vy_samples=0, controller_frequency=20.0, sim_time=1.5, short_circuit_trajectory_evaluation=true, plugin dwb_cor; YAML sets vx_samples: 20, vy_samples: 0, vth_samples: 40 - the key 'vth_samples' is misspelled (DWB expects 'vtheta_samples'), so the 40 is silently ignored and; CPU mean 3.56% p95 9.9% peak 19.7%; RSS 47.66 MB; 16 threads; ctx switches 332 vol / 26 invol over 10 s | The claim of 800 trajectories/cycle is not what runs: live vtheta_samples is 20, so DWB would evaluate 20x20 = 400 trajectories per 20 Hz cycle, because the YAML key vth_samples is a typo and never applied. Idle controller_server (with the local costmap inside it) already costs 3.6% CPU mean; the sampling cost itself was not exercised (no goal allowed). The real defect is the misspelled key, which silently drops the intended tuning. | medium |
| jetank_navigation-17 | measured-partial | CPU mean 0.10% p95 0.0% peak 9.9%; RSS 32.08 MB; 11 threads; 0 ctx switches in 10 s; CPU mean 0.40% p95 0.0% peak 9.9%; RSS 29.87 MB; 11 threads; 203 voluntary ctx switches in 10 s; CPU mean 1.88% p95 9.9% peak 9.9%; RSS 34.92 MB; 13 threads; 11 voluntary ctx switches | Partially supported. waypoint_follower and smoother_server are genuinely unused (zero clients, zero subscribers to /plan_smoothed, absent from the active BT) and cost ~2.0% CPU mean and ~67 MB RSS combined at idle, most of it smoother_server (1.88%) because it subscribes to the full global costmap_raw. velocity_smoother is NOT unused: it is the only path from /cmd_vel_nav to /cmd_vel and removing it would stop the robot moving. Finding should be narrowed to waypoint_follower + smoother_server. | low |
| jetank_navigation-14 | measured-partial | No process found - the costmap runs inside /controller_server (pid 7538): CPU mean 3.56% p95 9.9% peak 19.7%, RSS 47.66 MB, 16 threads; plugins [voxel_layer, inflation_layer]; voxel_layer.publish_voxel_map=true; z_voxels=16, z_resolution=0.05; update_frequency=5.0, publish_frequency=2.0; 3x3 m @; 50 msgs / 10 s = 5.01 Hz, 506,025 B/s (~101 KB/msg), jitter 3.3 ms, 0 drops, latency p50 1.7 ms p95 2.1 ms | Supported. The voxel map is published at the full 5 Hz update rate and is the single largest topic in the idle stack at ~506 KB/s, 6.8x the bandwidth of the costmap itself, with zero consumers. For a 2D-lidar-only robot a 16-level VoxelLayer adds serialisation work in controller_server every cycle for no benefit; setting publish_voxel_map: false (or switching to ObstacleLayer) removes it. | low |
| jetank_navigation-15 | measured-partial | always_send_full_costmap=true on both (live). local publish_frequency=2.0; global publish_frequency=1.0, 5x5 m @ 0.05 m, static_layer subscribes /map transient_; 1.67 Hz, 73,929 B/s; 0 msgs in 10 s (0 Hz, 0 B/s) | Confirmed as configured: costmap_updates carried zero messages on both costmaps while the full OccupancyGrid was re-sent every publish cycle (local ~74 KB/s at 1.67 Hz, global ~20 KB/s at 0.67 Hz, plus costmap_raw ~88 + ~25 KB/s). At these small map sizes (120x120 and 100x100 cells) the absolute cost is modest (~200 KB/s total), and with RViz absent the only consumers are behavior_server and smoother_server via costmap_raw, so the saving from partial updates is small today; it grows with map size. Note measured publish rates (1.67 / 0.67 Hz) are below the configured 2.0 / 1.0 Hz. | low |
| jetank_navigation-18 | measured | CPU mean 3.07% p95 9.9% peak 9.9%; RSS 53.04 MB; 14 threads; 103 vol / 6 invol ctx switches in 10 s; enable_interactive_mode=true, interactive_mode=false (live), map_update_interval=5.0, mode=mapping, minimum_time_interval=0.5, transform_publish_period=0.02; /map, /map_metadata, /pose, /slam_toolbox/graph_visualization (MarkerArray), /slam_toolbox/scan_visualization (LaserScan), /slam_toolbox/update (InteractiveMark | Not supported at runtime. enable_interactive_mode: true only creates the interactive-marker server and the toggle service; the live interactive_mode flag is false, so no InteractiveMarkerUpdate messages were published (0 in 15 s of observation) and the only graph traffic is /slam_toolbox/graph_visualization at 0.2 Hz / 558 B/s while the robot is stationary. Cost is one extra publisher/subscriber pair and a service; the parameter is harmless unless someone toggles it on (which would only matter with RViz). | low |
| jetank_navigation-28 | blocked | not present; stack is navigation-only with slam_toolbox providing map->odom; no saved map loaded. Launching AMCL is forbidden for this pass. | Blocked: AMCL is not running (navigation-only mode, no saved map), so its parameters cannot be measured live. | low |
| jetank_ros_main-35 | blocked | no rviz node present; headless Jetson, no display attached | Blocked: RViz is not running and there is no display, so unified.rviz defaults cannot be exercised or measured. | low |

Stack snapshot (idle): {"method": "sum of profile_node 10 s means per process; costmaps are included inside controller_server/planner_server", "processes": {"/controller_server (+local_costmap)": {"cpu_mean_pct": 3.56, "rss_bytes": 47661056, "threads": 16}, "/planner_server (+global_costmap)": {"cpu_mean_pct": 2.48, "rss_bytes": 42008576, "threads": 17}, "/bt_navigator": {"cpu_mean_pct": 3.27, "rss_bytes": 52133888, "threads": 11}, "/behavior_server": {"cpu_mean_pct": 1.98, "rss_bytes": 37314560, "threads": 12}, "/smoother_server": {"cpu_mean_pct": 1.88, "rss_bytes": 34922496, "threads": 13}, "/velocity_smoother": {"cpu_mean_pct": 0.4, "rss_bytes": 29868032, "threads": 11}, "/waypoint_follower": {"cpu_mean_pct": 0.1, "rss_bytes": 32075776, "threads": 11}, "/lifecycle_manager_navigation": {"cpu_mean_pct": 0.1, "rss_bytes": 32645120, "threads": 13}, "/slam_toolbox": {"cpu_mean_pct": 3.07, "rss_bytes": 53039104, "threads": 14}}, "total_cpu_mean_pct": 16.84, "total_rss_bytes": 361668608, "total_rss_mb": 361.7, "total_threads": 118, "idle_topic_bandwidth_bytes_per_sec": {"/local_costmap/voxel_grid": 506025, "/local_costmap/costmap_raw": 88201, "/local_costmap/costmap": 73929, "/global_costmap/costmap_raw": 24530, "/global_costmap/costmap": 19912, "/map": 8493, "/slam_toolbox/graph_visualization": 558, "/slam_toolbox/update": 0, "/local_costmap/costmap_updates": 0, "/global_costmap/costmap_updates": 0}, "note": "CPU% is per-process share of one core as sampled by profile_node; free -m not permitted, RSS sum used instead."}

**New finding from this pass — jetank_navigation-29 (medium, measured):** `nav2_params.yaml:152` uses the key `vth_samples: 40`; DWB reads `vtheta_samples`, so the intended 40 theta samples never apply and the default 20 is in effect (live params confirm vx_samples=20, vtheta_samples=20). The audit's "800 trajectories" claim (navigation-16) was therefore wrong at runtime (400/cycle); the real defect is a silently ignored tuning key. Fix: rename the key.

## Addendum 2026-09-27 — Measurement pass 2: arm stack (MoveIt + manipulation nodes on MOCK hardware)

Serial retry: `unified.launch.py hardware:=serial` — `Servo id 1 (S1_joint) did not respond to ping`, ros2_control_node exit -6 (12:07). The four feetech/serial findings remain blocked. Stack re-launched with `hardware:=mock`; grasp_server, base_approach_node, mobile_grasp_coordinator started with the hardware-launch parameters. One `/grasp_object` goal (empty target_pose, preset sequence) was sent on the mock arm: SUCCEEDED, 19.5 s wall, planning 34–47 ms per move, execution 1.7–3.7 s per move.

| id | status | key numbers | live verdict | severity now |
|---|---|---|---|---|
| jetank_motor_control-16 | blocked |  | Serial bus dead: servo id 1 does not answer on /dev/ttyTHS1 at 1 Mbps (ros2_control_node exited -6 on hardware:=serial at 12:07). Bus not touched. Note cited line 346 is beyond the current 290-line file. | medium (unverified) |
| jetank_motor_control-17 | blocked |  | Same serial-bus blocker; cited line 378 does not exist in the current 290-line file. | medium (unverified) |
| jetank_motor_control-18 | blocked |  | Same serial-bus blocker. Source at feetech_bus.cpp:135-150 confirms incremental parse with early return, so a present reply does not pay the full timeout; only a silent servo does. | medium (unverified) |
| jetank_motor_control-19 | blocked |  | Same serial-bus blocker. | low (unverified) |
| jetank_manipulation-02 | measured-partial | cpu 13.10 %, rss 71.1 MB, threads 12, ctxsw vol/invol 15863/4060 (lifetime); cpu 14.00 %, rss 72.8 MB, threads 12 | Not exercised (only the coordinator/base_approach call spin_until_complete and neither ran a goal). Idle numbers only; the 50 Hz poll is not what drives the 13-14 % idle CPU (see -05). | low |
| jetank_manipulation-03 | measured | 1790492854.060 -> 859.270 (5.21 s = 5.0 s gripper wait_for_server timeout + 0.2 s dwell sleep); 861.080 -> 866.890 (5.81 s = 5.0 s timeout + 0.8 s dwell sleep) | The blocking sleeps exist (1.0 s total per goal) but with 6 executor threads they had no observable effect: feedback kept flowing and the goal completed. The 10 s of gripper wait_for_server timeouts dwarf them. | low -> info |
| jetank_manipulation-04 | measured-partial | idle cpu 14.00 %, rss 72.8 MB, threads 12 | Coordinator SEGMENT path not exercised (execute_sock_grasp forbidden). Idle only. | low |
| jetank_manipulation-05 | measured | TransformListener(self._tf_buffer, self) - no spin_thread kwarg; installed tf2_ros default spin_thread=False; TransformListener(..., spin_thread=False) explicit | Claim as written is wrong: spin_thread is False, so no dedicated spin thread and no hidden node. The real cost is elsewhere: the two nodes that own a Python TransformListener each burn 13-14 % of a core idle, consistent with servicing /tf at 47 Hz through Python callbacks on the 6-thread executor, versus 0.1 % for grasp_server which has none. Reword the finding to 'Python TransformListener on /tf@47 Hz costs ~13 % CPU per node idle'. | reword; raise to medium (27 % of a core idle across the two nodes) |
| jetank_manipulation-06 | measured-partial | GripperCommand server /gripper_controller/gripper_cmd 'not available' -> 5.0 s wait_for_server timeout x2 = 10.0 s of the 19.5 s goal; wait_for_server(timeout_sec=grasp_timeout_s=60.0) confirmed | Coordinator service not exercised, but the same wait_for_server-equals-timeout pattern in grasp_server was measured live: a missing gripper action server cost 10 s per goal. Supports the claim that a missing server blocks for the full timeout (60 s in the coordinator). | low -> medium |
| jetank_manipulation-07 | measured | 6; grasp_server 11, base_approach 12, coordinator 12 (all 'python3' named; 6 executor workers + main + rclpy/DDS threads) | Thread counts are consistent with a 6-worker MultiThreadedExecutor per node (3 nodes x 6 = 18 worker threads for at most 2-3 concurrent callbacks). The idle CPU cost of the idle workers themselves is negligible (grasp_server 0.1 %); memory ~70-77 MB per node. | low |
| jetank_manipulation-11 | measured | 3 calls per goal (one per 'Reached ... (error_code=1 SUCCESS)' line); grasp_server total 1.45 CPU-s at age 384 s; increment since pre-goal ps snapshot ~0.43 CPU-s over 270 s with 0.10 % idle baseline => <=~0.2  | Three reflective lookups per goal inside a ~1 % CPU process; unmeasurable against 8.2 s of trajectory execution and 10 s of gripper timeouts. | low -> info |
| jetank_manipulation-12 | measured-partial | 'legacy preset grasp (no target_pose)' - pose-target builder (grasp_server.py:301) not executed | Only reachable on the pose-target path, which this preset goal does not take. Idle only. | low |
| jetank_manipulation-18 | measured-partial | 14.0 % / 13.1 % CPU, 72.8 / 71.1 MB | cloud_to_xyz only runs on a detection; none available. Idle only. | low |
| jetank_manipulation-19 | measured-partial | 14.0 % CPU, 72.8 MB | pca_long_axis_yaw only runs on a detection. Idle only. | low |
| jetank_manipulation-20 | measured-partial | 13.10 % CPU, 71.1 MB, 12 threads, no approach goal running | Control loop not exercised (base_approach forbidden). The 13 % idle CPU is present without the loop running, so it is not caused by the per-tick re-import/param read; see -05. | low |
| jetank_manipulation-32 | measured | 'Sending MoveGroup goal' 851.2007 -> move_group 'Received request' 851.2118 = 11 ms (x3 moves: 11, 2, 1 ms); 5.0 s timeout paid twice per goal | Re-querying wait_for_server costs ~1-11 ms when the server is up, so the claim's cost is negligible in the success path; the real cost is the 5 s timeout when a server is absent, paid on every command. | low |
| jetank_moveit_config-11 | blocked |  | demo.launch.py not used in the live bringup. | low (unverified) |
| jetank_moveit_config-17 | blocked |  | RViz not running, no display. | low (unverified) |
| jetank_ros_main-24 | measured | static_tf 1.02 s, robot_state_publisher 1.07 s, imu 1.11 s, rplidar 1.14 s, ros2_control_node 1.18 s (hardware configured 1.19 s), move_grou; move_group context init complete 1.88 s, 'You can start planning now' between 1.88 and 2.10 s; gripper_controller activated 2.32 s; arm_cont | On the mock hardware path the whole arm stack (controllers + move_group) is ready 3.5 s after launch, versus 16-46 s fixed staggers in the mobile_grasp launches (5-13x). Supports the claim; Gazebo sim path not measured so the 18 s move_group wait there is unverified. | low (medium if mobile_grasp launches remain the hardware entry point) |
| jetank_moveit_config-07 | measured | ompl.arm.longest_valid_segment_fraction=0.005, ompl.gripper=0.01, arm planner RRTConnect, kinematics KDL timeout 0.05; grasp_pre 44 ms, grasp_reach 34 ms, home 47 ms | Even at 0.005 the whole planning phase is 34-47 ms per 3-DOF move and move_group CPU did not measurably rise during the goal. Doubling collision checks is real but costs tens of ms; not worth a finding above info. | low -> info |
| jetank_moveit_config-09 | measured | 16 links carry collision geometry (chassis, arm_bearing, S1-S5, gripper_base/left/right, camera_link, 4 wheels, laser) => 120 pairs; SRDF di; 34-47 ms including collision checking | Structurally true (86 % of pairs still checked, wheels/laser/chassis never disabled), but the measured planning time is tens of milliseconds, so the cost is not observable at this arm size. | low |

goal_exercise: {"action": "/grasp_object (jetank_manipulation/action/GraspObject), goal {object_hint: ''} empty target_pose -> preset sequence", "result": "SUCCEEDED: success=true 'Grasp sequence completed successfully.'; stages moving_to_pre_grasp, opening_gripper, moving_to_grasp, closing_gripper, parking, done", "wall_time_s": 19.53, "timeline_epoch": {"goal_received": 1790492851.191, "grasp_pre_reached": 1790492854.058, "grasp_reach_reached": 1790492861.078, "home_reached": 1790492870.719, "success": 1790492870.726}, "planning_times_s": [0.044, 0.034, 0.047], "execution_times_s": [2.76, 1.74, 3.74], "gripper_wait_for_server_timeouts_s": [5.0, 5.0], "side_finding": "GripperCommand action server /gripper_controller/gripper_cmd reported 'not available' by grasp_server although gripper_controller was 'Configured and activated' at launch; 10.0 of the 19.5 s goal were spent in these timeouts. move_group also warns 'Unable to transform object from frame world to planning frame base_footprint' on every request.", "move_group_cpu_during": "NOT directly captured (30 s profile ended before the goal); derived goal-window increment ~0 +/- 0.2 CPU-s over 19.5 s, i.e. no measurable rise above the 3.3 % idle mean", "grasp_server_cpu_during": "NOT directly captured (sampler 1790492818-848 vs goal 851-871); derived <=~0.2 CPU-s over the goal (~1 %), idle 0.10 %", "controller_manager_cpu_during": "NOT directly captured; derived ~0.5-0.7 CPU-s over the goal (~3 %) vs 2.0 % idle; goals: 3 accepted, 3 'Goal 

stack_snapshot: {"idle_cpu_pct_sum": 32.55, "idle_cpu_pct_by_node": {"move_group": 3.27, "controller_manager": 2.08, "grasp_server": 0.1, "base_approach_node": 13.1, "mobile_grasp_coordinator": 14.0}, "rss_mb_sum": 331.8, "rss_mb_by_node": {"move_group": 67.8, "controller_manager": 42.6, "grasp_server": 77.4, "base_approach_node": 71.1, "mobile_grasp_coordinator": 72.8}, "threads_by_node": {"move_group": 21, "controller_manager": 21, "grasp_server": 11, "base_approach_node": 12, "mobile_grasp_coordinator": 12}, "tf_hz": 47.0}

**New findings from this pass:**
- **jetank_manipulation-33 (medium, measured):** `base_approach_node` and `mobile_grasp_coordinator` each sit at ~13–14 % CPU and ~72 MB *idle*, versus 0.1 % for `grasp_server`. The difference is the Python `TransformListener` consuming `/tf` at 47 Hz (robot_state_publisher + ros2_control broadcasting). Fix options: a lower-rate `/tf` publisher for the arm, `spin_thread=False` with an explicit executor, or a C++ TF consumer. This replaces the reasoning in manipulation-05 (whose `spin_thread=True` claim is wrong as written: coordinator.py:105 does not pass it).
- **jetank_manipulation-34 (medium, measured):** `grasp_server` logs `/gripper_controller/gripper_cmd` "not available" and spends two 5 s `wait_for_server` timeouts per goal (10.0 s of the 19.5 s goal) even though `gripper_controller` is active. Either the action name differs from what the controller exposes or the client is created before discovery; check `ros2 action list` against grasp_server's configured name.
- **Note on jetank_motor_control-16/17:** the cited lines (jetank_serial_hardware.cpp:346/378) do not exist — the file is 290 lines; the auditor's line anchors are stale. Re-anchor before applying.


## Addendum 2026-09-27 — Follow-ups that stay static (blocked), with the exact blocker

| id | severity | blocker (verified 2026-09-27) |
|---|---|---|
| jetank_detection-01 | medium | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-02 | medium | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-04 | medium | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-05 | medium | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_motor_control-17 | medium | servo bus silent: servo id 1 no ping on /dev/ttyTHS1 @ 1 Mbps (ros2_control_node exit -6, 12:07) |
| jetank_motor_control-18 | medium | servo bus silent: servo id 1 no ping on /dev/ttyTHS1 @ 1 Mbps (ros2_control_node exit -6, 12:07) |
| jetank_simulation-09 | medium | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-14 | medium | Gazebo pass declined by user (headless sim skipped) |
| jetank_web_control-05 | medium | needs a browser client connected to web_control; none available in this run |
| jetank_motor_control-16 | medium | servo bus silent: servo id 1 no ping on /dev/ttyTHS1 @ 1 Mbps (ros2_control_node exit -6, 12:07) |
| jetank_description-27 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_description-28 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_detection-03 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-06 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-07 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-08 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-10 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-11 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_detection-12 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |
| jetank_motor_control-19 | low | servo bus silent: servo id 1 no ping on /dev/ttyTHS1 @ 1 Mbps (ros2_control_node exit -6, 12:07) |
| jetank_moveit_config-11 | low | reason: demo.launch.py not used in the live bringup |
| jetank_moveit_config-17 | low | RViz cannot run: headless Jetson, no DISPLAY |
| jetank_navigation-19 | low | RViz cannot run: headless Jetson, no DISPLAY |
| jetank_navigation-28 | low | AMCL not launched: navigation-only mode, no saved map |
| jetank_perception-10 | low | reason: camera_node (single_camera.cpp) not running |
| jetank_perception-40 | low | sim-only input path; Gazebo pass skipped |
| jetank_ros_main-16 | low | reason: diagnostic scripts not run |
| jetank_ros_main-35 | low | RViz cannot run: headless Jetson, no DISPLAY |
| jetank_simulation-01 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-02 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-04 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-05 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-06 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-07 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-11 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-15 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-16 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_simulation-18 | low | Gazebo pass declined by user (headless sim skipped) |
| jetank_web_control-10 | low | needs a browser client connected to web_control; none available in this run |
| jetank_web_control-13 | low | needs a browser client connected to web_control; none available in this run |
| jetank_web_control-15 | low | reason: cmd_vel_bridge (sim-only) not running |
| jetank_detection-09 | low | sock detector cannot run: `ultralytics` is not in the pixi env and no model file exists (`~/models/` absent) |

42 items remain static (jetank_ros_main-24 was measured in pass 2).

## Addendum 2026-09-27 — Measurement pass 3: sock_segmentation_server against the live camera

Stack: system-ROS stereo camera + pixi-built sock_segmentation_server. No detector exists, so one synthetic Detection2DArray (bbox centre 320,180, 80x80, frame camera_left_link) was published at 5 Hz and one /segment_socks goal sent. Goal SUCCEEDED with found=false: the bbox blob had 150 points, 15 after ground removal, below min_points=30, so the transform stage was not reached.

| id | status | key numbers | live verdict | severity now |
|---|---|---|---|---|
| jetank_perception-12 | measured | 8 INFO lines for one goal with one detection (1 goal-received + 3 [snapshot] + 3 [det] + 1 summary); 4 of them are per-detection stage lines so this s; scan of 230400 px (640x360 32FC1) completed between log line 3 (.315525) and line 4 (.315871): <=0.35 ms; reported 6880/230400 valid pixels | Confirmed as written: per-goal full-frame scan exists and logs are per-stage, but at 640x360 the scan costs <0.35 ms and the 8 INFO lines are ~2% of the 15.5 ms execute span. No measurable CPU/RSS impact at the actual goal cadence (one-shot goals). | low (keep); could be downgraded to info/cosmetic — convert [snapshot]/[det] lines to DEBUG and drop the scan, but there is no performance case at 640x360 |
| jetank_perception-13 | blocked | NOT REACHED: goal dropped at ground-removal stage (15 < min_points 30), so toROSMsg/doTransform/fromROSMsg and the two cloud copies never executed; 0 messages (debug publish not reached) | Could not exercise live: the synthetic bbox did not survive ground removal on the current scene, and a second attempt is not permitted for this reason (not a frame_id/stamp issue). Static reading confirms the round-trip and two copies; with blobs of a few hundred points the cost is microseconds, far below the 14 ms ground-removal step observed. | low (keep); expected cost is negligible relative to RANSAC ground removal in the same path |
| jetank_perception-41 | measured-partial | 12 / 12 (max) / 12; detached thread lived ~15 ms and was not observable at 1 s resolution; no thread accumulation after the goal; 2.71 s end-to-end for the ros2 action send_goal CLI (includes pixi/CLI startup and server discovery); server-side execute span from log = 15.5 ms | Detached-thread-per-goal confirmed statically; no leak or thread growth observed after one goal. The TF-blocking claim could not be exercised (TF stage not reached); static reading shows the worst case is 2 x 0.4 s = 0.8 s per goal, which the finding understates. Blocking is in a detached thread and does not stall the executor. | low (keep); correct the text to 'up to 0.8 s (two lookups x 0.4 s)' and note that concurrent goals would each spawn a thread with no cap |

goal_exercise: {"attempts": 1, "frame_id_used": "camera_left_link (read from /stereo_camera/left/camera_info; the prompt's left_camera_optical_frame was not the live frame)", "synthetic_detection": "5 Hz Detection2DArray, bbox 80x80 at (320,180), score 0.9", "goal_status": "SUCCEEDED", "result_found": false, "fail_stage": "ground: reprojected points=150 -> after ground removal 15 < min_points 30", "feedback": {"processed": 1, "total": 1}, "wall_time_s": {"cli_send_goal_incl_discovery": 2.71, "server_execute_span_from_log": 0.0155}, "server_log_timestamps": {"received_goal": 1790493327.315002, "final_line": 1790493327.330497}, "log_lines_per_goal": 8, "log_lines_breakdown": ["Received SegmentSocks goal", "[snapshot] present", "[snapshot] disparity image ... valid pixels=6880/230400", "[snapshot] now/stamps/ages", "[det 0/1] reprojected points=150", "[det] after ground removal points=15", "[det] dropped ... stage=ground", "No valid sock blob ... found=false"], "threads_before_after": {"before": 12, "during_25s_1Hz_sampling_max": 12, "after": 12, "note": "detached execute thread lived ~15 ms, below the 1 s sampling resolution; no thread leak observed"}, "cpu_pct_during_goal_window": {"avg": 4, "max_

stack_snapshot: {"date": "2026-09-27", "ros_domain_id": 42, "server_pid": 13879, "server_binary": "/home/koen/workspaces/ros2_ws/install/jetank_perception/lib/jetank_perception/sock_segmentation_server", "server_params": {"max_age": 1.0, "max_sync_dt": 0.5, "min_points": 30, "remove_ground": true, "ground_filter": "height", "ground_margin": 0.012, "default_target_frame": "base_link", "base_frame": "base_link"}, "idle_server": {"tool": "bash /proc sampling (profile_node refused: 'Ambiguous: 3 processes match', PIDs 13785 pixi / 13876 ros2 python / 13879 binary)", "cpu_pct_avg": 4.5, "rss_kb": 58568, "threads": 12, "voluntary_ctxt_switches": 7117, "nonvoluntary_ctxt_switches": 1629}, "disparity_topic": {"tool": "measure_topic_perf 10 s", "hz": 0.621, "count": 6, "bw_bytes_per_sec": 1669716.3, "jitter_sec": 0.03075, "latency_ms": {"p50": 332.42, "p95": 351.53, "p99": 353.12}, "drop_estimate": 0}, "camera_info_topic": {"tool": "get_topic_hz 5 s", "hz": 32.599, "count": 120, "frame_id": "camera_left_link", "size": "640x360"}, "detector_running": false, "note": "Disparity arrives at 0.62 Hz (period ~1.6 s) while the server's max_age is 1.0 s, so a goal has a substantial chance of hitting the 'Stale inpu

**New observations from this pass:**
- **jetank_perception-42 (medium, measured):** the disparity topic period (~1.6 s at 0.62 Hz, see perception-05) exceeds the segmentation server's `max_age` (1.0 s), so `/segment_socks` goals will intermittently fail on the "Stale input" branch purely by phase. Fixing perception-05 (disparity QoS/rate) removes this; otherwise raise `max_age` or make it relative to the measured disparity period.
- Disparity frame had only 6,880 of 230,400 pixels valid (3 %) in this scene — worth re-checking the stereo tuning before relying on segmentation on hardware.
- Correction to perception-41 text: worst-case TF blocking is 0.2 s + 0.2 s per lookup x 2 lookups = 0.8 s, not 0.4 s; the detached thread lived ~15 ms and did not leak.


## Post-fix addendum 2026-09-27 — high + medium findings applied (session 20260927-093513-apply-audit-high-medium)

Branch `chore/efficiency-fixes` in every repo (from each repo's HEAD; unmerged, unpushed). 74 commits: one per finding plus grill-review fix commits (2 adversarial review rounds per phase). Gates: `pixi run build` exit 0 after every phase; full `colcon test` with only the pre-existing offline xmllint failures; post-Phase-C clean rebuild exit 0 (2 min 15 s, BUILD_TESTING off); smoke test 52 executables/launch files, 0 failures; ldd clean.

**Applied:** all 7 high; 42 of 46 medium. **Deferred:** jetank_motor_control-16/17/18 (Feetech serial timing — bus still silent, cannot verify), cross-07 (track width 0.14 sim vs 0.11 hardware — needs a physical wheel-separation measurement first). **Tagged needs-runtime-validation:** detection-01/02/04/05, simulation-09/14/19.

### Before / after (live, same tools as the audit)

| metric | before | after | note |
|---|---|---|---|
| stereo_camera_node CPU mean idle (no disparity/points subscriber) | ~190-195 % | 47.3 % (p95 58.95, peak 59.2) | PID 79013 |
| stereo_camera_node RSS idle | ~440 MB | 378 MB mean (peak 385 MB) |  |
| stereo_camera_node threads | 32 | 30 |  |
| stereo_camera_node CPU mean while /stereo_camera/disparity subscribed | ~190-195 % (always computed) | 46.73 % (p95 49.4) | RSS 387 MB mean |
| stereo_camera_node CPU mean while /stereo_camera/points subscribed | ~190-195 % (always computed) | 46.78 % (p95 49.88) | RSS 394 MB mean |
| /stereo_camera/disparity Hz (with subscriber) | 0.60 Hz | 0.625 Hz (9 msgs / 15 s) |  |
| /stereo_camera/disparity bandwidth | 1.66 MB/s | 1.71 MB/s |  |
| /stereo_camera/disparity latency p50 (header.stamp) | ~350 ms | 322.7 ms (p95 338.6, p99 342.7) | jitter 64.9 ms, drop_estimate 0 |
| /stereo_camera/points Hz (with subscriber) | 27.8 Hz | 4.89 Hz (48 msgs / 10 s), 2.30 MB/s, latency p50 340.2 ms | rate dropped vs before; drop_estimate 0, jitter 13.9 ms |
| /stereo_camera/left/camera_info Hz | 34 Hz | 30.04 Hz (150 msgs / 5 s) |  |
| /stereo_camera/left/image_raw/compressed bandwidth | ~4 MB/s (30 msgs / 5 s) | 1.62 MB/s (149 msgs / 5 s = 29.8 Hz) | per-message size ~54 KB |
| /stereo_camera/left/image_raw encoding | n/a | NOT OBSERVABLE: read_topic (which subscribes) received 0 messages in 5 s and again in 10 s | topic is advertised but delivered nothing to the MCP subscriber; encoding (expected bgra8) could not be confirmed |
| web_control_node CPU idle (zero viewers) | not recorded (decoded ~30 fps JPEG) | 1.78 % mean (p95 9.9), RSS 90.2 MB, 12 threads | ps pcpu lifetime avg 2.2 % |
| web_control_node subscription to /stereo_camera/left/image_raw/compressed with zero viewers | subscribed (always decoding) | NO image subscription. Subscribers: /amcl_pose, /detections/socks, /map, grasp_object + navigate_to_pose action feedback/status | lazy camera subscription confirmed |
| icm20948_imu CPU idle (no subscribers) | not recorded (published 3 topics at 100 Hz regardless) | 1.58 % mean, RSS 27.9 MB, 11 threads, 1014 voluntary ctx switches / 10 s |  |
| /imu/data_raw Hz once subscribed | 100 Hz | 100.02 Hz (501 msgs / 5 s) | get_topic_hz itself subscribes, so silence-without-subscribers cannot be observed directly by this tool; node still advertises /imu/data_raw, /imu/magnetic_field, /imu/temperature |
| rplidar_node CPU / RSS | 8.6 % / 26 MB | 10.46 % mean (p95 19.7) / 26.9 MB | baseline, no fix expected |
| base_approach_node CPU idle / RSS | 13.1 % / 71 MB | 0.4, 0.4, 0.3 % (ps pcpu, 3 samples) / 65.7 MB, 11 threads | ps pcpu is process-lifetime average, PID 79936 |
| mobile_grasp_coordinator CPU idle / RSS | 14.0 % / 73 MB | 0.4, 0.4, 0.4 % (ps pcpu) / 67.6 MB, 11 threads | PID 79938 |
| grasp_server CPU idle / RSS | 0.1 % | 0.4, 0.4, 0.4 % (ps pcpu) / 73.3 MB, 11 threads | PID 79937 |
| move_group CPU idle | 3.3 % | 4.04 % mean (p95 9.9, peak 19.7), RSS 65.1 MB, 21 threads |  |
| move_group CPU during grasp goal (30 s window) | not recorded | 4.05 % mean (p95 9.9, peak 19.8), RSS 68.9 MB |  |
| controller_manager CPU idle | 2.1 % | 2.87 % mean (p95 9.9), RSS 43.4 MB, 21 threads | profile_node '/controller_manager' was Ambiguous (5 PIDs); matched by process name instead |
| robot_controller CPU idle | not recorded | 1.48 % mean, RSS 26.9 MB, 11 threads | after only |
| grasp goal wall time (CLI, incl. pixi + ros2 CLI startup) | 19.5 s | 13.72 s (RC=0, SUCCEEDED) | server-side goal received->SUCCESS = 9.61 s (log 776.037 -> 785.650) |
| 5 s gripper wait_for_server timeouts per goal | 2 x 5 s = 10.0 s | 0 (grep 'not available' -> no hits; grep 'wait' -> only startup line 'Waiting for /move_action'). GripperCommand round-trips 58 ms and 57 ms |  |
| per-move planning time | 34-47 ms/move | NOT LOGGED: final_grasp_server.log has no planning-time lines. Send->Reached (plan+execute) per move: grasp_pre 2.83 s, grasp_reach 1.77 s, home 3.84 s |  |

### Still open after the apply run
- **jetank_perception-05 NOT resolved:** `/stereo_camera/disparity` still delivers 0.625 Hz to a subscriber (KeepLast(5) on a RELIABLE 0.9 MB message did not change delivery; the verifier had ruled best-effort out because sock_segmentation_server documents needing RELIABLE). Next step: measure with a co-located C++ subscriber vs the MCP Python subscriber to separate transport from consumer cost; consider a compressed/half-resolution disparity or shared-memory transport.
- **Point cloud rate 27.8 → 4.9 Hz, but ~9x more points per message (53 KB → 470 KB):** the corrected rectification (perception-22) and Q sign now yield a dense valid cloud; the lower rate is the consumer-side cost of 2.3 MB/s clouds plus StatisticalOutlierRemoval on 29k points. Re-tune `pointcloud` voxel/range filters for the new density (perception-09 made SOR optional — verify it is disabled in the shipped config).
- **IMU lazy publishing** could not be observed directly (the measurement tool subscribes); idle CPU 1.6 %.
- Minors left from review rounds (all documented in the plan run log): bgra8 raw is 33 % more bytes than bgr8 (trade-off), redundant double can_transform wait, no tests pinning the BGR8 pipeline template or the cold-buffer TF path, SRDF virtual_joint stays type fixed, jetank_simulation lacks exec_depend jetank_motor_control.
- **Build-tree note:** anyone building this branch on top of an old `build/` must delete the dangling symlink `build/jetank_ros_main/config/motor_params.yaml` (file moved to jetank_motor_control) or clean `build/jetank_ros_main`.

### pixi.toml (Phase C, user-confirmed re-solve)
Default env = robot set (ros-base + explicit runtime deps + gripper-controllers); `desktop` feature (rviz2, rviz-default-plugins, jsp-gui, teleop-twist-keyboard); `sim` feature (ros-gz-*, ign-ros2-control; linux-64 only); `detect` pypi feature (ultralytics, opt-in); environments default/dev/detect; build tasks `-DBUILD_TESTING=OFF`, `build-tests` feeds `test`; `gazebo`/`detect`/`urdf` tasks moved into their features (`pixi run -e dev gazebo`). Env 5.9 GB → 5.2 GB, 792 packages; rviz2 remains via the moveit metapackage. Backups: `pixi.toml.bak-2026-09-27`, `pixi.lock.bak-2026-09-27`. Do not pin python (rplidar-ros needs 3.11/3.12; env is 3.12).

### Commits per repo

### jetank_description — 0 commits on chore/efficiency-fixes

### jetank_detection — 4 commits on chore/efficiency-fixes
- 3621327 perf(detection): expose imgsz/half/device to predict() [jetank_detection-05]
- c8818c2 perf(detection): warm up CUDA/cuDNN in load() [jetank_detection-04]
- 335be45 perf(detection): throttle continuous-mode inference to max_rate_hz [jetank_detection-02]
- c82c229 perf(detection): replace action busy-poll with Condition wait [jetank_detection-01]

### jetank_manipulation — 12 commits on chore/efficiency-fixes
- fa7c9c1 fix(manipulation): SRDF guard catches TypeError and empty state set [cross-09 review r2]
- 4c23a32 fix(manipulation): parse SRDF states at server start; pep257 [cross-09 review]
- 942a43c perf(manipulation): read SRDF group_states instead of a hand-mirrored dict [cross-09]
- fcb2641 docs(manipulation): grasp_server docstring states the restored gripper_action default
- 198d408 fix(manipulation): gripper_action default is the per-controller name; hw launch overrides [jetank_manipulation-34 review r2]
- d82a89c style(manipulation): D213 docstring summaries on second line [jetank_manipulation-33 review r2]
- 77ea6f6 fix(manipulation): grasp_server gripper action name matches the real endpoint [jetank_manipulation-34 review]
- 2ba40ab fix(manipulation): fix cold-buffer TF snapshot + listener teardown race [jetank_manipulation-33 review]
- 4d5469b perf(manipulation): create TF listeners lazily, only for an active goal [jetank_manipulation-33]
- 56c2541 perf(manipulation): stop burning 2x5s/goal on gripper action discovery [jetank_manipulation-34]
- 6419267 perf(manipulation): guard empty t_approach in pose-targeted grasp [jetank_manipulation-01]
- ba48042 perf(manipulation): planning frame world -> odom [jetank_ros_main-01-followup]

### jetank_motor_control — 1 commits on chore/efficiency-fixes
- 95fb043 perf(motor_control): add motor_params.yaml with the node's real param names [cross-06]

### jetank_moveit_config — 4 commits on chore/efficiency-fixes
- 53c6217 docs(moveit_config): fix stale 'arm' group / S5 documentation [cross-09]
- 9a294d0 perf(moveit_config): bound spawner wait with controller-manager-timeout [jetank_moveit_config-01]
- a626ef2 fix(moveit_config): demo static TF and docs follow odom planning frame [jetank_ros_main-01 review r2]
- 23fcfa4 fix(moveit_config): virtual_joint parent_frame world->odom [jetank_ros_main-01 review]

### jetank_navigation — 6 commits on chore/efficiency-fixes
- b21c287 fix(navigation): use_sim_time default 'false' for consistency with unified.launch.py [cross-01 review]
- 49e05aa perf(navigation): delegate navigation_full.launch.py to unified.launch.py [cross-01]
- 1977d0d perf(navigation): drop unused waypoint_follower/velocity_smoother/smoother_server [jetank_navigation-17]
- ded868f perf(navigation): shrink DWB trajectory sample budget for 0.3 m/s robot [jetank_navigation-16]
- 40ce892 perf(navigation): fix misspelled vth_samples key in DWB config [jetank_navigation-29]
- f5c7a72 perf(navigation): gate IMU publish on subscriber count [jetank_navigation-01]

### jetank_perception — 23 commits on chore/efficiency-fixes
- 422b46b fix(perception): shared raw_frame_encoding for camera_node; PNG drops BGRx padding [jetank_perception-04 review r2]
- fc70548 fix(perception): make the default BGR8 CSI pipeline colour-preserving and CPU-videoconvert-free [jetank_perception-04 review]
- 87e75a0 fix(perception): remove dead quality_monitoring.metrics/calibration_validation keys [jetank_perception-20 review]
- 93ee94a fix(perception): construct camera info managers on left/right sub-nodes [jetank_perception-21 review]
- 397b022 fix(perception): raise segmentation server max_age default to 2.0s [jetank_perception-42]
- b83bab1 perf(perception): scope PCL find_package/link to used components [footprint-12]
- 323cb40 chore(perception): wire or remove dead launch arguments [jetank_perception-28]
- 5428f52 chore(perception): collapse pipeline template/cache machinery to one function [jetank_perception-23]
- e5ed086 chore(perception): remove duplicate calibration services and fake stereo calibration [jetank_perception-21]
- 3bb4b1e chore(perception): remove 21 dead declared parameters [jetank_perception-20]
- a0818cd perf(perception): apply camera.processing_threads via cv::setNumThreads [jetank_perception-15]
- 76dfada perf(perception): drop cv_bridge/image_transport, remove second OpenCV runtime [jetank_perception-14]
- 9e9427b perf(perception): default statistical outlier filter off in point cloud stage [jetank_perception-09]
- b840807 perf(perception): cache disparity f/t from calibration, fix delta_d [jetank_perception-07]
- a6d6ed4 perf(perception): build GStreamer pipeline from camera.format/fps, add hw GRAY8 path [jetank_perception-04]
- b037688 perf(perception): bound appsink queue depth instead of no-op CAP_PROP_BUFFERSIZE [jetank_perception-03]
- af55986 fix(perception): guard stage-1 quality modulo; rect consumers respect publish mode [jetank_perception-01 review r2]
- 79cc5f3 perf(perception): skip rectify remap when nothing consumes rectified/depth output [jetank_perception-01 review]
- 45f7a28 fix(perception): correct Q(3,2) sign in camera-info calibration path [jetank_perception-22 review]
- 6341282 fix(perception): load calibrated R/P from CameraInfo instead of re-deriving parallel-camera rectification [jetank_perception-22]
- f699e86 perf(perception): raise disparity publisher queue depth to KeepLast(5) [jetank_perception-05]
- cd397e1 perf(perception): rectify mono, not BGR, and fix rect image mono8 mislabel [jetank_perception-02]
- c231b3e perf(perception): skip disparity compute when nothing consumes it [jetank_perception-01]

### jetank_ros_main — 14 commits on chore/efficiency-fixes
- e4830e1 fix(ros_main): remaining enable_* gates compare case-insensitively [cross-01 review r2]
- 35449c7 fix(ros_main): sock_detector_autostart starts polling after the detector timer; D213 [cross-02 review]
- 1fd11be fix(ros_main): hardware-layer gates compare use_sim_time case-insensitively [cross-01 review]
- be925a9 perf(ros_main): move motor_params.yaml to jetank_motor_control, drop dead keys [cross-06]
- 6fa041d perf(ros_main): deduplicate sock_detector lifecycle autostart [cross-02]
- ae66f27 perf(ros_main): gate unified.launch.py hardware layer for navigation_full reuse [cross-01]
- ead0457 fix(ros_main): pass gripper_action=/controller_manager/gripper_cmd to grasp_server on hardware [jetank_manipulation-34 review r2]
- b158782 chore(ros_main): use exec_depend for sibling JeTank packages [jetank_ros_main-10]
- 6745031 perf(ros_main): shut down orphaned MoveIt spawners/move_group on controller_manager crash [jetank_ros_main-06]
- fb67b4a perf(ros_main): gate lidar/IMU includes behind enable_lidar/enable_imu args [jetank_ros_main-04]
- a093984 perf(ros_main): gate joint_state_publisher off when MoveIt owns /joint_states [jetank_ros_main-02]
- 670f273 docs(ros_main): state publish_odom requirement for MoveIt planning frame [jetank_ros_main-01 review r2]
- bde572d fix(ros_main): drop static world->odom TF; MoveIt plans in odom [jetank_ros_main-01 review]
- 9f4361d perf(ros_main): fix world/odom dual-parent TF conflict on base_footprint [jetank_ros_main-01]

### jetank_simulation — 4 commits on chore/efficiency-fixes
- 98ea843 fix(simulation): launch test count 6->5; gripper_controllers not position_controllers [jetank_simulation-09/-19 review]
- ec3ff91 perf(simulation): depend on specific controller packages, not metapackages [jetank_simulation-19]
- 3bfc5e0 perf(simulation): raise physics step size to 2ms in all worlds [jetank_simulation-14]
- 5aa5681 perf(simulation): sequence robot_remote controller spawners [jetank_simulation-09]

### jetank_web_control — 6 commits on chore/efficiency-fixes
- 9456aaf fix(web_control): decode bgra8/rgba8 raw images [jetank_perception-04 review r2]
- bb9ded5 chore(web_control): declare undeclared exec_depends [footprint-28]
- 4755fa9 perf(web_control): stop repeating zero-velocity WS sends while idle [jetank_web_control-05]
- d419670 perf(web_control): grayscale + optimize=False map PNG encode [jetank_web_control-03]
- 6e98e5d fix(web_control): clear cached frame on unsubscribe; capture never saves stale frame [jetank_web_control-01 review]
- 7d62dfb perf(web_control): gate camera subscription on active viewers [jetank_web_control-01]



## Correction 2026-09-27 — disparity and point-cloud rates were measurement artifacts

A dedicated rclpy probe (reliable and best-effort, from both the system-ROS and the pixi environment) measured the real delivery on the post-fix stack:

| topic | ros2-mcp tool reported | real (probe) |
|---|---|---|
| /stereo_camera/disparity | 0.60–0.63 Hz, ~330 ms | 30.1 Hz, p50 latency 16–17 ms, reliable and best-effort, both environments |
| /stereo_camera/points | 4.9 Hz (27.8 Hz pre-fix) | 30.0 Hz, ~2,600 points per cloud, p50 latency 24 ms |
| stereo_camera_node CPU | 47 % "while subscribed" | 40 % idle, 120 % while serving disparity + points at 30 Hz (/proc sampling) |

Consequences: **jetank_perception-05** (disparity 0.6 Hz) was never a real defect — the only evidence was the ros2-mcp `measure_topic_perf` / `get_topic_hz` tools, which under-report large messages. Its KeepLast(5) change is harmless and stays. **jetank_perception-42** (disparity period > segmentation max_age) rests on the same false premise; its change is harmless. The "point cloud rate dropped / 9x denser, retune filters" item is withdrawn. The 190 % → 47 % "subscribed" figure is replaced by 190 % → 120 % at a real 30 Hz; the idle saving (≈190 % → 40 %) stands. Note: `ros2 topic hz` / `bw` on this Jetson produced no output for these topics within 25 s (sensor-data QoS CLI subscriber), so neither CLI nor MCP rates can be trusted for multi-MB messages here — use a dedicated probe.
