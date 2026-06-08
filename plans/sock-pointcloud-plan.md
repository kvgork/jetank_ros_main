# Plan — Sock 3D point-cloud blob (toward grasp poses)

**Status:** Phases 0–4 + 6 done + sim-verified end-to-end · **Created:** 2026-06-07 · **Mode:** sim-first
**Progress (branch `feature/sim-disparity-source`):** P1 sim disparity/cloud restored (`ros_topics` input source) ✅ · P1b optical frames RPY(-90,0,-90) ✅ · P2 SegmentSocks action + stub ✅ · P3 real segmentation server (sim `found=true`, centroid base_link) ✅ · **P4 full detection-in-loop with `sock_sim.pt`** — real sock detected (score 0.79), `remove_ground=true` → 5–8 cm sock blob, centroid base_link x=1.03 z=-0.02 ✅ · **P6 grasp-pose module** (`grasp_pose_node`: `/segment_socks` → top-down grasp PoseStamped, togglable PCA yaw) ✅. **Full chain verified:** camera→detect→3D blob→grasp pose `x=1.03 yaw=29.8° score=0.79`. Model: `/home/koen/models/sock_sim.pt` (run `detect_sim model_path_sim:=…`). **Remaining: P5 (real hardware) only.**
**Goal:** From a detected sock, produce a clean 3D point-cloud *blob* (the points belonging to that
sock, in a stable frame) that a later stage can turn into a grasp pose. Grasp-pose generation itself is
**out of scope** here — this plan stops at "one PointCloud2 blob + centroid per sock".

---

## Phase 0 findings (2026-06-07, empirical — sock_arena sim, RTX 3080)

1. ✅ **Sim stereo camera works**: `/stereo_camera/{left,right}/image_raw` + `camera_info` publish at ~8 Hz (gz sensors via `ros_gz_image`).
2. ⛔ **No disparity/pointcloud node runs in sim today.** `stereo_camera_node` hardcodes `CameraFactory::create_camera(JETSON_CSI)` → it is a **Jetson-CSI hardware-capture** node (GStreamer/`VideoCapture`), cannot run in sim. The intended sim variant `point_cloud_node` is referenced by `stereo_camera_sim.launch.py` but **is not built** (perception only builds `stereo_camera_node` + `camera_node`). → the sim launch is broken and **sim has no `DisparityImage`/`PointCloud2`**. *(Corrects the earlier "continuous cloud already exists in sim" assumption.)*
3. ✅ **TF chain exists**: `base_footprint→base_link→chassis→…→camera_link→camera_left_optical_frame`; `base_link → camera_left_optical_frame = [0.105, 0.027, 0.087]`. ⚠️ **rotation is identity** → confirms the known sim **optical-frame orientation quirk** (CLAUDE.md): reprojected points (optical convention) would be mis-oriented once TF'd into `base_link` until the gz camera `<sensor>` frame is fixed.
4. ✅ **Published cloud is unorganized** — confirmed by voxel + statistical + range filters in `stereo_camera_node` (compaction) → crop-by-bbox impossible; **reproject-from-disparity-ROI is the right approach** (decision §1 holds).
5. ⚠️ Detection-alignment unverified: no `sock_sim.pt` model in the workspace → detection produces nothing here; validate in Phase 3 with the model.

**Consequence — the stereo *processing* logic is reusable, only the input source differs.** `stereo_processing_strategy.hpp` (disparity strategy + `generate_pointcloud` + Q-reproject) is hardware-agnostic; `stereo_camera_node` just bolts CSI capture in front of it. So the fix is to feed that same processing from ROS image topics in sim. This adds a **prerequisite phase** (see §5 Phase 1) and unifies sim/real at the `DisparityImage` topic.

---

## 1. Decision: on-demand **action server**, not continuous "full cloud → crop"

**Recommendation: a ROS 2 action server** that, on request, reprojects the detected ROI(s) from the
latest disparity into per-sock point-cloud blobs. **Do not** continuously publish a full dense cloud
just to crop it. Keep a continuous debug topic only as an optional secondary path.

### Why (grounded in this codebase)
| Factor | Continuous full-cloud → crop | **On-demand action (recommended)** |
|---|---|---|
| Published `PointCloud2` is **unorganized** (`pcl::toROSMsg` of a compacted cloud) | crop-by-bbox needs a pixel→point map that no longer exists → must reproject anyway | reproject **only** the bbox ROI from `DisparityImage` (which *is* pixel-registered) — cheap + exact |
| Jetson Orin Nano compute | full-frame reproject + filter every frame, ~all discarded | work happens only when a grasp is wanted |
| Grasping is **intermittent / goal-driven** | topic gives no result/feedback/cancel semantics | action = goal → feedback → result, cancellable, matches `grasp_server`'s action style |
| Determinism | races 3 async topics (image/disparity/detections) + TF | captures one synchronized snapshot → reproducible blob |
| Camera **rides the arm** | continuous TF drift while arm moves | capture-time TF with the arm parked → valid transform |

**Nuance:** stereo disparity is *already* produced continuously (that is the expensive part, and it
already runs in sim and real). Detection already runs continuously on `/detections/socks`. So the new
work is **not** "make a cloud" — it is "reproject the detected region on demand". The action consumes
the existing continuous feeds; it does not re-run stereo.

A `publish_debug_cloud` node param streams the cropped blob on `/socks/points` for RViz tuning
(decision #4): **default `true` in sim** (set in the sim launch), **default `false` on real** until its
per-frame cost is measured on the Orin Nano. It is a debug aid, **not** the grasp-facing contract.

---

## 2. Current state (verified interfaces — reuse, don't rebuild)

- **`jetank_perception` / `stereo_camera_node`** (exec `point_cloud_node` in sim): publishes
  `sensor_msgs/PointCloud2` **and** `stereo_msgs/DisparityImage`; builds the cloud with
  `cv::reprojectImageTo3D(disparity, Q)` inside `stereo_processing_strategy.hpp`
  (`GPUStereoStrategy` / SGBM strategy). Frames param: `frames.left_frame_id=camera_left_link`,
  `base_frame_id=base_link`. Sim runs SGBM (`camera.use_hardware_acceleration:=false`); real runs GPU.
- **`jetank_detection` / `sock_detector_node`** (LifecycleNode): subscribes
  `/stereo_camera/left/image_raw`, publishes `vision_msgs/Detection2DArray` on `/detections/socks`
  (bbox in left-image pixels; header copied from the image → left optical frame). Has both a continuous
  mode and a `DetectSocks` action.
- **Sim** (`jetank_simulation`): gz stereo sensors bridged to `/stereo_camera/{left,right}/image_raw`
  + `camera_info` (~8 Hz observed) via `ros_gz_image`. **Same image topics as real**, BUT **no node
  consumes them into disparity/pointcloud** (see Phase 0 finding #2) — that node is missing.
- **`jetank_manipulation` / `grasp_server`**: open-loop preset grasp (joint targets) via MoveIt
  `/move_action`; empty goal today. Future grasp-pose step will consume this plan's blob.
- **Frames / TF:** `camera_left_optical_frame` exists; the camera (and IMU) **ride the arm**
  (`imu_link←camera_link←S1_link`). Sim camera has a known optical-frame orientation quirk (see
  `CLAUDE.md` Gotchas) — validate orientation early.

---

## 3. Target architecture

**Unify sim and real at the `DisparityImage` (+ `camera_info`) topic.** Two interchangeable input
sources publish it; everything downstream is source-agnostic:

```
 INPUT SOURCE (publishes DisparityImage + camera_info):
   • REAL:  stereo_camera_node   (CSI capture → SGBM/GPU)            [exists]
   • SIM:   stereo_proc_node     (subscribes /stereo_camera/{l,r})   [MISSING — build in Phase 1]
            └─ both reuse stereo_processing_strategy.hpp (Q-reproject)
                              │
 /stereo_camera/left/camera_info ┐
 DisparityImage  ────────────────┤  (continuous)
 /detections/socks (detection) ──┘
                                 │   ACTION GOAL: SegmentSocks
                                 ▼
                    ┌───────────────────────────────┐
                    │  sock_segmentation_server      │  (new; in jetank_perception)
                    │  1. snapshot latest synced set │
                    │  2. per detection bbox:        │
                    │     reproject ROI via Q        │
                    │  3. filter: NaN, z-range,      │
                    │     plane removal, Euclidean    │
                    │     cluster → largest blob     │
                    │  4. select sock nearest base_link, TF → target_frame │
                    └───────────────────────────────┘
                                 │  RESULT: SockCloud {cloud, centroid, dims, label, score}
                                 ▼
                    (future) grasp_pose_node → grasp_server
```

**Package placement (keep modular):**
- **Action definition** → `jetank_detection/action/SegmentSocks.action` (locked, decision #1).
- **Sim stereo source** (`stereo_proc_node`) → `jetank_perception`. Prefer **refactoring
  `stereo_camera_node`** to take an `input_source` param (`csi` | `ros_topics`) so ONE node serves both
  (CSI capture vs `message_filters` image subscribers), with the SAME `stereo_processing_strategy`
  downstream. Avoids a forked second node. Fix `stereo_camera_sim.launch.py` to launch it.
- **Server node** → `jetank_perception` (C++; owns Q/reproject + PCL/OpenCV; reuse
  `stereo_processing_strategy`'s reproject for the ROI).

**Action contract** (`jetank_detection/action/SegmentSocks.action`) — decisions locked:
```
# Goal
string  target_frame     # output frame, default "base_link" (TF at capture time)
float32 min_score        # detection score gate
float32 max_range        # z clip (m)
bool    publish_debug    # also publish /socks/points for RViz (default from node param)
---
# Result
bool       found         # false if no sock passed the gates
SockCloud  sock          # the sock CLOSEST to the robot (min centroid distance to base_link origin)
---
# Feedback
uint16 processed
uint16 total
```
`SockCloud` = `{ sensor_msgs/PointCloud2 cloud, geometry_msgs/PointStamped centroid,
geometry_msgs/Vector3 dimensions, string label, float32 score }`.

**Selection (decision #3):** the server reprojects **all** detections that pass `min_score`/`max_range`,
then returns the **single sock whose centroid is closest to the `base_link` origin** (nearest = most
reachable for grasping). The `SockCloud` array form is intentionally avoided for now; revisit if
multi-sock planning is needed later.

---

## 4. Sim ↔ real unification (the "make it work both" part)

Both paths are **identical** except two knobs, already parameterised:
1. **Disparity backend:** sim = SGBM (CPU), real = GPU CUDA (`camera.use_hardware_acceleration`).
2. **Calibration / `Q`:** sim `camera_info` comes from gz (ideal pinhole); real `Q` from stereo
   calibration. The reproject math is the same; only `Q` differs and it flows from `camera_info`.

Everything downstream of `DisparityImage` (ROI reproject, filter, cluster, TF, action) is
**hardware-agnostic** → write/validate once in sim, run unchanged on real. This is why sim-first works
cleanly here.

---

## 5. Phased implementation (sim-first)

**Phase 0 — Baseline & truth-check (sim). ✅ DONE 2026-06-07.** See "Phase 0 findings" above. Camera
OK; sim disparity/pointcloud node missing; TF chain OK with optical-frame quirk; cloud unorganized.

**Phase 1 — Restore the sim disparity source (NEW prerequisite, sim).** Make `DisparityImage` +
`camera_info` exist in sim. Refactor `stereo_camera_node` to add an `input_source` param
(`csi` default | `ros_topics`); in `ros_topics` mode subscribe `/stereo_camera/{left,right}/image_raw`
+ `camera_info` (`message_filters` ApproximateTime) and feed the existing `stereo_processing_strategy`
(SGBM). Fix `stereo_camera_sim.launch.py` to launch this (it currently names the non-existent
`point_cloud_node`). Also fix the gz camera **optical-frame** orientation (Phase 0 #3) so reprojected
geometry is correct in `base_link`. *Accept:* `DisparityImage` + `PointCloud2` publish in sim;
`PointCloud2` of a sock looks correct in RViz (not rotated 90°).

**Phase 2 — Action interface.** Add `jetank_detection/action/SegmentSocks.action` + `SockCloud.msg`;
build; generate typesupport; stub server returns `found=false`. *Accept:* `ros2 action list` shows it;
goal round-trips.

**Phase 3 — Segmentation server (sim).** Implement in `jetank_perception`:
sync latest (`camera_info`/`Q`, `DisparityImage`, `/detections/socks`) via `message_filters`
ApproximateTime + latest-cache; per detection, slice the bbox from disparity, `reprojectImageTo3D` the
ROI, drop invalid/NaN + out-of-range, optional RANSAC plane removal (arena floor) + Euclidean cluster →
largest cluster; centroid + AABB; **pick the sock nearest `base_link`**; TF to `target_frame`. Default
`publish_debug_cloud:=true` in the sim launch. *Accept:* action returns `found=true` with a `SockCloud`
whose centroid is within a few cm of the true sim sock pose; blob hugs the sock, not the floor.

**Phase 4 — Validation & debug (sim).** Needs `sock_sim.pt` (not in workspace — locate/retrain).
RViz config showing bbox + blob + centroid marker; test multi-sock (nearest selected), partial
occlusion, empty scene (`found=false`). *Accept:* nearest sock chosen correctly; no crash on 0
detections; centroid stable across repeats.

**Phase 5 — Real hardware bring-up.** Same action/server unchanged; real source = `stereo_camera_node`
CSI (GPU strategy) on the same `DisparityImage` topic; verify real `Q`/`camera_info`; **park the arm**
before capture (camera-on-arm) or use capture-time TF; reality-gap pass: textureless socks → sparse
disparity (tune SGBM/PCL filters, min-cluster-size); **profile `/socks/points` cost before enabling on
real** (decision #4). *Accept:* on-robot action returns a sane blob+centroid for a real sock, arm parked.

**Phase 6 — Hooks for grasp poses (interface only).** Document/stub how the future grasp-pose node
consumes `SockCloud` (centroid + principal axis / top-surface → approach pose) and calls `grasp_server`.
No grasp logic implemented here. *Accept:* documented contract + a stub node that logs a candidate pose
from the centroid.

---

## 6. Risks / gotchas
- **Unorganized published cloud** → reproject from disparity ROI, do **not** try to index the published
  cloud by pixel.
- **Sim camera optical-frame quirk** (CLAUDE.md) — **confirmed in Phase 0** (`base_link→optical` rotation
  is identity, not the optical convention) → fix the gz `<sensor>` frame in Phase 1 before trusting
  reprojected geometry in `base_link`.
- **Camera rides the arm** → only trust the blob's stable-frame transform with the arm parked (or use
  capture-time TF and forbid motion during capture).
- **Textureless socks** → stereo gives sparse/holey disparity; budget PCL filtering + cluster tuning in
  Phase 4; consider a slight texture or lighting aid on real.
- **Time sync** across image/disparity/detections → ApproximateTime + bounded staleness; reject stale
  snapshots in the action.
- **Frame in `Detection2DArray`** is the left **optical** frame; ensure ROI↔disparity pixel indexing
  matches (same resolution; handle any rectification offset).

## 7. Decisions (locked 2026-06-07)
1. **Action def home:** `jetank_detection/action/SegmentSocks.action`.
2. **Output frame:** `base_link` (robot base).
3. **Selection:** return the **single sock closest to the robot** (min centroid distance to `base_link`
   origin); array form deferred.
4. **Debug `/socks/points` topic:** param-gated — **on in sim**, **off on real** until per-frame cost is
   profiled on the Orin Nano.

## 8. Phase 7 — Mobile-manip grasp EXECUTION (modular)

**Goal:** actually pick the sock — drive the base into reach, then execute the computed grasp pose with
the arm + gripper. **Scope: full** (base approach + pose-targeted arm grasp + gripper + retreat).

**Feasibility (corrected 2026-06-08):** floor grasp **IS** reachable with `S2≈110°(1.92) / S3≈-15°`
(the old `grasp_server` "can't reach floor" comment was conservative). Arm is **4-DOF** (S1/S2/S3/S5,
chain base_link→S5_link, EE `gripper_ee`/S5_link) → orientation is constrained; treat the grasp as
**position + yaw** (top-down), let MoveIt IK solve. Reach ~0.25 m → base approach required. The camera
sits on its own joint and can tilt down to inspect, but too close degrades stereo — keep a segmentation
standoff; camera-inspection is an **optional** module, not on the critical path.

**Modular components (each independently runnable, composed via action interfaces):**
1. **`grasp_server` upgrade** (jetank_manipulation) — extend `GraspObject.action` with an optional
   `geometry_msgs/PoseStamped target_pose` (+ approach offset). Empty frame ⇒ legacy preset (back-compat);
   set ⇒ pose-targeted grasp via a `/move_action` MoveGroup goal (pre-grasp above → reach pose → close
   gripper → retreat), position+orientation constraints on `gripper_ee`. Floor pose solved by IK.
2. **`base_approach_node`** (NEW, standalone) — `ApproachTarget` action: goal {PointStamped target,
   float32 standoff}, drives `/diff_drive_controller/cmd_vel` (TwistStamped) with a proportional
   rotate-to-face + drive-to-standoff servo; feedback = distance; succeeds within standoff. Reusable for
   any "drive up to a point" need.
3. **`mobile_grasp_coordinator`** (NEW, thin state machine) — orchestrates only:
   SEGMENT (`/segment_socks`) → REACH-CHECK (centroid in arm envelope?) → if not APPROACH
   (`ApproachTarget`) → RE-SEGMENT (sock now near) → GRASP (`GraspObject` with the pose) → DONE/retreat.
   Trigger via a service/action; cancellable.

**Sim validation (incremental):** P7a base_approach drives base to standoff + stops; P7b grasp_server
pose-grasp plans+executes the arm to a reachable floor pose + actuates gripper (needs `moveit_sim`
move_group + gazebo controllers); P7c coordinator sequences end-to-end. Full physical pick success in
Gazebo is best-effort (gripper-sock contact physics); the gate is "each module behaves correctly".

**P7 status (2026-06-08):** all 3 modules **built + unit-tested** (grasp_server pose path; base_approach
+ 10 control tests; coordinator state machine, graceful-fail verified). **P7a base_approach
SIM-VERIFIED** (`success=true`, base stops at standoff; fixed an arrival-tolerance bug). **P7b
(move_group pose-grasp execution) + P7c (full coordinator integration) NOT yet sim-validated** — needs
the heavy bring-up (gazebo + moveit_sim move_group + perception + detector + base + coordinator) and
carries the 4-DOF-IK-to-floor-pose + Gazebo grasp-physics risks. Next step.
**P7 integration run (2026-06-08):** SEGMENT + REACH_CHECK + **APPROACH sim-verified** (base drove 1.03m up to the sock, 'Arrived within standoff 0.18m'; fixed a TF-stamp bug — the odom snapshot must be looked up at latest time, not the frozen detection stamp). **New gap at RE_SEGMENT:** after closing to ~0.2 m the detector loses the sock — the floor sock drops below the forward-looking arm-mounted camera's FOV. Needs a **camera-tilt-down inspection step** before RE_SEGMENT (the 'camera can move down' behaviour). GRASP (move_group) not yet reached. The 8-node bring-up is also flaky in this env (intermittent startup termination).

**P7 final (2026-06-08):** coordinator runs the WHOLE sequence correctly in sim — SEGMENT → **store grasp pose in odom** (1.029,-0.062,0.006) → REACH_CHECK → **APPROACH** (arrived, standoff 0.21m) → **GRASP: recovered pose in base_link (0.209,-0.000,-0.024)** — the remember-in-odom→retrieve open-loop tracking is VERIFIED. The ONLY unvalidated piece is the final `grasp_server`→`move_group` arm motion (4-DOF IK to the floor pose + Gazebo grasp physics): the 8–9-node headless bring-up is flaky in this env (intermittent startup termination), so validate it INTERACTIVELY:
```
ros2 launch jetank_ros_main gazebo_sim.launch.py world:=sock_arena   # +controllers
ros2 launch jetank_moveit_config moveit_sim.launch.py headless:=false use_rviz:=true  # move_group +RViz
ros2 launch jetank_ros_main stereo_camera_sim.launch.py             # disparity/cloud
ros2 launch jetank_detection detect_sim.launch.py model_path_sim:=/home/koen/models/sock_sim.pt confidence:=0.3
ros2 lifecycle set /sock_detector configure && ros2 lifecycle set /sock_detector activate
ros2 run jetank_perception sock_segmentation_server --ros-args -p use_sim_time:=true
ros2 run jetank_manipulation grasp_server --ros-args -p use_sim_time:=true
ros2 run jetank_manipulation base_approach_node --ros-args -p use_sim_time:=true
ros2 run jetank_manipulation mobile_grasp_coordinator --ros-args -p use_sim_time:=true
ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger
```
Watch the arm plan/move in RViz; if IK fails on the floor pose, tune the grasp z / orientation tolerance / `approach_height` (4-DOF reach is tight).

**P7 grasp CONFIRMED (2026-06-08):** the preset arm grasp executes fully in sim (grasp_pre→open→grasp_reach→close[reached_goal]→retreat→home, all move_group SUCCESS). A Cartesian pose grasp at floor level is INFEASIBLE on the 4-DOF arm (wrist self-collides with the arm-mounted camera → OMPL can't sample IK), so `mobile_grasp_coordinator` PICK uses `grasp_mode:=preset` (the RViz-tuned grasp_reach); the base APPROACH centres the sock at the reach standoff. Full single-run chain is gated only by flaky detector lifecycle-startup timing in the headless launch.
