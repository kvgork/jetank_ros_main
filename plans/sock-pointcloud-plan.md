# Plan — Sock 3D point-cloud blob (toward grasp poses)

**Status:** plan / not started · **Created:** 2026-06-07 · **Mode:** sim-first, sim + real
**Goal:** From a detected sock, produce a clean 3D point-cloud *blob* (the points belonging to that
sock, in a stable frame) that a later stage can turn into a grasp pose. Grasp-pose generation itself is
**out of scope** here — this plan stops at "one PointCloud2 blob + centroid per sock".

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
  + `camera_info` at ~30 Hz via `ros_gz_image`. **Same topics as real** → one code path.
- **`jetank_manipulation` / `grasp_server`**: open-loop preset grasp (joint targets) via MoveIt
  `/move_action`; empty goal today. Future grasp-pose step will consume this plan's blob.
- **Frames / TF:** `camera_left_optical_frame` exists; the camera (and IMU) **ride the arm**
  (`imu_link←camera_link←S1_link`). Sim camera has a known optical-frame orientation quirk (see
  `CLAUDE.md` Gotchas) — validate orientation early.

---

## 3. Target architecture

```
/stereo_camera/left/image_raw ─┐
/stereo_camera/left/camera_info ┤ (continuous, sim+real)
DisparityImage (perception)  ───┤
/detections/socks (detection) ──┘
                                 │   ACTION GOAL: SegmentSocks
                                 ▼
                    ┌───────────────────────────────┐
                    │  sock_segmentation_server      │  (new; lives in jetank_perception)
                    │  1. snapshot latest synced set │
                    │  2. per detection bbox:        │
                    │     reproject ROI via Q        │
                    │  3. filter: NaN, z-range,      │
                    │     plane removal, Euclidean    │
                    │     cluster → largest blob     │
                    │  4. TF blob → stable frame      │
                    └───────────────────────────────┘
                                 │  RESULT: SockCloud[] {cloud, centroid, label, score}
                                 ▼
                    (future) grasp_pose_node → grasp_server
```

**Package placement (keep modular):**
- **Action definition** → `jetank_detection` (owns detection-domain msgs) *or* a new tiny
  `jetank_msgs` pkg if shared more widely. Default: `jetank_detection/action/SegmentSocks.action`.
- **Server node** → `jetank_perception` (C++; it already owns Q/reproject + PCL/OpenCV deps and the
  strategy code to reuse). Reuse `stereo_processing_strategy`'s reproject helper for the ROI.

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

**Phase 0 — Baseline & truth-check (sim).** Bring up `sim_demo` + `sock_arena`; confirm
`/stereo_camera/left/image_raw`, `DisparityImage`, `PointCloud2`, and `/detections/socks` all flow and
align in RViz. Confirm/measure: is the published cloud unorganized? what frame does disparity carry?
does the sim camera optical-frame quirk distort geometry? **Output:** a short findings note + go/no-go
on reprojection approach. *Accept:* sock visible in RViz cloud; bbox overlaps the sock in the left image.

**Phase 1 — Action interface.** Add `SegmentSocks.action` + `SockCloud.msg`; build; generate
typesupport; stub server that returns empty result. *Accept:* `ros2 action list` shows it; goal
round-trips.

**Phase 2 — Segmentation server (sim).** Implement in `jetank_perception`:
sync latest (left image, `camera_info`/`Q`, `DisparityImage`, `/detections/socks`) via
`message_filters` ApproximateTime + a latest-cache; per detection, slice the bbox from disparity,
`reprojectImageTo3D` the ROI, drop invalid/NaN and out-of-range points, optional RANSAC plane removal
(arena floor) + Euclidean clustering → keep largest cluster; compute centroid + AABB; TF to
`target_frame`. *Accept:* action returns ≥1 `SockCloud` for a sock in view; centroid within a few cm of
the true sock pose in the sim world; blob hugs the sock, not the floor.

**Phase 3 — Validation & debug (sim).** RViz config showing bbox + blob + centroid marker; test multi-
sock, partial occlusion, empty scene (graceful empty result); optional `/socks/points` debug topic.
*Accept:* correct blob count vs scene; no crash on 0 detections; centroid stable across repeats.

**Phase 4 — Real hardware bring-up.** Run the **same** node with GPU strategy + real stereo
calibration; verify `Q`/`camera_info`; **park the arm** before capture (camera-on-arm) or rely on
capture-time TF; reality-gap pass: textureless socks → sparse disparity (tune SGBM/PCL filters, fill,
min-cluster-size); confirm blob in `base_link`. *Accept:* on-robot action returns a sane blob+centroid
for a real sock with the arm parked.

**Phase 5 — Hooks for grasp poses (interface only).** Document/stub how the future grasp-pose node
consumes `SockCloud` (centroid + principal axis / top-surface → approach pose) and calls `grasp_server`.
No grasp logic implemented here. *Accept:* documented contract + a stub node that logs a candidate pose
from the centroid.

---

## 6. Risks / gotchas
- **Unorganized published cloud** → reproject from disparity ROI, do **not** try to index the published
  cloud by pixel.
- **Sim camera optical-frame quirk** (CLAUDE.md) → validate blob orientation in Phase 0; fix the gz
  `<sensor>` frame if geometry is rotated.
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
