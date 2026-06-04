# Sim Sock-Model Auto-Train Plan (`sock_sim.*`)

**Date:** 2026-06-04 · **Parent:** [`sock-detection-plan.md`](./sock-detection-plan.md) (P0-sim / P1-sim)
**Scope:** produce a working **sim** detector model `sock_sim.pt` (+ `.engine` on
Jetson) trained on Gazebo `sock_arena` imagery, with as much of the loop
automated as practical. This is the deferred "Option 2" split out of the
sim-integration task — the code/capture tooling lands separately; this plan is
the data→model loop.

> **Why separate:** training needs a running Gazebo + a GPU + an annotation
> step. Annotation quality gates model quality, and good labels are not fully
> automatable. This plan maximises automation while keeping a human in the
> labelling loop where it matters.

---

## 0. Decision summary

| Question | Decision | Why |
|---|---|---|
| Capture method | Headless `jetank_detection capture_frames` node against `sim_demo world:=sock_arena` | No human clicking; deterministic interval; resumes numbering. Already built. |
| Scene diversity | **Domain randomization in `sock_arena`** — randomize sock pose/colour, floor texture, light, camera height between capture runs | Single static scene overfits; sim lets us randomize cheaply (the whole point of the sim track). |
| Labelling | **Auto-pre-label from sim ground truth** where possible; else COCO-pretrained rough pass + human review | Gazebo knows the sock pose → can project a bbox without manual clicks (best case). Fall back to assisted labelling. |
| Train | Ultralytics `yolo train` off-robot (x86 GPU), recipe from parent §4.3 | Same as real track; sim data only. |
| Output | `sock_sim.pt` (+ ONNX → `.engine` built on Jetson) | Wired via `model_path_sim` in `config/sock_detector.yaml`. |
| Acceptance | Parent §6 tests 1–9, 13, 15 run on a **sim** held-out set | Reuse the existing numbered test plan, sim domain. |

---

## 1. Phases

### S0 — Domain-randomized capture (automatable)
- Extend `sock_arena` (in `jetank_simulation` worlds) with a randomization hook:
  a small script that, between capture runs, respawns the sock at random
  `(x, y, yaw)` on the floor plane, swaps the floor material, and jitters the
  light. Options: an Ignition `gz service`/`gz model` script, or a ROS node that
  calls `/world/.../set_pose` and material services.
- Drive captures with the existing node:
  ```bash
  ros2 launch jetank_ros_main sim_demo.launch.py world:=sock_arena slam:=false
  ros2 run jetank_detection capture_frames --ros-args \
      -p output_dir:=$HOME/datasets/detection/sim \
      -p domain:=sim -p interval_sec:=0.5 -p max_frames:=600
  ```
- Target: 400–600 frames across ≥4 floor textures, ≥3 light levels, sock
  distances 0.2–1.0 m, including ~20% empty-floor negatives.
- **Acceptance:** ≥400 sim JPEGs in `~/datasets/detection/sim/`, diversity
  spot-checked.

### S1 — Auto-labelling (semi-automatable)
- **Path A (preferred) — ground-truth projection:** at capture time, also log
  the sock's world pose + the camera `camera_info`/TF, project the sock's
  bounding box into image space, and write the YOLO `.txt` label directly. Zero
  manual clicks. Requires extending `capture_frames` (or a sibling node) to read
  the model pose and project — design it in S0 so labels come free with frames.
- **Path B (fallback) — assisted:** run a COCO-pretrained YOLO11n to pre-label,
  then human-review in CVAT/Roboflow (parent §4.2). Use when Path A projection
  is too noisy (sim camera ~90° optical-frame offset, see parent §sim-gotchas).
- 80/20 train/val + held-out ~20% test split, single class `sock`.
- **Acceptance:** every train/val image has a label file; test set never used in
  training; `sock.yaml` (sim) written.

### S2 — Train + export (automatable, off-robot)
- ```bash
  yolo train model=yolo11n.pt data=sock_sim.yaml epochs=100 imgsz=640 batch=16 \
      hsv_s=0.5 degrees=15 flipud=0.3 mosaic=1.0
  yolo export model=runs/.../best.pt format=onnx     # off-robot
  # On the Jetson:
  yolo export model=sock_sim.pt format=engine half=True device=0
  ```
- Place `sock_sim.pt` where `model_path_sim` points (config/sock_detector.yaml).
- **Acceptance:** parent §6 tests 1–5 (model quality on sim test set), 6–7
  (.engine builds on Jetson).

### S3 — In-sim validation
- ```bash
  ros2 launch jetank_ros_main sim_demo.launch.py world:=sock_arena detect:=true \
      slam:=false
  ros2 lifecycle set /sock_detector configure
  ros2 lifecycle set /sock_detector activate
  ros2 topic echo /detections/socks
  ```
- **Acceptance:** parent §6 tests 8–9 (latency/Hz), 13 (action found/​not-found),
  15 (debug image shows box) — all on the sim domain.

### S4 — Concurrent + thermal (sim, parent P5)
- Run with Nav2 + SLAM in `sock_arena`; record the budget numbers; finalise the
  live-vs-on-demand default. Parent §6 tests 10–12.

---

## 2. Automation ceiling (where a human is required)

| Step | Automatable? | Human needed |
|---|---|---|
| Capture frames | ✅ full | none (start/stop) |
| Domain randomization | ✅ full | author the randomizer once |
| Labelling via GT projection (Path A) | ✅ if projection validated | validate projection accuracy once |
| Labelling assisted (Path B) | ⚠️ partial | review/correct boxes |
| Train/export | ✅ full | none |
| Acceptance judgement | ⚠️ partial | eyeball debug images, sign off mAP |

**Recommendation:** invest in S1 Path A (GT projection) — it is the single
change that turns this from a human-in-loop chore into a one-command sim model
factory. If projection proves too noisy given the sim optical-frame offset, fall
back to Path B for the first model, then revisit projection.

---

## 3. Do-NOT-touch boundaries
Same as parent: detection is additive. The randomizer touches only
`jetank_simulation` world/launch assets; the label projector lives in
`jetank_detection`. No edits to Nav2/SLAM/motor/moveit configs.
