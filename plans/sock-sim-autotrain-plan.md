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
| Capture method | **Web control `/capture` button** against `sim_demo world:=sock_arena web:=true` (the web UI runs in sim with `sim:=true`) | Frames go through the *same* camera path the detector sees at runtime → train/serve consistency. Headless `capture_frames` remains a batch fallback. |
| Scene diversity | `sock_arena` already has **6 coloured socks** planted (red/blue/green/yellow/white/pink). Drive the robot around to vary viewpoint; later add domain randomization (pose/floor/light) | Static spread already gives viewpoint diversity; randomization is a later upgrade, not a blocker. |
| Labelling | **Annotation tab in the web UI** with an *Auto (rough)* toggle: a CV colour-blob detector proposes boxes, human reviews/corrects/saves | No model exists yet (chicken-and-egg) and COCO has no `sock` class, so CV blob is the only viable "auto" today. GT-projection (Path A) stays the long-term upgrade. |
| Train | Ultralytics `yolo train` off-robot (x86 GPU), recipe from parent §4.3 | Same as real track; sim data only. |
| Output | `sock_sim.pt` (+ ONNX → `.engine` built on Jetson) | Wired via `model_path_sim` in `config/sock_detector.yaml`. |
| Acceptance | Parent §6 tests 1–9, 13, 15 run on a **sim** held-out set | Reuse the existing numbered test plan, sim domain. |

---

## 1. Phases

### S0 — Capture via web control (DONE — tooling shipped)
- `sock_arena` already has 6 coloured socks planted, and `sim_demo` runs the web
  UI in sim mode (`web_control.launch.py sim:=true`), so captured frames use the
  same camera path the detector sees.
  ```bash
  ros2 launch jetank_ros_main sim_demo.launch.py world:=sock_arena web:=true slam:=false
  # open http://localhost:8080 → drive the robot → click 📷 Capture per frame
  # frames land in ~/datasets/detection/ (web_control capture_dir)
  ```
- Drive to vary viewpoint/distance; include ~20% empty-floor negatives.
- **Later upgrade:** domain randomization (respawn socks at random pose, swap
  floor material, jitter light between runs) via a `gz service` / ROS node.
- **Acceptance:** ≥400 sim JPEGs in the capture dir, diversity spot-checked.

### S1 — Annotation tab with rough auto-label (DONE — tooling shipped)
- The web UI has an **Annotate tab** (`showTab('annotate')`): pick a capture,
  draw/edit boxes, save YOLO sidecars. An **Auto (rough)** toggle + *Auto-detect*
  button call `POST /captures/autolabel/{name}`, which runs a CV colour-blob
  detector (`rough_boxes_from_bgr`, HSV saturation/value threshold + contours)
  and proposes boxes for the human to correct. Boxes map to the `sock` class.
  - Limitation: misses low-saturation socks (the white one) — draw those by hand.
- **Path A (long-term upgrade) — ground-truth projection:** log the sock world
  pose + `camera_info`/TF at capture and project an exact bbox → zero manual
  clicks. Replaces the CV rough pass once the sim optical-frame offset is handled
  (parent §sim-gotchas).
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
