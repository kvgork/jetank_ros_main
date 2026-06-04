# JeTank Sock-Detection Pipeline — Research, Decision & Implementation Plan

**Date:** 2026-05-31 · **Target:** detect **socks lying on the floor** (single class `sock`)
**Hardware:** Jetson Orin Nano Super 8GB (JetPack 6.2, 25 W / 67 TOPS, 1024-core Ampere)
**Stack:** ROS2 Humble inside pixi/RoboStack conda env (no system ROS) · stereo IMX219 via `jetank_perception`
**Scope:** detection only. Grasping / 3D-to-arm is a later task (3D approach sketched, not built here).

---

## 0. TL;DR decision

| Question | Decision | Why |
|---|---|---|
| Live on stream vs on-demand? | **On-demand, lifecycle action server** (`detect_socks` action). Engine loaded once at `on_configure`, GPU-resident; `activate` → run 5–10 frames → return best detection → `deactivate`. A `continuous:=true` param keeps it live for the later "drive-toward-sock" behaviour. | Pick task is discrete (drive → detect → grasp). Saves GPU/power/thermal while navigating. No cold-start after first activate. |
| Model | **YOLO11n**, fine-tuned from COCO weights on captured sock images. TensorRT **FP16** (INT8 later if needed). | ~16–30 ms end-to-end real-world on Orin Nano Super → ≥10 Hz with margin. 2.6 M params, single class needs little capacity. |
| Camera / 3D | **Mono left image for detection; stereo depth ON-DEMAND** (single disparity query at the detected bbox), NOT live stereo. | Live CUDA-SGBM + YOLO + Nav2 contends for 8 SMs / CPU and risks Nav2 timing. On-demand stereo sidesteps the contention question entirely. Live VPI-OFA stereo is a *future* optimisation (see §3). |
| Sim vs real model | **Two separate models** (`sock_sim.*`, `sock_real.*`), selected at runtime by an explicit `sim` flag. The detector node loads `model_path_sim` when `sim:=true`, else `model_path_real`. `detect_sim.launch.py` / `detect_real.launch.py` are the two entry points; `sim_demo` uses the sim one. | Synthetic Gazebo imagery (perfect rectification, synthetic textures/lighting, no sensor noise) is a different domain from real camera frames — one model generalises poorly across both. Decouple from `use_sim_time` so a real rosbag replayed under sim time still picks the real model. See §2a. |
| Compute cost | Detection alone ≈ **5–8 W GPU, 16–30 ms/frame**, fits the 25 W budget with the kit fan while Nav2+SLAM (CPU-bound) run. The risky combo is *live stereo + detect + nav simultaneously* — avoided by the on-demand design. | §3 |
| Biggest risk | **TensorRT is not in the pixi/conda env** (JetPack-only libs). Mitigation = staged integration (§5): PyTorch-in-pixi first, then `--system-site-packages` TRT, then subprocess fallback. | §5 |

---

## 1. Model options (Orin Nano Super, 640×640, TensorRT)

| Model | Precision | Inference (kernel) | End-to-end* | mAP50-95 (COCO) | pixi-friendly? |
|---|---|---|---|---|---|
| **YOLO11n** | FP16 | ~4.6 ms | ~16–30 ms | 39.5 | TRT needs JetPack libs (§5) |
| YOLO11n | INT8 | ~3.8 ms | ~15–28 ms | ~37.5 | + calibration set, on-device |
| YOLOv8n | FP16 | ~26 ms (MDPI) | ~30–40 ms | 37.3 | same |
| YOLO11n | PyTorch (no TRT) | ~14–16 ms | ~30–40 ms | 39.5 | **Yes — runs in pixi** (conda-forge torch) |
| NanoOWL (open-vocab) | FP16 | ~10 ms *(AGX Orin; 2–4× slower on Nano)* | — | — | No (JetPack+torch2trt) |
| jetson-inference detectNet | FP16 | ~10 ms | — | ~22 | No (C++ system install) |
| Isaac ROS yolov8 | — | — | — | — | No (Docker/JetPack only) |

\* End-to-end = decode + resize + infer + NMS + publish. Official ~4.6 ms is **kernel-only post-warmup**; a real ROS node lands ~16–30 ms (Ultralytics issue #22479). Still ≫ the 10 Hz we need.

**Choice: YOLO11n.** Smaller + more accurate + faster than v8n. FP16 default; INT8 only if thermal/power becomes the bottleneck (INT8 needs ~100-image on-device calibration and can lose accuracy on a single custom class with a non-representative calib set).

**Rejected:** NanoOWL (zero-shot is unnecessary for a known object; JetPack-only), detectNet (low mAP, C++ system install), Isaac ROS (Docker/JetPack — incompatible with pixi).

---

## 2. Architecture: live vs on-demand (the core question)

**Continuous publisher** — subscribe to image, infer every frame, publish `Detection2DArray`.
- + Lowest latency once warm, simplest idiom. − Burns ~6–8 W GPU continuously; thermal under sustained 25 W load; contends with anything else on GPU.

**On-demand lifecycle action server** — engine loaded at `on_configure` (GPU-resident), `activate` on a trigger from the task planner, run N frames, return best detection, `deactivate`.
- + Zero GPU burn while driving; clean state machine; no cold-start after first activate (50–200 ms warm-up only on first). − DDS round-trip (~0.5–5 ms intra-host); slightly more code; engine VRAM stays resident even when inactive.

**Decision: on-demand lifecycle action server**, because the JeTank pick task is discrete and Nav2/SLAM are CPU-bound (GPU is free *when not detecting*, but there's no reason to burn it while just driving). Expose `continuous:=true` to flip the same node into a live publisher for the future "see sock while exploring → drive toward it" behaviour — so we don't repaint the architecture later.

Action interface (new `jetank_msgs` or reuse `vision_msgs`):
- Action `DetectSocks`: goal `{timeout, min_confidence, n_frames}` → result `{Detection2DArray best, float32 confidence, bool found}` → feedback `{frames_processed}`.
- Also publish `Detection2DArray` on `/detections/socks` while active, + annotated debug image on `/detections/socks/debug` (param-gated).

---

## 2a. Sim vs real: two models, one node

Detection in Gazebo and on the real robot are **two different visual domains**:
sim frames are perfectly rectified, noise-free, with synthetic textures and
lighting; real IMX219 frames have sensor noise, real floor textures, motion
blur and lens distortion. A model trained only on one transfers poorly to the
other. So the pipeline carries **two models** and selects between them.

**Selection mechanism (implemented):**
- The node takes an explicit `sim` (bool) parameter — **not** derived from
  `use_sim_time`, because the clock source and the visual domain are
  independent (e.g. replaying a real rosbag under sim time must still use the
  real model).
- `sim:=true` → loads `model_path_sim`; `sim:=false` → loads `model_path_real`.
- `model_path` (single explicit path) still exists and **overrides** the
  sim/real selection — handy for one-off testing of a specific file.
- Empty resolved path → node starts, logs a warning, awaits a model (unchanged
  graceful behaviour).

**Two launch entry points** (thin wrappers over `detect.launch.py`):
- `detect_sim.launch.py` — pins `sim:=true`, defaults to continuous (live) mode;
  used by `jetank_ros_main/sim_demo.launch.py`.
- `detect_real.launch.py` — pins `sim:=false`, defaults to on-demand mode
  (the discrete pick task); used on the physical robot.

**Consequence for the dataset/training phases:** P0 and P1 fork into a sim
track and a real track producing `sock_sim.*` and `sock_real.*` respectively
(see §4). The sim model can be trained first and entirely off-robot (cheap
Gazebo captures via `sim_demo world:=sock_arena`), de-risking the ROS/action
plumbing before any real-world data collection.

---

## 3. Compute budget & the stereo question

Baseline (Nav2 + slam_toolbox + stereo camera publish), Orin Nano Super 25 W:

| Resource | Used by baseline | Free for detection |
|---|---|---|
| GPU SM | ~5–15 % (SLAM/Nav2 are CPU-bound) | ~85–95 % |
| GPU mem | ~0.5–1 GB | ~6–7 GB |
| CPU (6 cores) | ~2–3.5 cores | ~2.5–4 cores |
| Mem bandwidth | ~20–30 GB/s | ~70–80 GB/s |
| Power | ~8–12 W | ~13–17 W |

**Detection alone fits comfortably** (~5–8 W GPU, INT8/FP16 YOLO11n). The contention question is *only* about **live stereo**:

| Stereo option | GPU SM | CPU | Stereo FPS | Verdict |
|---|---|---|---|---|
| A. Live VPI **OFA** (off-SM fixed-fn) + live YOLO | YOLO only | ~0.3 core | 20–35 | Best IF VPI-OFA reachable from pixi (doubtful — JetPack path) |
| B. Live **CUDA-SGBM** + live YOLO | 60–80 % (contention) | 1–2 cores | 8–15 | **Avoid** — SM contention degrades YOLO + risks Nav2 timing |
| C. **Mono YOLO + on-demand stereo** (1 disparity query when sock confirmed) | YOLO only | ~0.1 core | n/a | **Chosen** — robust, no contention, depth only when needed |

**Recommendation: Option C now.** Detection runs mono; once a sock is confirmed in N consecutive frames, a single stereo disparity is computed at the bbox to get depth (feeds the later grasp). Re-evaluate **Option A (VPI-OFA)** only if continuous 3D tracking during arm approach is later required — and only after confirming VPI-OFA is usable inside the pixi env (it likely is not without `--system-site-packages`).

**Power mode:** run `nvpmodel -m 0` (25 W MAXN Super) + kit fan. At 15 W YOLO11n FP16 still gives ~100–150 FPS; 25 W is headroom for the concurrent stack and thermal stability (throttle at 80 °C junction).

---

## 4. Custom training path (sock dataset)

> **Two tracks (see §2a).** Run this path twice: a **sim** track (images
> captured from Gazebo `sock_arena`, output `sock_sim.*`) and a **real** track
> (images from the physical robot, output `sock_real.*`). The sim track can be
> done first and entirely off-robot. Keep the two datasets and `sock.yaml`
> files separate; do not mix domains in one training run.

1. **Collect** 400–600 images via the web `/capture` button (already saving to `~/datasets/detection`). Diversity is the whole game: floor textures (tile/wood/carpet/mat), sock colours/patterns, lighting (overhead/side/dim), states (balled/flat/rolled), distances 0.2–1.0 m matching the robot's eye height. Include ~20 % **negatives** (floor, no sock) to cut false positives. Min viable ~200–300 from COCO-pretrained; <200 overfits.
2. **Annotate** in CVAT or Roboflow (single bbox class `sock`); use a rough first model for AI-assisted pre-labelling. Export **Ultralytics YOLO** format. 80/20 train/val + a held-out ~20 % test set never trained on.
3. **Train off-robot** (x86 GPU / Colab T4, ~15–30 min/100 epochs/500 imgs):
   ```bash
   yolo train model=yolo11n.pt data=sock.yaml epochs=100 imgsz=640 batch=16 \
     hsv_s=0.5 degrees=15 flipud=0.3 mosaic=1.0   # avoid heavy perspective (socks are flat)
   ```
4. **Export ONNX off-robot**, transfer, then **build the `.engine` ON the Jetson** (`yolo export format=engine half=True device=0`). TensorRT engines are bound to GPU arch + TRT/CUDA version — a desktop/AGX engine will NOT load on the Orin Nano. INT8: pass the calibration set on device.

---

## 5. The pixi ↔ TensorRT integration risk (staged, de-risked)

The known gotcha (CLAUDE.md): conda-forge lacks JetPack's CUDA/TensorRT/NVMM plugins. Stage the integration so each step is independently shippable:

- **Stage 1 — PyTorch-in-pixi (works today):** `pip install ultralytics` inside pixi → run the `.pt` model on GPU via conda-forge torch. ~30–40 ms/frame. Proves the ROS node, topics, action, and accuracy end-to-end without touching TensorRT. **This unblocks everything else.**
- **Stage 2 — TensorRT via `--system-site-packages`:** recreate the conda env exposing JetPack's `libnvinfer*` (Python must match JetPack's 3.10); load the on-device `.engine`. Gets the 16–30 ms speed.
- **Stage 3 — subprocess fallback (if Stage 2 fights the env):** a tiny detector process using *system* Python + JetPack TRT, publishing `Detection2DArray` back over ROS DDS. Decouples TRT from the pixi env entirely.

Pick the earliest stage that meets the latency target. For ≥10 Hz on a discrete pick task, **Stage 1 may already suffice** — measure before investing in Stage 2/3.

---

## 6. Test plan — numbered checklist (each PASS/FAIL by a command/inspection)

**Model quality (off-robot, `yolo val` on held-out test set):**
1. mAP@0.5 ≥ 0.80 for `sock`.
2. mAP@0.5:0.95 ≥ 0.60.
3. Precision ≥ 0.90 at the operating confidence (false grasps are worse than misses → precision-weighted).
4. Recall ≥ 0.80.
5. Pick operating confidence = argmax F1 on val; record it.

**On-device inference:**
6. `.engine` builds on the Jetson without error (`yolo export format=engine`).
7. `trtexec --loadEngine=sock.engine --iterations=1000 --warmUp=200` → mean kernel latency recorded; p99 < 2× mean.
8. Node end-to-end latency (image header stamp → `/detections/socks` publish) **< 50 ms** — measure with `ros2 topic delay /detections/socks`.
9. `ros2 topic hz /detections/socks` ≥ 10 Hz while active.

**Under full concurrent load (Nav2 + SLAM + stereo + detection, sock_arena world):**
10. Detection FPS degrades < 20 % vs standalone; end-to-end still < 60 ms.
11. Nav2 controller loop stays healthy (no `control loop missed` warnings in the node log) while detecting.
12. Thermal soak 10 min: `jtop`/`tegrastats` junction temp < 80 °C, no throttle in `dmesg`.

**Functional / ROS integration:**
13. `DetectSocks` action returns `found=true` with a plausible bbox when a sock is in the sim/real frame; `found=false` (no crash) on an empty floor.
14. Lifecycle transitions work: `configure` loads engine (logs VRAM), `activate`→`deactivate` leaves no dangling subscriptions (`ros2 node info`).
15. Debug image on `/detections/socks/debug` shows the box on the sock (visual check in RViz/`rqt_image_view`).

**Acceptance for "which architecture" decision:** test 8–11 produce the numbers; if live (continuous) holds tests 10–12 with margin AND a future behaviour needs it, flip `continuous:=true`. Otherwise on-demand is the shipped default.

---

## 7. Implementation plan (phased)

**New package: `jetank_detection`** (ament_python — fastest path for Ultralytics; can port hot path to C++ later). Do **not** bloat `jetank_perception` (keep it camera/stereo only).

| Phase | Deliverable | Files / nodes | Acceptance |
|---|---|---|---|
| **P0. Dataset** | 400–600 labelled images **per track** (`sock_sim`, `sock_real`), each with its own `sock.yaml` + train/val/test split | `~/datasets/detection/{sim,real}/` (+ labels). Sim track off-robot via `sock_arena`; real track on-robot | tests 1–5 reachable |
| **P1. Train + engine** | `sock_sim.pt` + `sock_real.pt` (off-robot) and matching `.engine` files (built on Jetson) | training repo/Colab; export on device | tests 1–7 (run per track) |
| **P2. ROS node (Stage 1, PyTorch)** ✅ | `jetank_detection` pkg: `sock_detector_node` subscribing left image, publishing `Detection2DArray` + debug image; **sim/real model selection** (`sim`, `model_path_sim`, `model_path_real` params, `model_path` override); base `detect.launch.py` + `detect_sim.launch.py` / `detect_real.launch.py` entry points | `jetank_detection/{package.xml,CMakeLists.txt,sock_detector_node.py,backends.py,launch/detect{,_sim,_real}.launch.py,config/sock_detector.yaml}` | tests 8–9, 13, 15 (PyTorch speed) |
| **P3. Lifecycle action server** | wrap node as `LifecycleNode` + `DetectSocks` action (load engine at configure, run-N-frames-on-activate) | `+ action/DetectSocks.action` (in `jetank_msgs` or local), `sock_detector_node.py` | tests 13, 14 |
| **P4. TensorRT (Stage 2/3)** | swap to `.engine` via `--system-site-packages` or subprocess; meet latency target | env recipe in `pixi.toml`/docs; node backend switch | tests 6–11 |
| **P5. Concurrent + thermal validation** | run with Nav2+SLAM in `sock_arena`; record budget numbers; finalise live-vs-ondemand default | sim launch incl. detection | tests 10–12 |
| **P6. Docs** | `jetank_detection/README.md`, root README + CLAUDE.md entry, vault wiki | docs | — |

**3D-for-grasp (LATER, sketch only):** bbox centroid → median disparity in inner 50 % of box → `Z = f·B/d` → backproject via `image_geometry::PinholeCameraModel` → tf2 `camera_optical → base_link`. Average 5–10 frames; reject invalid disparity. Inline in the node (no point-cloud hop) for the single-object case.

### Do-NOT-touch boundaries
`jetank_motor_control`, `jetank_navigation` (Nav2/SLAM configs), `jetank_moveit_config`, `jetank_web_control` nav code. Detection is **additive**: a new package + topics/action only. Camera topics in `jetank_perception` are read-only consumers.

### Sim fixtures & gotchas
- `sock_arena` Gazebo world is the integration fixture (`sim_demo.launch.py world:=sock_arena`).
- Known sim camera ~90° optical-frame offset (jetank_description README) — detection node must use the real `camera_info` / may need a sim remap; sim's perfect rectification makes disparity look better than real — validate on real images before trusting depth.
