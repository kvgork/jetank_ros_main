# Sim→Real Fetch-Sock — Next Steps (on-robot)

Follows the off-robot code work of 2026-06-26 (see `sim2real-hardware-mission.md` +
`docs/hardware-bringup-fetch-sock.md`). Code/launch/guards are built and verified
off-robot (build + 38 bridge unit tests + launch-import). Everything below needs the
**physical JeTank** and is ordered to de-risk before any autonomous motion.

Launch under test: `ros2 launch jetank_mission web_mission_hw.launch.py map:=<map> model_path_real:=<pt>`

---

## Phase 0 — Bench safety + device access (BLOCKING, do first)
- **Wheels off ground, arm clear, hand on power.** First real bringup only.
- Device perms: `/dev/i2c-*` (PCA9685 + IMU), `/dev/gpiochip*`, `/dev/ttyTHS1` (servos),
  `/dev/ttyUSB0` (lidar). Add user to `i2c`/`gpio`/`dialout` (see runbook §2).
- Confirm e-stop / kill path (Ctrl-C, `pkill -f`, power). Publishing zero `/cmd_vel`
  does NOT stop a flooding node.
- **Exit criteria:** every device opens without permission error; kill path rehearsed.

## Phase 1 — Per-subsystem smoke tests (no motion / wheels up)
- Camera: `stereo_camera_node` emits frames inside pixi? (CSI/NVMM gotcha — may need
  host GStreamer outside pixi). Echo `/stereo_camera/left/image_raw`, `disparity`,
  `points` non-empty; frame_id = `*_optical_frame`.
- Lidar `/scan` ~10 Hz; IMU `/imu/data_raw` ~100 Hz (gravity sane); `/odom` present.
- Detector: `model_path_real` set, `/detections/socks` publishes on a real sock.
- **Exit criteria:** all sensor topics healthy; detector fires on a real sock.

## Phase 2 — Actuator checks (wheels still up)
- **Arm:** with `hardware:=serial`, command a small MoveIt move; confirm a servo
  PHYSICALLY moves (catches the mock-backend silent-success trap). `ros2 control
  list_hardware_interfaces` shows the serial system, not mock.
- **Base / cmd_vel arbiter:** `ros2 topic echo /cmd_vel` while (a) web teleop, (b) Nav2
  goal active, (c) APPROACH/PICK — confirm exactly one active source; Twist type; arbiter
  silent when idle. Watch the teleop-during-nav overlap caveat.
- **Exit criteria:** arm moves on command; `/cmd_vel` carries the right source at the
  right time with no idle flood.

## Phase 3 — Calibration (each a standalone effort; values are robot-specific)
- Motor α/β → `config/motor_params.yaml`: straight-line + on-spot-rotate until odom matches.
- Stereo checkerboard → `jetank_perception/config/calibration/{left,right,stereo}_camera.yaml`
  + baseline; verify disparity → metric depth on a known target.
- **Exit criteria:** commanded vs measured motion within tolerance; depth error acceptable.

## Phase 4 — Real perception model
- Collect real-camera sock dataset → train → place `sock_real.pt`; set `model_path_real`.
- **Exit criteria:** detector precision/recall acceptable on held-out real frames.

## Phase 5 — Navigation tuning (wheels down, open floor, supervised)
- Map the arena (SLAM), save map. Tune AMCL + nav2 for drifting odom + noisy real scan
  (the sim params are a starting point only).
- **Exit criteria:** robot localizes + reaches nav goals reliably on the real floor.

## Phase 6 — End-to-end fetch (supervised, hand on kill)
- Full `web_mission_hw.launch.py`; drive a fetch from the browser. Tune the launcher
  TimerAction periods to real startup times (camera/lidar/AMCL spin-up, model load).
  Watch for the +30/+40/+55/+60s stagger being too tight on the Jetson.
- **Exit criteria:** NAVIGATE→SEARCH→PICK→DEPOSIT completes on hardware.

## Phase 7 — Soak / robustness
- Thermal soak under full stack (`tegrastats`); repeated fetches; failure recovery.
- Carry over the open sim issue: concurrent-mission overlap (reject-when-busy) —
  see `jetank_mission/plans/single-mission-guard-plan.md`.

---

## Risk register (from off-robot review)
- Launcher stagger timing is a greenfield estimate — first real bringup will need tuning (Phase 6).
- CSI/NVMM camera may not deliver frames inside pixi (Phase 1 gate).
- Teleop during active Nav2 = brief two-publisher overlap on `/cmd_vel` (documented residual).
- `sock_real.pt` does not exist yet — mission no-ops until Phase 4 done.
