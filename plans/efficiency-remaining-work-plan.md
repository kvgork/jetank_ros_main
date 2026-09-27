# Efficiency Audit — Remaining Work Plan

Created 2026-09-27, after the apply run (74 commits merged and pushed in 9 repos).
Source documents in this folder: `efficiency-audit-2026-09-26.md` (full report, incl. post-fix
addendum and measurement correction), `efficiency-audit-2026-09-26-digest.md`,
`efficiency-audit-2026-09-26-index.md` (all 359 findings), `efficiency-fixes-2026-09-27-plan.md`
(apply-run log).

Everything left needs someone at the robot. Work through the phases in order; each has a
stop condition so a hardware fault does not turn into a code change made blind.

## Phase 1 — Bring the Feetech servo bus back (blocks Phase 2)

**Symptom:** `ros2_control_node` with `hardware:=serial` exits at activate with
`Servo id 1 (S1_joint) did not respond to ping` on `/dev/ttyTHS1` at 1 Mbps (seen 2026-09-26 and
2026-09-27). Drive motors, IMU, lidar and camera are fine.

Steps:
1. Check servo power: the servo rail is separate from the Jetson supply; confirm it is switched
   on and measure its voltage at the bus connector.
2. Check the half-duplex wiring from the Jetson UART (pins for `/dev/ttyTHS1`) to the servo
   adapter board, and that nothing else holds the port: `sudo fuser /dev/ttyTHS1`.
3. Confirm the servo family and protocol. Memory from 2026-09-12 says these are SCS15 (10-bit,
   big-endian), not STS. Check that `feetech_bus.cpp` and the ID/baud assumptions match.
4. Fix the `hardware-test` MCP ping tool first so the bus can be probed without launching
   ros2_control: it fails with `ImportError: cannot import name 'sms_sts' from 'scservo_sdk'`
   (harness tracker item 26). Use the SCS protocol class.
5. Ping IDs 1–5 at 1 Mbps; if silent, sweep 500k/115200 once to catch a reset servo.

**Done when:** `unified.launch.py hardware:=serial enable_moveit:=true` activates
`arm_controller`, `gripper_controller` and `joint_state_broadcaster`.
**Stop if:** no servo answers after power and wiring are confirmed. That is a hardware
repair, not a software task.

## Phase 2 — The deferred serial-driver findings (needs Phase 1)

Findings `jetank_motor_control-16/17/18`, in `src/hardware/jetank_serial_hardware.cpp` and
`src/hardware/feetech_bus.cpp`. The line anchors in the report are stale; re-anchor them first.

1. **Baseline:** with the arm idle on the live bus, record the controller loop period and its
   jitter, the `read()`/`write()` duration per cycle, and the bus error count.
2. **motor_control-17:** skip re-sending unchanged goal positions each cycle, including S5.
   Lowest risk; do it first.
3. **motor_control-18:** shorten the 12 ms reply timeout toward the real frame time at 1 Mbps.
   Keep a margin, and test with one servo unplugged so a silent servo cannot overrun the 20 ms
   cycle.
4. **motor_control-16:** reduce the blocking transactions per cycle, for example with sync
   read/write if SCS15 supports it.
5. The `expect_write_replies:=false` option from the July audit is only safe after the servos'
   status-return (SRM) register is reconfigured. Do not combine the two changes.

**Done when:** each change is measured against the baseline, and a MoveIt pick on the real arm
still succeeds. One commit per finding, as before.

## Phase 3 — Track width (finding cross-07)

The simulation and URDF say 0.14 m wheel separation (`jetank_controllers.yaml:123`, `wheels.xacro`).
The hardware odometry says 0.11 m (`motor_params.yaml`, now in `jetank_motor_control/config/`).

1. Measure the real separation between the tread centre lines with a ruler.
2. Check it by driving: command a 360° spin in place and compare the odometry yaw with the real
   rotation. The value that makes odometry match is the effective track width, which differs
   from the ruler value on a tracked robot because the tracks slip.
3. Put the effective value in one place, and have the URDF/sim and hardware configs reference
   it through launch substitution. The verifier noted that YAML includes will not work here.

**Done when:** the spin test gives odometry yaw within 5% of the real rotation, and the sim and
hardware values match or have a documented reason for differing.

## Phase 4 — Runtime validation of fixes applied blind

These are merged but were only checked by build and lint:

- **Detection** (`detection-01/02/04/05`: busy-poll, rate limit, warm-up, FP16/imgsz). Needs
  `pixi install -e detect` (ultralytics) and a real model at `~/models/sock_real.pt`. Measure the
  detector's CPU/GPU load and the first-goal latency before and after warm-up.
- **Simulation** (`simulation-09/14/19`: spawner sequencing, physics step, controller deps). On
  the linux-64 workstation run `pixi run -e dev gazebo` and confirm the controllers spawn in order
  and the real-time factor improved.
- **Hardware launch chain:** run `pixi run slam` on the robot and confirm IMU and lidar start. This
  covers the `use_sim_time` case fix in the new navigation delegation.
- **July-audit leftovers still unchecked on hardware:** spot-check the IMU `imu/data_raw` axes
  (the MPU-9250 to ICM-20948 register fix) and servo `poll()` behaviour, which needs Phase 1.

## Measurement rule (learned 2026-09-27)

For any topic above about 500 KB per message, measure rate and latency with
`jetank_perception/scripts/topic_rate_probe.py`, not with the ros2-mcp tools or `ros2 topic hz`.
Those reported disparity at 0.6 Hz when it was really 30 Hz. CPU numbers must come from a profile
that actually overlaps a live consumer.

## Optional minors left from review rounds

- Tests that pin the BGR8 pipeline template (no `videoconvert`) and the cold-buffer TF lookup path.
- A regression gtest for the Q(3,2) sign.
- Drop the redundant double `can_transform` wait in base_approach and the coordinator.
- `jetank_simulation` should `exec_depend` on `jetank_motor_control`.
- SRDF `virtual_joint` type `fixed` vs `planar` (planning-scene objects in `odom` after the base drives).
- Hoist the duplicated detector-start delay constants in the mobile_grasp launches.
