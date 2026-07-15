# Sim-to-Real Implementation Plan — Mobile Sock-Grasp

Companion to [`sim2real-gap-analysis.md`](./sim2real-gap-analysis.md). This plan
covers the **software-implementable** subset of the sim→hardware work — what can
be written and compile-verified **without** the physical robot. Hardware/data-gated
items (training `sock_real.pt`, per-servo calibration, on-robot tuning, device
permissions) are explicitly **out of scope** here and remain owner tasks on the robot.

Branch: `feature/sim2real-hardware-bringup` (in `jetank_motor_control`,
`jetank_description`, `jetank_ros_main`).

## Scope decision

| In scope (this plan) | Out of scope (robot/data needed) |
|---|---|
| `JetankSerialHardware` C++ driver (compile-verified) | Per-servo zero/direction calibration values |
| `hardware` xacro arg + mock branch (safety fix) | Training `sock_real.pt`, dataset capture |
| Open-loop `odom→base_footprint` | Encoder-based odometry |
| Hardware pick launch + cmd_vel wiring | On-robot grasp/depth tuning, stagger timers |
| Build + xacro parse verification | Runtime servo I/O verification |

## Phases

### P1 — `JetankSerialHardware` ros2_control SystemInterface  *(blocker #1)*
- `feetech_bus.{hpp,cpp}` — minimal Feetech ST/SC serial protocol over POSIX
  termios (open `/dev/ttyTHS1` @ 1 Mbps, write goal position, read present
  position, torque enable/disable, ping). No external SDK dependency.
- `jetank_serial_hardware.{hpp,cpp}` — `hardware_interface::SystemInterface`.
  Generic per-(joint,interface) storage like `GenericSystem`; joints with a
  `servo_id` param do real serial I/O, joints without (gripper_right, wheels)
  mirror command→state. rad↔tick via per-joint `offset`/`direction`/`ticks_per_rad`
  params (defaults: center 2048, dir +1, 4096 ticks/rev). Velocity state by
  finite difference. `PLUGINLIB_EXPORT_CLASS`.
- **Accept:** compiles; plugin discoverable; `hardware:=serial` xacro selects it.

### P2 — `hardware` xacro arg + mock branch  *(safety hazard)*
- `jetank_ros2_control.urdf.xacro`: add `<xacro:arg name="hardware" default="serial"/>`,
  thread into macro.
- `config/ros2_control.xacro`: macro param `hardware`; plugin = IgnitionSystem
  (if `use_sim`) | `mock_components/GenericSystem` (if `hardware==mock`) | serial.
- **Accept:** `hardware:=mock` → GenericSystem (no real plugin); `:=serial` →
  JetankSerialHardware; `use_sim:=true` → IgnitionSystem. All three xacro-parse.

### P3 — Open-loop odometry in `robot_controller.cpp`  *(blocker #2, fast path)*
- Publish `nav_msgs/Odometry` on `/odom` + TF `odom→base_footprint`, integrated
  from commanded (clamped) `lin_vel`/`ang_vel`. Zeroed on safety-stop.
  Param-gated (`publish_odom`, default true).
- **Accept:** compiles; provides the `odom→base_link` chain the approach servo +
  grasp recovery require. (Open-loop drift is expected until encoders exist.)

### P4 — Hardware pick launch + cmd_vel wiring  *(blockers #3, #5)*
- `jetank_ros_main/launch/mobile_grasp_hw.launch.py`: `unified hardware:=serial
  enable_moveit:=true` + `detect_real.launch.py` + the 4 pipeline nodes
  (`sock_segmentation_server`, `grasp_server`, `base_approach_node`,
  `mobile_grasp_coordinator`) with `use_sim_time:=false`; `base_approach_node`
  `cmd_vel_topic:=/cmd_vel_manip`; `cmd_vel_bridge` (`output_stamped:=false`,
  `output_topic:=/cmd_vel`, `manip_topic:=/cmd_vel_manip`, `nav_topic:=''`);
  lifecycle configure/activate for `/sock_detector`.
- **Accept:** launch imports/parses; base motion path now reaches `/cmd_vel`.

### P5 — Build + verify
- `pixi run build` (motor_control, description, ros_main); xacro parse all 3
  modes; existing tests green.

## Known residual risks (compile-only)
- Feetech register map / endianness assume the SMS/STS family (0–4095, LE).
  **Must be confirmed against the actual servos** before enabling torque — the
  `hardware-test` MCP `feetech_servo_*` tools are the verification path.
- `sock_real.pt`, calibration, and CM 2.54 `joints empty` fix remain open
  (latter tracked separately; affects the standalone controller_manager path).
