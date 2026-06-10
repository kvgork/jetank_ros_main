# JeTank — Simulation Control Cheatsheet

How to launch the Gazebo simulation and drive every part of the robot.
All commands run inside the pixi env (`pixi shell`, or prefix with `pixi run`).

## Launch

```bash
# Full sim: Gazebo GUI + RViz (unified.rviz) + arm (move_group) + web control
ros2 launch jetank_ros_main sim_demo.launch.py arm:=true web:=true

# Args (all optional):
#   world := empty | simple_test | obstacle_course | sock_arena | house   (default house)
#   slam  := true | false   (default true)   rviz := true | false   (default true)
#   arm   := true | false   (default false)  web  := true | false   (default false)
```

> GUI windows (Gazebo, RViz) open **behind** other windows — alt-tab to them.
> RViz uses the **Orbit** view (LMB orbit, scroll zoom, Shift+LMB pan).

### Fetch-sock mission (one command)

The full **navigate → search → pick → carry → deposit** loop, driven from the browser:

```bash
ros2 launch jetank_mission web_mission.launch.py \
  map:=$HOME/maps/sock_arena.yaml gui:=true use_rviz:=true
# Args: world (sock_arena) · map (~/maps/sock_arena.yaml) · model_path_sim
#       · gui (false) · use_rviz (false)   — headless by default, browser-driven
```

Open **http://localhost:8080**, set a deposit area (once), then *Fetch sock* mode → click the
pick site. FSM: `NAVIGATE_TO_SITE → SEARCH → PICK → NAVIGATE_TO_DEPOSIT → DEPOSIT`.
Full architecture: `jetank_mission/docs/fetch-sock-mission.md`.

### Pick stack only (drive a grasp interactively)

```bash
ros2 launch jetank_ros_main mobile_grasp.launch.py gui:=true use_rviz:=true
ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger
```

> `gazebo_sim.launch.py` / `mobile_grasp.launch.py` take `gui:=true|false` (Gazebo GUI vs
> server-only) and `start_arm_active:=true|false`. `mobile_grasp` passes
> `start_arm_active:=true` so MoveIt's trajectories aren't rejected by an inactive `arm_controller`.

## Drive the base

Web UI (easiest): open **http://localhost:8080** (or `http://<host-ip>:8080` from
another device) → WASD / arrows / on-screen joystick.

Keyboard teleop (note: the controller takes **TwistStamped**):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -r /cmd_vel:=/diff_drive_controller/cmd_vel
```

Odometry: `/diff_drive_controller/odom`.

## Move the arm (4-DOF: S1, S2, S3, S5)

Send a joint trajectory to the `arm_controller` (requires `arm:=true`):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [S1_joint, S2_joint, S3_joint, S5_joint],
     points: [{positions: [0.0, 0.5, -0.3, 0.2], time_from_start: {sec: 2}}]}}"
```

Joint limits (rad): S1 ±2.35, S2/S3/S5 ±1.57.

## Move the gripper (parallel jaw)

`gripper_controller` is a `JointGroupPositionController`. Position in meters,
`0.0` = closed … `0.04` = open:

```bash
# open
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.04, 0.04]}"
# close
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

The arm and gripper motion is visible in both RViz (RobotModel) and Gazebo.

## RViz — known limitation

The MoveIt **MotionPlanning** display is intentionally **not** in `unified.rviz`:
the `motion_planning_rviz_plugin` **segfaults RViz** in this RoboStack/Gazebo
environment (SIGSEGV on load). Drive the arm with the action command above
instead of dragging a goal marker. `move_group` still runs (planning available
programmatically / via MoveIt's Python or C++ API).

## Teardown

```bash
pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f rviz2; pkill -f move_group; pkill -f web_control
```
