# jetank_ros_main

Integration / seed package for the JeTank workspace. Owns the top-level launch
files, Gazebo worlds, RViz configs, and `config/motor_params.yaml`, and
`<depend>`s on all sibling packages so they build together. Also carries
`install.sh` + `workspace_template/` for one-clone bootstrap.

## Simulation bring-up

### `sim_demo.launch.py` — the one-command sim (Gazebo GUI + RViz + optional SLAM)

```bash
ros2 launch jetank_ros_main sim_demo.launch.py
# args:
#   world := empty | simple_test | obstacle_course | sock_arena   (default: obstacle_course)
#   slam  := true | false   (default true — slam_toolbox builds /map from the sim lidar)
#   rviz  := true | false   (default true — loads jetank_ros_main/rviz/unified.rviz)
#   arm   := true | false   (default false — also starts MoveIt move_group and
#                            activates arm_controller, attached to THIS Gazebo)
```

`arm:=true` is the truly-unified path: it starts `arm_controller` active (via
`gazebo_sim`'s `start_arm_active`) and includes `moveit_sim.launch.py` with
`start_gazebo:=false` so move_group attaches to the running sim instead of
spawning a second one.

```bash
# Everything in one sim: base + lidar + SLAM + arm + RViz
ros2 launch jetank_ros_main sim_demo.launch.py arm:=true
```

Includes `gazebo_sim.launch.py` (Gazebo + robot + sensors + controllers),
`rviz.launch.py` (unified.rviz, `use_sim_time:=true`), and — when `slam:=true` —
`jetank_navigation/navigation_full.launch.py mode:=slam use_sim_time:=true` with
its own RViz suppressed.

> GUI windows open on your X display but may appear **behind** other windows.

### `gazebo_sim.launch.py` — Gazebo + robot only

```bash
ros2 launch jetank_ros_main gazebo_sim.launch.py world:=obstacle_course
```

Selects a world by name and includes `jetank_simulation/gazebo.launch.py`.

### Drive the base (controller takes **TwistStamped**)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -r /cmd_vel:=/diff_drive_controller/cmd_vel
```

`/diff_drive_controller/odom` publishes odometry (`odom → base_footprint`,
`enable_odom_tf: true`).

## Worlds (`worlds/`)

| World | Contents | Use |
|---|---|---|
| `empty_fortress.sdf` (in jetank_simulation) | ground + sun | baseline |
| `simple_test.sdf` | box + cylinder | quick lidar/depth check |
| `obstacle_course.sdf` | 5×5 m arena, walls + 8 cylinders | **lidar / nav / SLAM** |
| `sock_arena.sdf` | room + furniture + socks | manipulation/collection |

## RViz

`rviz/unified.rviz` — RobotModel, TF, LaserScan, Map, camera images, PointCloud2
and the Navigation 2 panel; fixed frame `base_link`. Launch standalone:

```bash
ros2 launch jetank_ros_main rviz.launch.py use_sim_time:=true
```

## Other launch files

`main.launch.py`, `unified.launch.py` (real-robot bringup), `urdf.launch.py`,
`motor_controller.launch.py`, `stereo_camera*.launch.py`, `rviz.launch.py`.

## Notes / fixed bugs

- `gazebo_sim.launch.py` previously raised `UnboundLocalError` (a function-local
  `LaunchConfiguration` import shadowed the module import) — fixed; do not
  re-add that import.
