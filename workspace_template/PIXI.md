# Pixi workflow

This workspace is managed with [pixi](https://pixi.sh) using the
[RoboStack](https://robostack.github.io/) channel. Pixi is the **only**
supported entrypoint — there is no expectation that the host system has
ROS2 installed.

## One-time setup

1. Install pixi:
   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   exec $SHELL   # pick up the new PATH
   ```
2. Resolve and download the environment (≈ 3 GB on first run, ARM64):
   ```bash
   cd ~/ros2_ws
   pixi install
   ```
   The conda env lands in `.pixi/envs/default/` (gitignored).

## Day-to-day

| Action | Command |
| --- | --- |
| Drop into a shell with ROS2 on PATH | `pixi shell` |
| Build everything | `pixi run build` |
| Build a single package | `pixi run build-perception` (or `build-nav`, `build-motor`) |
| Debug build | `pixi run build-debug` |
| Run tests | `pixi run test` |
| Show test results | `pixi run test-results` |
| Wipe build artifacts | `pixi run clean` |
| Launch SLAM (mapping) | `pixi run slam` |
| Launch Nav2 navigation | `pixi run nav2 -- map:=$HOME/maps/jetank_map.yaml` |
| Launch stereo camera | `pixi run stereo-camera` |
| Launch Gazebo sim | `pixi run gazebo` |

The colcon overlay (`install/setup.bash`) is auto-sourced by
`scripts/pixi-activate.sh` whenever a pixi shell is entered.

## How dependencies are organised

Everything lives in a single root `pixi.toml`. There are **no per-package
pixi manifests** — the JeTank `src/jetank_*` packages remain regular ROS2
packages, built by colcon inside the pixi environment. Channels:

1. `https://prefix.dev/robostack-staging` — provides all `ros-humble-*`
   packages (desktop, nav2, slam-toolbox, moveit, ros2_control, gazebo
   bindings, rplidar, …).
2. `conda-forge` — provides build tooling and native libraries
   (`cmake`, `ninja`, `libgpiod`, `gstreamer`, `yaml-cpp`, compilers).

The lock file (`pixi.lock`) pins every transitive package by hash; commit
it to source control so other contributors and CI converge to the same
environment.

## Hardware notes (Jetson-specific)

- **Architecture**: `linux-aarch64`. RoboStack ships full Humble desktop
  coverage for this platform.
- **GStreamer**: Pixi installs the upstream conda-forge gstreamer.
  This **does not include the NVMM hardware codec plugins** that come
  with JetPack's vendored gstreamer. CSI camera capture via NVMM
  pipelines will fall back to software-decoded paths. If you need HW
  decode for the production perception pipeline, run the perception
  node against the host `/opt/nvidia/gstreamer/*` plugins outside the
  pixi shell.
- **CUDA**: Pixi detects `__cuda=12.6` from the L4T runtime as a
  virtual package, so CUDA-enabled conda packages are eligible. The
  JeTank packages do not currently link CUDA, so this is informational.
- **GPIO**: `libgpiod` is in the env. The motor driver opens
  `/dev/gpiochip*` directly; that device needs to be readable by the
  user running `pixi shell` — it is on this Jetson.

## Updating

```bash
pixi update                # bump every dep to latest compatible
pixi update ros-humble-nav2-bringup   # bump a single package
```

After any update, commit the resulting `pixi.lock`.

## Why pixi-only?

Mixing a system-wide `/opt/ros/humble` install with a conda ROS install
in the same shell causes namespace collisions (two `rclpy` packages, two
sets of message types, etc.) that fail in subtle, hard-to-debug ways at
runtime. Picking a single owner — pixi — removes the entire class of
"works in pixi shell, breaks in system shell" bug.
