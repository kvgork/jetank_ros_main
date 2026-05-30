#!/usr/bin/env bash
# Sourced automatically when `pixi shell` or `pixi run` enters this workspace.
# Layers the colcon overlay on top of the conda-managed ROS2 install.

# Conda env already sources the system /opt/ros/... layout via pixi activation
# scripts shipped with the ros-humble-* packages, so nothing to do for the
# underlay. Only attach the local colcon overlay if it has been built.

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    . "${WS_ROOT}/install/setup.bash"
fi
