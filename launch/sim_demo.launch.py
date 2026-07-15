#!/usr/bin/env python3
r"""
One-shot simulation demo bring-up for the JeTank robot.

Brings up the WHOLE demo with no arguments — `ros2 launch jetank_ros_main
sim_demo.launch.py` starts:

  - Gazebo Fortress (GUI), sock_arena world (default)
  - Robot via gz_ros2_control (diff-drive base + arm + gripper)
  - Lidar (/scan), IMU (/imu), stereo cameras bridged to ROS 2
  - RViz, SLAM (slam_toolbox)
  - Arm: MoveIt move_group + arm_controller + the grasp server (/grasp_object)
  - Web control on :8080 (drive + Grab button)
  - Sock detector, AUTO-configured + activated (loads ~/models/sock_sim.pt)

This is the simulation counterpart to ``navigation_full.launch.py`` (real robot).
It reuses the sim-aware path of ``navigation_full`` (``use_sim_time:=true``).

Usage::

    # Full demo (arm + web + detection + SLAM + RViz), sock_arena
    ros2 launch jetank_ros_main sim_demo.launch.py

    # Turn pieces off
    ros2 launch jetank_ros_main sim_demo.launch.py detect:=false arm:=false slam:=false

    # Different world / detection model
    ros2 launch jetank_ros_main sim_demo.launch.py world:=house model_path_sim:=/path/to.pt

Drive the base from another terminal (note: TwistStamped on the controller topic)::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -p stamped:=true \\
        -r /cmd_vel:=/diff_drive_controller/cmd_vel

Move the arm interactively with MoveIt instead via::

    ros2 launch jetank_moveit_config moveit_sim.launch.py
"""

import os

from jetank_ros_main.topics import (
    camera_left_raw,
    detections_socks,
    detections_socks_debug,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the simulation demo launch description."""
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('rviz')
    arm = LaunchConfiguration('arm')
    web = LaunchConfiguration('web')
    detect = LaunchConfiguration('detect')
    model_path_sim = LaunchConfiguration('model_path_sim')

    # Defaults bring up the WHOLE demo with no args: sock_arena world + SLAM +
    # RViz + arm (MoveIt + grasp server) + web UI + sock detection (auto-activated).
    # Turn pieces off explicitly, e.g. `sim_demo.launch.py arm:=false detect:=false`.
    declare_world = DeclareLaunchArgument(
        'world', default_value='sock_arena',
        description='World to load: empty, simple_test, obstacle_course, sock_arena, house')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Run slam_toolbox (mapping) against the simulated lidar')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz')
    declare_arm = DeclareLaunchArgument(
        'arm', default_value='true',
        description='Start MoveIt move_group + arm_controller + the grasp server (Grab action)')
    declare_web = DeclareLaunchArgument(
        'web', default_value='true',
        description='Start the web control in sim mode (port 8080, cmd_vel bridge, Grab button)')
    declare_detect = DeclareLaunchArgument(
        'detect', default_value='true',
        description='Start + auto-activate the sock detector against the sim left camera')
    declare_model_path_sim = DeclareLaunchArgument(
        'model_path_sim', default_value=os.path.expanduser('~/models/sock_sim.pt'),
        description='Trained sim model (.pt/.engine) for the sock detector. Default '
                    '~/models/sock_sim.pt; detector logs a warning if absent.')

    # 1. Gazebo + robot + sensors + controllers (sim-time, GUI). When arm:=true
    #    the arm_controller is started active so MoveIt can drive it.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_ros_main'),
                                  'launch', 'gazebo_sim.launch.py'])),
        launch_arguments={'world': world, 'start_arm_active': arm}.items(),
    )

    # 2. RViz with the project's unified.rviz config (RobotModel + TF + lidar
    #    scan + map + Navigation 2 panel). When arm:=true, RViz is instead
    #    launched by moveit_sim below so it carries the MoveIt SRDF/kinematics
    #    params the MotionPlanning panel needs — so launch this plain RViz only
    #    when the arm is NOT requested.
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_ros_main'),
                                  'launch', 'rviz.launch.py'])),
        condition=IfCondition(PythonExpression(
            ["'", use_rviz, "' == 'true' and '", arm, "' == 'false'"])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # 3. Optional SLAM: reuse navigation_full's sim-aware path (no hardware
    #    nodes because use_sim_time:=true). RViz is provided above by unified.rviz,
    #    so suppress navigation_full's own RViz to avoid a duplicate window.
    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_navigation'),
                                  'launch', 'navigation_full.launch.py'])),
        condition=IfCondition(slam),
        launch_arguments={
            'mode': 'slam',
            'use_sim_time': 'true',
            'rviz': 'false',
        }.items(),
    )

    # 4. Optional arm: attach MoveIt move_group to THIS Gazebo (start_gazebo:=false
    #    so it does not spawn a second simulation). RViz is the unified one above.
    arm_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_moveit_config'),
                                  'launch', 'moveit_sim.launch.py'])),
        condition=IfCondition(arm),
        launch_arguments={
            'start_gazebo': 'false',
            # When arm and rviz are both on, moveit_sim provides RViz loaded
            # with unified.rviz AND the MoveIt params the MotionPlanning panel
            # needs (the plain RViz above is suppressed in that case).
            'use_rviz': use_rviz,
            'rviz_config': PathJoinSubstitution([
                FindPackageShare('jetank_ros_main'), 'rviz', 'unified.rviz']),
        }.items(),
    )

    # 5. Optional web control (sim mode): serves the UI on :8080 and bridges
    #    its Twist /cmd_vel to the controller's TwistStamped topic. Topic names
    #    come from the system topic contract (config/topics.yaml): the raw
    #    camera stream (Gazebo has no compressed transport) as a launch arg,
    #    and the detections topic via a scoped SetParameter (the included
    #    launch file declares no launch arg for it).
    web_stack = GroupAction(
        condition=IfCondition(web),
        actions=[
            SetParameter(name='detections_topic', value=detections_socks()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('jetank_web_control'),
                                          'launch', 'web_control.launch.py'])),
                launch_arguments={
                    'sim': 'true',
                    'image_topic': camera_left_raw(),
                }.items(),
            ),
        ],
    )

    # 6. Optional sock detector (jetank_detection): uses the SIM entry point
    #    (detect_sim.launch.py), which pins sim:=true so the node loads the sim
    #    model (model_path_sim) — sim and real need different models because the
    #    synthetic Gazebo imagery differs from real camera frames.
    #    The sim publishes the same left-camera raw topic as the real robot,
    #    so no remapping is needed. Runs continuous (live) by default.
    #    The detector lifecycle is auto-activated by detector_autostart below.
    #    Topic names come from the system topic contract (config/topics.yaml);
    #    the detections/debug topics ride a scoped SetParameter because the
    #    included launch file declares no launch args for them.
    detect_stack = GroupAction(
        condition=IfCondition(detect),
        actions=[
            SetParameter(name='detections_topic', value=detections_socks()),
            SetParameter(name='debug_image_topic', value=detections_socks_debug()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('jetank_detection'),
                                          'launch', 'detect_sim.launch.py'])),
                launch_arguments={
                    'input_image_topic': camera_left_raw(),
                    'continuous': 'true',
                    'model_path_sim': model_path_sim,
                }.items(),
            ),
        ],
    )

    # 7. Grasp server (jetank_manipulation): advertises /grasp_object and backs the
    #    web Grab button. Needs the arm, so gate on arm:=true.
    grasp_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_manipulation'),
                                  'launch', 'grasp.launch.py'])),
        condition=IfCondition(arm),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # 8. Auto-activate the sock_detector lifecycle node so the demo is one command
    #    (no manual `ros2 lifecycle set`). Polls until the node exists (Gazebo is
    #    slow to come up), then configure -> activate. Non-fatal if the model is
    #    missing — the detector just stays unconfigured and logs a warning.
    detector_autostart = ExecuteProcess(
        condition=IfCondition(detect),
        cmd=['bash', '-c',
             'for i in $(seq 1 90); do '
             'ros2 node list 2>/dev/null | grep -q /sock_detector && break; sleep 2; done; '
             'ros2 lifecycle set /sock_detector configure && sleep 3 && '
             'ros2 lifecycle set /sock_detector activate'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_slam)
    ld.add_action(declare_rviz)
    ld.add_action(declare_arm)
    ld.add_action(declare_web)
    ld.add_action(declare_detect)
    ld.add_action(declare_model_path_sim)
    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(slam_stack)
    ld.add_action(arm_stack)
    ld.add_action(web_stack)
    ld.add_action(detect_stack)
    ld.add_action(grasp_stack)
    ld.add_action(detector_autostart)
    return ld
