#!/usr/bin/env python3
"""
All-in-one mobile sock-grasp stack (HARDWARE).

Hardware counterpart of ``mobile_grasp.launch.py`` (which is sim-only). Brings up
the full pick pipeline on the physical JeTank:

  unified (motor+odom + real stereo + IMU + lidar + MoveIt hardware:=serial)
  + sock detector (real model) + segmentation action server
  + grasp_server + base_approach_node + mobile_grasp_coordinator
  + cmd_vel_bridge (TwistStamped manip -> Twist /cmd_vel for robot_controller)

Then trigger a pick:
  ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger

Key sim->hardware differences vs mobile_grasp.launch.py:
  - use_sim_time:=false everywhere (there is no /clock on hardware).
  - ros2_control backend = JetankSerialHardware (real Feetech servos), via
    unified.launch.py hardware:=serial.
  - odom->base_footprint comes from robot_controller's open-loop integrator
    (drifts; no encoders) — required by base_approach_node + the coordinator.
  - base motion is bridged: base_approach_node publishes TwistStamped on
    /cmd_vel_manip; cmd_vel_bridge converts it to plain Twist on /cmd_vel, which
    robot_controller subscribes to. (Sim published TwistStamped straight to the
    Gazebo diff_drive_controller — that path does not exist on hardware.)

PRE-REQUISITES (NOT handled here — see plans/sim2real-gap-analysis.md):
  - A real model: pass model_path_real:=/path/to/sock_real.pt (none ships yet).
  - The CSI stereo node must publish /stereo_camera/* . On this platform the
    GStreamer-backed camera node must run under SYSTEM ROS2 (/opt/ros/humble),
    NOT the pixi env (pixi OpenCV lacks the GStreamer binding). Launch it
    separately there; this stack consumes the topics it publishes.
  - Bench safety: first bring-up with wheels off the ground.

Args:
----
  model_path_real (/home/koen/models/sock_real.pt)  real YOLO/TRT model
  confidence      (0.5)    detector confidence
  enable_web_control (false)  browser teleop (own cmd_vel_bridge is used instead)

"""

import os

from ament_index_python.packages import get_package_share_directory

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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    model_path_real = LaunchConfiguration("model_path_real")
    confidence = LaunchConfiguration("confidence")
    enable_web_control = LaunchConfiguration("enable_web_control")

    ros_main = get_package_share_directory("jetank_ros_main")
    detection = get_package_share_directory("jetank_detection")
    manipulation = get_package_share_directory("jetank_manipulation")
    grasp_poses_yaml = os.path.join(manipulation, "config", "grasp_poses.yaml")

    not_sim = {"use_sim_time": False}

    # --- core hardware stack: motor(+odom) + real stereo + IMU + lidar + MoveIt
    #     with the REAL serial servo backend. ---
    unified = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_main, "launch", "unified.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "hardware": "serial",       # JetankSerialHardware (real Feetech servos)
            "enable_moveit": "true",
            "enable_navigation": "false",
            "enable_web_control": enable_web_control,
        }.items(),
    )

    # --- cmd_vel bridge: manip TwistStamped (/cmd_vel_manip) -> Twist (/cmd_vel).
    #     robot_controller subscribes to plain Twist on /cmd_vel. nav_topic empty
    #     (Nav2 disabled here); teleop empty (web control off by default). ---
    cmd_vel_bridge = Node(
        package="jetank_web_control",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
        parameters=[{
            "output_stamped": False,
            "output_topic": "/cmd_vel",
            "manip_topic": "/cmd_vel_manip",
            "teleop_topic": "",
            "nav_topic": "",
            "use_sim_time": False,
        }],
    )

    # --- detector (real model), staggered after the core stack is up.
    #     Topic names come from the system topic contract (config/topics.yaml);
    #     the detections/debug topics ride a scoped SetParameter because
    #     detect_real.launch.py declares no launch args for them. ---
    detector = TimerAction(period=12.0, actions=[
        GroupAction([
            SetParameter(name="detections_topic", value=detections_socks()),
            SetParameter(name="debug_image_topic", value=detections_socks_debug()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(detection, "launch", "detect_real.launch.py")
                ),
                launch_arguments={
                    "input_image_topic": camera_left_raw(),
                    "model_path_real": model_path_real,
                    "continuous": "true",
                    "confidence": confidence,
                }.items(),
            ),
        ]),
    ])

    # --- pipeline nodes (use_sim_time:=false) ---
    seg = Node(package="jetank_perception", executable="sock_segmentation_server",
               name="sock_segmentation_server", parameters=[not_sim], output="screen")
    grasp = Node(package="jetank_manipulation", executable="grasp_server",
                 name="grasp_server", parameters=[grasp_poses_yaml, not_sim],
                 output="screen")
    # base_approach publishes TwistStamped to /cmd_vel_manip (bridged to /cmd_vel),
    # NOT the sim diff_drive_controller topic.
    approach = Node(package="jetank_manipulation", executable="base_approach_node",
                    name="base_approach_node",
                    parameters=[{"cmd_vel_topic": "/cmd_vel_manip"}, not_sim],
                    output="screen")
    coordinator = Node(package="jetank_manipulation", executable="mobile_grasp_coordinator",
                       name="mobile_grasp_coordinator", parameters=[not_sim],
                       output="screen")
    pipeline = TimerAction(period=16.0, actions=[seg, grasp, approach, coordinator])

    # --- auto configure + activate the sock_detector lifecycle node ---
    lc_configure = TimerAction(period=22.0, actions=[ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/sock_detector", "configure"], output="screen")])
    lc_activate = TimerAction(period=28.0, actions=[ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/sock_detector", "activate"], output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument("model_path_real",
                              default_value="/home/koen/models/sock_real.pt"),
        DeclareLaunchArgument("confidence", default_value="0.5"),
        DeclareLaunchArgument("enable_web_control", default_value="false"),
        unified, cmd_vel_bridge, detector, pipeline, lc_configure, lc_activate,
    ])
