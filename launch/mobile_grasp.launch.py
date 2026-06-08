#!/usr/bin/env python3
"""All-in-one mobile sock-grasp stack (sim).

Brings up the entire pick pipeline in one command so the mobile-manip grasp can be
driven/validated interactively:

  gazebo (+ros2_control controllers) + MoveIt move_group + stereo->disparity/cloud
  + sock detector (auto configure+activate) + segmentation action server
  + grasp_server (pose-targeted arm grasp) + base_approach_node + mobile_grasp_coordinator

Then trigger a pick:
  ros2 service call /mobile_grasp_coordinator/execute_sock_grasp std_srvs/srv/Trigger

Args:
  world           (sock_arena)  Gazebo world
  model_path_sim  (/home/koen/models/sock_sim.pt)  YOLO sim model
  confidence      (0.3)         detector confidence
  use_rviz        (true)        MoveIt RViz (set false for headless/CI)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    model_path_sim = LaunchConfiguration("model_path_sim")
    confidence = LaunchConfiguration("confidence")
    use_rviz = LaunchConfiguration("use_rviz")

    ros_main = FindPackageShare("jetank_ros_main")
    moveit = FindPackageShare("jetank_moveit_config")
    detection = FindPackageShare("jetank_detection")
    sim_time = {"use_sim_time": True}

    def inc(pkg, rel, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, "launch", rel])),
            launch_arguments=launch_args.items(),
        )

    # --- core stack (staggered: gz first, then move_group, then perception) ---
    gazebo = inc(ros_main, "gazebo_sim.launch.py", world=world)
    move_group = TimerAction(period=6.0, actions=[
        inc(moveit, "moveit_sim.launch.py", headless="false", use_rviz=use_rviz),
    ])
    perception = TimerAction(period=10.0, actions=[
        inc(ros_main, "stereo_camera_sim.launch.py"),
    ])
    detector = TimerAction(period=12.0, actions=[
        inc(detection, "detect_sim.launch.py",
            model_path_sim=model_path_sim, continuous="true", confidence=confidence),
    ])

    # --- pipeline nodes ---
    seg = Node(package="jetank_perception", executable="sock_segmentation_server",
               name="sock_segmentation_server", parameters=[sim_time], output="screen")
    grasp = Node(package="jetank_manipulation", executable="grasp_server",
                 name="grasp_server", parameters=[sim_time], output="screen")
    approach = Node(package="jetank_manipulation", executable="base_approach_node",
                    name="base_approach_node", parameters=[sim_time], output="screen")
    coordinator = Node(package="jetank_manipulation", executable="mobile_grasp_coordinator",
                       name="mobile_grasp_coordinator", parameters=[sim_time], output="screen")
    pipeline = TimerAction(period=14.0, actions=[seg, grasp, approach, coordinator])

    # --- auto configure + activate the sock_detector lifecycle node ---
    lc_configure = TimerAction(period=18.0, actions=[ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/sock_detector", "configure"], output="screen")])
    lc_activate = TimerAction(period=22.0, actions=[ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/sock_detector", "activate"], output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="sock_arena"),
        DeclareLaunchArgument("model_path_sim", default_value="/home/koen/models/sock_sim.pt"),
        DeclareLaunchArgument("confidence", default_value="0.3"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        gazebo, move_group, perception, detector, pipeline, lc_configure, lc_activate,
    ])
