#!/usr/bin/env python3
r"""
Unified launch file for complete JeTank robot system.

Launches integrated system with:
  - Robot description (URDF/TF)
  - Motor control (base mobility + odometry)
  - Stereo perception (depth + point cloud)
  - Laser scan converter (navigation sensor)
  - Web control interface (optional, browser-based remote control)
  - MoveIt2 arm control (optional, manipulation)
  - Navigation stack (optional, SLAM or Nav2)

Launch arguments:
  use_sim_time:       Use simulation clock (default: false)
  enable_web_control: Enable browser remote control (default: true)
  web_port:           Port for web control server (default: 8080)
  enable_navigation:  Enable Nav2/SLAM (default: false)
  enable_moveit:      Enable MoveIt2 arm control (default: false)
  navigation_mode:    'slam' or 'nav2' (default: 'slam')
  map_file:           Map YAML for nav2 mode (default: '')
  hardware:           ros2_control backend for MoveIt: 'mock' (no motors) or
                      'serial' (real JetankSerial servos). Default 'mock'
                      preserves existing callers. Passed through to
                      moveit_bringup.launch.py unchanged.
  left_frame_id:      Left camera optical frame id, forwarded to
                      stereo_camera.launch.py's left_frame_id arg (overrides the
                      non-optical *_link default in stereo_camera_config.yaml so
                      disparity reprojection is z-forward). Default
                      'camera_left_optical_frame'.
  right_frame_id:     Right camera optical frame id. Default
                      'camera_right_optical_frame'.

Usage:
  # Full system (hardware) with web control:
  ros2 launch jetank_ros_main unified.launch.py

  # Disable web control:
  ros2 launch jetank_ros_main unified.launch.py enable_web_control:=false

  # Custom web port:
  ros2 launch jetank_ros_main unified.launch.py web_port:=9090

  # Base mobility only (no navigation/arm):
  ros2 launch jetank_ros_main unified.launch.py \
    enable_navigation:=false enable_moveit:=false

  # Navigation with existing map:
  ros2 launch jetank_ros_main unified.launch.py \
    navigation_mode:=nav2 map_file:=/path/to/map.yaml

  # Hardware mission (real servos, optical frame ids):
  ros2 launch jetank_ros_main unified.launch.py \
    hardware:=serial \
    left_frame_id:=camera_left_optical_frame \
    right_frame_id:=camera_right_optical_frame
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate unified launch description."""
    # ============================================================================
    # PACKAGE DIRECTORIES
    # ============================================================================
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')
    pkg_jetank_perception = get_package_share_directory('jetank_perception')
    pkg_jetank_navigation = get_package_share_directory('jetank_navigation')
    # jetank_description is only needed by the lazy MoveIt builder.

    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_web_control = LaunchConfiguration('enable_web_control')
    web_port = LaunchConfiguration('web_port')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_moveit = LaunchConfiguration('enable_moveit')
    navigation_mode = LaunchConfiguration('navigation_mode')
    map_file = LaunchConfiguration('map_file')
    hardware = LaunchConfiguration('hardware')
    left_frame_id = LaunchConfiguration('left_frame_id')
    right_frame_id = LaunchConfiguration('right_frame_id')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_web_control = DeclareLaunchArgument(
        'enable_web_control',
        default_value='true',
        description='Enable browser-based web control interface'
    )

    declare_web_port = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web control HTTP server'
    )

    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable navigation stack (SLAM/Nav2)'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='false',
        description='Enable MoveIt2 arm control'
    )

    declare_navigation_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='slam',
        choices=['slam', 'nav2'],
        description='Navigation mode: slam for mapping, nav2 for navigation'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map YAML file (required for nav2 mode)'
    )

    declare_hardware = DeclareLaunchArgument(
        'hardware',
        default_value='mock',
        choices=['mock', 'serial'],
        description=(
            'ros2_control backend for MoveIt2: '
            'mock = software-only (arm reports success, no motors move); '
            'serial = real JetankSerialHardware servos (requires /dev/ttyTHS1).'
        )
    )

    # stereo_camera.launch.py has no launch args for frame_ids — they come from
    # stereo_camera_config.yaml (frames.left_frame_id / frames.right_frame_id).
    # We pass them as direct parameter overrides via launch_arguments using the
    # dotted YAML key syntax that stereo_camera_node declares with
    # declare_parameter("frames.left_frame_id", ...).
    declare_left_frame_id = DeclareLaunchArgument(
        'left_frame_id',
        default_value='camera_left_optical_frame',
        description=(
            'Left camera optical frame id (overrides frames.left_frame_id in '
            'stereo_camera_config.yaml). Used by disparity reprojection; must '
            'match the URDF optical-frame name. Real config default is '
            'camera_left_link; sim / optical correct value is '
            'camera_left_optical_frame.'
        )
    )

    declare_right_frame_id = DeclareLaunchArgument(
        'right_frame_id',
        default_value='camera_right_optical_frame',
        description=(
            'Right camera optical frame id (overrides frames.right_frame_id in '
            'stereo_camera_config.yaml). Same usage as left_frame_id.'
        )
    )

    # ============================================================================
    # LAYER 1: ROBOT DESCRIPTION
    # ============================================================================

    # Robot state publisher (URDF + TF tree)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'urdf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Static TF: world → base_footprint (MoveIt2 virtual joint)
    world_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", enable_moveit, "' == 'true'"
        ]))
    )

    # ============================================================================
    # WEB CONTROL (Conditional)
    # ============================================================================

    pkg_web_control = get_package_share_directory('jetank_web_control')

    web_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web_control, 'launch', 'web_control.launch.py')
        ),
        launch_arguments={'web_port': web_port}.items(),
        condition=IfCondition(PythonExpression([
            "'", enable_web_control, "' == 'true'"
        ]))
    )

    # ============================================================================
    # LAYER 2: HARDWARE INTERFACES
    # ============================================================================

    # Motor controller (base mobility + odometry)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'motor_controller.launch.py')
        )
    )

    # Stereo camera (perception pipeline). left/right_frame_id forward to
    # stereo_camera.launch.py's frame args (default the z-forward optical frames),
    # so disparity/pointcloud reprojection matches the URDF geometry on hardware.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_perception, 'launch', 'stereo_camera.launch.py')
        ),
        launch_arguments={
            'namespace': 'stereo_camera',
            'publish_camera_transforms': 'false',  # TF handled by URDF
            'left_frame_id': left_frame_id,
            'right_frame_id': right_frame_id,
        }.items()
    )

    # IMU (ICM-20948 on Waveshare IMX219-83 Stereo Camera module)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'imu.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Laser scan source: C1M1 RPLidar hardware
    laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'lidar.launch.py')
        )
    )

    # ============================================================================
    # LAYER 3: MOVEIT2 ARM CONTROL (Conditional)
    # ============================================================================
    # All MoveIt orchestration lives in jetank_moveit_config/launch/moveit_bringup.launch.py
    # so MoveIt remains self-contained. We just IncludeLaunchDescription with a
    # condition; the configs / move_group / spawners are only parsed and
    # spawned when enable_moveit:=true.
    # The 'hardware' arg is passed through so moveit_bringup selects the correct
    # ros2_control backend baked into the robot description XACRO:
    #   mock   -> mock ros2_control (arm reports SUCCESS, no servos move)
    #   serial -> JetankSerialHardware (real servos on /dev/ttyTHS1)

    pkg_jetank_moveit = get_package_share_directory('jetank_moveit_config')
    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_moveit, 'launch', 'moveit_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',  # The unified launch handles its own RViz.
            'hardware': hardware,
        }.items(),
        condition=IfCondition(enable_moveit),
    )

    # ============================================================================
    # LAYER 4: NAVIGATION STACK (Conditional)
    # ============================================================================

    # SLAM Toolbox (mapping mode)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
                "'", enable_navigation, "' == 'true' and '", navigation_mode, "' == 'slam'"
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Nav2 stack (navigation mode)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
                "'", enable_navigation, "' == 'true' and '", navigation_mode, "' == 'nav2'"
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )

    # ============================================================================
    # LAUNCH INFORMATION
    # ============================================================================

    launch_info = LogInfo(
        msg=[
            '\n========================================\n',
            'JeTank Unified System Launch\n',
            '========================================\n',
            '  Use Sim Time:   ', use_sim_time, '\n',
            '  Web Control:    ', enable_web_control, ' (port ', web_port, ')\n',
            '  Navigation:     ', enable_navigation, ' (', navigation_mode, ')\n',
            '  MoveIt2:        ', enable_moveit, ' (hardware=', hardware, ')\n',
            '  LiDAR: RPLidar C1M1 (hardware)\n',
            '  IMU: ICM-20948 (imu/data_raw, imu/magnetic_field)\n',
            '  Map File:       ', map_file, '\n',
            '  Camera frames:  left=', left_frame_id, ' right=', right_frame_id, '\n',
            '========================================\n'
        ]
    )

    # ============================================================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================================================

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_web_control)
    ld.add_action(declare_web_port)
    ld.add_action(declare_enable_navigation)
    ld.add_action(declare_enable_moveit)
    ld.add_action(declare_navigation_mode)
    ld.add_action(declare_map_file)
    ld.add_action(declare_hardware)
    ld.add_action(declare_left_frame_id)
    ld.add_action(declare_right_frame_id)

    # Launch info
    ld.add_action(launch_info)

    # Layer 1: Robot description
    ld.add_action(urdf_launch)
    ld.add_action(world_to_base_tf)

    # Layer 2: Hardware interfaces
    ld.add_action(motor_launch)
    ld.add_action(camera_launch)
    ld.add_action(imu_launch)
    ld.add_action(laser_scan_launch)

    # Web control (conditional)
    ld.add_action(web_control_launch)

    # Layer 3: MoveIt2 (conditional, delegated to jetank_moveit_config)
    ld.add_action(moveit_bringup)

    # Layer 4: Navigation (conditional)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    return ld
