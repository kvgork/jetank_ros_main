#!/usr/bin/env python3
"""One-shot simulation demo bring-up for the JeTank robot.

Brings up everything you need to *see* and *drive* the robot in Gazebo:

  - Gazebo Fortress (GUI) with a selectable world (obstacle_course by default,
    so the lidar has walls + cylinders to range against)
  - The robot spawned with gz_ros2_control (diff-drive base + arm + gripper)
  - Simulated lidar (/scan), IMU (/imu) and stereo cameras, bridged to ROS 2
  - RViz for visualisation
  - Optional SLAM (slam_toolbox) so you can build a map while you drive

This is the simulation counterpart to ``navigation_full.launch.py`` (which is
for the real robot). It deliberately reuses the sim-aware path of
``navigation_full`` (``use_sim_time:=true``), which skips the hardware nodes.

Usage::

    # Default: obstacle world + RViz + SLAM
    ros2 launch jetank_ros_main sim_demo.launch.py

    # Pick a different world, no SLAM (plain robot view)
    ros2 launch jetank_ros_main sim_demo.launch.py world:=sock_arena slam:=false

    # Sock arena with live detection enabled
    ros2 launch jetank_ros_main sim_demo.launch.py world:=sock_arena detect:=true slam:=false

Drive the base from another terminal (note: TwistStamped on the controller topic)::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -p stamped:=true \\
        -r /cmd_vel:=/diff_drive_controller/cmd_vel

Move the arm interactively with MoveIt instead via::

    ros2 launch jetank_moveit_config moveit_sim.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the simulation demo launch description."""
    pkg_main = get_package_share_directory('jetank_ros_main')
    pkg_nav = get_package_share_directory('jetank_navigation')

    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('rviz')
    arm = LaunchConfiguration('arm')
    web = LaunchConfiguration('web')
    detect = LaunchConfiguration('detect')

    declare_world = DeclareLaunchArgument(
        'world', default_value='house',
        description='World to load: empty, simple_test, obstacle_course, sock_arena, house')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Run slam_toolbox (mapping) against the simulated lidar')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz')
    declare_arm = DeclareLaunchArgument(
        'arm', default_value='false',
        description='Also start MoveIt move_group and activate arm_controller')
    declare_web = DeclareLaunchArgument(
        'web', default_value='false',
        description='Also start the web control in sim mode (port 8080, cmd_vel bridge)')
    declare_detect = DeclareLaunchArgument(
        'detect', default_value='false',
        description='Also start the sock detector node (jetank_detection) against the sim left camera')

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
    #    its Twist /cmd_vel to the controller's TwistStamped topic.
    web_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_web_control'),
                                  'launch', 'web_control.launch.py'])),
        condition=IfCondition(web),
        launch_arguments={'sim': 'true'}.items(),
    )

    # 6. Optional sock detector (jetank_detection): launches the lifecycle
    #    detector node in continuous mode against the sim left camera.
    #    The sim publishes /stereo_camera/left/image_raw — same as the real robot,
    #    so no remapping is needed.
    #    After launch, lifecycle transitions are still required:
    #      ros2 lifecycle set /sock_detector configure
    #      ros2 lifecycle set /sock_detector activate
    detect_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_detection'),
                                  'launch', 'detect.launch.py'])),
        condition=IfCondition(detect),
        launch_arguments={
            'input_image_topic': '/stereo_camera/left/image_raw',
            'continuous': 'true',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_slam)
    ld.add_action(declare_rviz)
    ld.add_action(declare_arm)
    ld.add_action(declare_web)
    ld.add_action(declare_detect)
    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(slam_stack)
    ld.add_action(arm_stack)
    ld.add_action(web_stack)
    ld.add_action(detect_stack)
    return ld
