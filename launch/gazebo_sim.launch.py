import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # All world assets ship with jetank_simulation (the sim package owns worlds).
    pkg_jetank_simulation = get_package_share_directory('jetank_simulation')

    # Declare world selection argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load: empty, simple_test, obstacle_course, sock_arena, house'
    )

    # Start the arm_controller active (needed when MoveIt drives the arm).
    start_arm_active = LaunchConfiguration('start_arm_active')
    declare_start_arm_active_arg = DeclareLaunchArgument(
        'start_arm_active',
        default_value='false',
        description='Start arm_controller active instead of inactive'
    )

    # gui:=false => server-only Gazebo (lighter, no GUI window); web/headless runs.
    gui = LaunchConfiguration('gui')
    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run the Gazebo GUI client (false => server-only).'
    )

    # Map world names to .sdf files in jetank_simulation/worlds
    world_files = {
        'empty': 'empty_fortress.sdf',
        'simple_test': 'simple_test.sdf',
        'obstacle_course': 'obstacle_course.sdf',
        'sock_arena': 'sock_arena.sdf',
        'house': 'house.sdf',
    }

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_world_arg)
    ld.add_action(declare_start_arm_active_arg)
    ld.add_action(declare_gui_arg)

    # One parameterized include per world name, gated on world:=<name>.
    # An unrecognized name matches no condition and launches nothing (same
    # semantics as the previous per-world blocks).
    for name, sdf in world_files.items():
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('jetank_simulation'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={
                'world': os.path.join(pkg_jetank_simulation, 'worlds', sdf),
                'start_arm_active': start_arm_active,
                'gui': gui,
            }.items(),
            condition=LaunchConfigurationEquals('world', name)
        ))

    return ld
