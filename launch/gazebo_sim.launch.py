import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_jetank_ros_main = get_package_share_directory('jetank_ros_main')
    pkg_jetank_simulation = get_package_share_directory('jetank_simulation')

    # Declare world selection argument
    world_name = LaunchConfiguration('world')
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load: empty, simple_test, obstacle_course, sock_arena'
    )

    # Map world names to file paths
    # Note: Python dictionary used for world selection
    world_files = {
        'empty': os.path.join(pkg_jetank_simulation, 'worlds', 'empty_fortress.sdf'),
        'simple_test': os.path.join(pkg_jetank_ros_main, 'worlds', 'simple_test.sdf'),
        'obstacle_course': os.path.join(pkg_jetank_ros_main, 'worlds', 'obstacle_course.sdf'),
        'sock_arena': os.path.join(pkg_jetank_ros_main, 'worlds', 'sock_arena.sdf'),
    }

    # Get selected world file path (default to empty if invalid selection)
    # Note: This evaluates at launch time based on the world argument
    def get_world_file(context):
        world_choice = context.launch_configurations.get('world', 'empty')
        return world_files.get(world_choice, world_files['empty'])

    # Include base gazebo launch with selected world
    from launch.substitutions import LaunchConfiguration, PythonExpression
    from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

    # We need to use multiple IncludeLaunchDescription with conditions
    # to properly substitute the world file path at launch time

    # Empty world
    gazebo_empty = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jetank_simulation'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': os.path.join(pkg_jetank_simulation, 'worlds', 'empty_fortress.sdf')
        }.items(),
        condition=LaunchConfigurationEquals('world', 'empty')
    )

    # Simple test world
    gazebo_simple_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jetank_simulation'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': os.path.join(pkg_jetank_ros_main, 'worlds', 'simple_test.sdf')
        }.items(),
        condition=LaunchConfigurationEquals('world', 'simple_test')
    )

    # Obstacle course world
    gazebo_obstacle_course = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jetank_simulation'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': os.path.join(pkg_jetank_ros_main, 'worlds', 'obstacle_course.sdf')
        }.items(),
        condition=LaunchConfigurationEquals('world', 'obstacle_course')
    )

    # Sock arena world
    gazebo_sock_arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jetank_simulation'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': os.path.join(pkg_jetank_ros_main, 'worlds', 'sock_arena.sdf')
        }.items(),
        condition=LaunchConfigurationEquals('world', 'sock_arena')
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_world_arg)

    # Add conditional gazebo launches
    ld.add_action(gazebo_empty)
    ld.add_action(gazebo_simple_test)
    ld.add_action(gazebo_obstacle_course)
    ld.add_action(gazebo_sock_arena)

    return ld
