"""
Publish the canonical JeTank robot model (robot_state_publisher + JSP).

Expands the canonical entrypoint jetank_description/urdf/
jetank_ros2_control.urdf.xacro via the canonical include
jetank_description/launch/robot_description.launch.py (the same model used
by the sim2real bringup, jetank_simulation and jetank_moveit_config). The
ros2_control block is disabled here (use_ros2_control:=false): this launch
only publishes description + TF for visualisation/bringup; controllers are
brought up by the motor/MoveIt layers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the description-publishing launch description."""
    pkg_share = get_package_share_directory('jetank_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_jsp = LaunchConfiguration('use_jsp', default='true')

    # Canonical include: expands jetank_ros2_control.urdf.xacro and starts
    # robot_state_publisher.
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={
            'use_ros2_control': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'use_jsp',
            default_value='true',
            description=(
                'Start joint_state_publisher. Set to false when another node '
                '(e.g. joint_state_broadcaster via ros2_control/MoveIt) is '
                'already publishing /joint_states, to avoid two publishers '
                'fighting over the same topic.'
            )
        ),

        robot_description_launch,

        # Joint State Publisher (only when nothing else publishes /joint_states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_jsp),
        ),
    ])
