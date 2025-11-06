import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('jetank_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'jetank.xacro')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Read URDF content - process XACRO file
    robot_description_config = Command(['xacro', ' ', urdf_file])
    
    return LaunchDescription([
        # Declare launch argument for sim time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            #output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            #output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # # RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     #arguments=['-d', os.path.join(pkg_share, 'config', 'robot_view.rviz')]
        # )
    ])