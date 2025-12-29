from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_jetank_perception = FindPackageShare('jetank_perception')

    # Point cloud generation node (processes simulated stereo images)
    point_cloud_node = Node(
        package='jetank_perception',
        executable='point_cloud_node',
        name='point_cloud_node',
        parameters=[
            PathJoinSubstitution([pkg_jetank_perception, 'config', 'stereo_camera_config.yaml']),
            {
                'use_sim_time': True,
                'camera.use_hardware_acceleration': False,  # No CUDA in sim
            }
        ],
        remappings=[
            ('/stereo_camera/left/image_raw', '/stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', '/stereo_camera/right/image_raw'),
        ],
        output='screen'
    )

    return LaunchDescription([
        point_cloud_node
    ])
