from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_jetank_perception = FindPackageShare('jetank_perception')

    # Stereo camera node driven by simulated stereo image topics (gz).
    # In ros_topics mode it subscribes to the simulated stereo stream instead of
    # capturing from CSI cameras, and produces DisparityImage + PointCloud2.
    stereo_camera_node = Node(
        package='jetank_perception',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        namespace='stereo_camera',
        parameters=[
            PathJoinSubstitution([pkg_jetank_perception, 'config', 'stereo_camera_config.yaml']),
            {
                'use_sim_time': True,
                'camera.use_hardware_acceleration': False,  # No CUDA in sim -> CPU SGBM/BM
                'input_source': 'ros_topics',
                # Tag disparity/pointcloud with the true optical frame (z-forward)
                # so reprojected geometry is correct when TF'd into base_link.
                'frames.left_frame_id': 'camera_left_optical_frame',
                'frames.right_frame_id': 'camera_right_optical_frame',
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        stereo_camera_node
    ])
