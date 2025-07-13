#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Basic arguments
        DeclareLaunchArgument(
            'use_gpu',
            default_value='true',
            description='Use GPU acceleration'
        ),
        
        DeclareLaunchArgument(
            'stereo_algorithm',
            default_value='GPU_BM',
            description='Stereo algorithm to use'
        ),
        
        # Stereo camera node
        Node(
            package='jetank_perception',
            executable='jetson_stereo_node',
            name='jetson_stereo_node',
            parameters=[{
                'camera_width': 640,
                'camera_height': 480,
                'camera_fps': 20,
                'use_gpu': LaunchConfiguration('use_gpu'),
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'num_disparities': 64,
                'block_size': 15,
                'left_frame_id': 'camera_left_link',
                'right_frame_id': 'camera_right_link',
                'voxel_leaf_size': 0.01,
                'statistical_filter_k': 50,
                'statistical_filter_stddev': 1.0
            }],
            output='screen'
        )
    ])