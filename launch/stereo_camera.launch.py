#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package containing the stereo camera node
    stereo_package_name = 'jetank_perception'
    
    # Launch arguments
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height', 
        default_value='480',
        description='Camera image height'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='20',
        description='Camera frames per second'
    )
    
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU acceleration for stereo processing'
    )
    
    stereo_algorithm_arg = DeclareLaunchArgument(
        'stereo_algorithm',
        default_value='GPU_BM',
        choices=['GPU_BM', 'CPU_BM', 'GPU_SGBM', 'CPU_SGBM'],
        description='Stereo matching algorithm to use'
    )
    
    num_disparities_arg = DeclareLaunchArgument(
        'num_disparities',
        default_value='64',
        description='Number of disparities for stereo matching'
    )
    
    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='15',
        description='Block size for stereo matching (must be odd)'
    )
    
    left_frame_id_arg = DeclareLaunchArgument(
        'left_frame_id',
        default_value='camera_left_link',
        description='Frame ID for left camera'
    )
    
    right_frame_id_arg = DeclareLaunchArgument(
        'right_frame_id',
        default_value='camera_right_link',
        description='Frame ID for right camera'
    )
    
    left_camera_info_url_arg = DeclareLaunchArgument(
        'left_camera_info_url',
        default_value='',
        description='URL to left camera calibration file'
    )
    
    right_camera_info_url_arg = DeclareLaunchArgument(
        'right_camera_info_url',
        default_value='',
        description='URL to right camera calibration file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='stereo_camera',
        description='Namespace for the stereo camera node'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Point cloud filtering parameters
    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.01',
        description='Voxel grid leaf size for point cloud downsampling'
    )
    
    statistical_filter_k_arg = DeclareLaunchArgument(
        'statistical_filter_k',
        default_value='50',
        description='Number of neighbors for statistical outlier removal'
    )
    
    statistical_filter_stddev_arg = DeclareLaunchArgument(
        'statistical_filter_stddev',
        default_value='1.0',
        description='Standard deviation threshold for statistical outlier removal'
    )
    
    max_processing_threads_arg = DeclareLaunchArgument(
        'max_processing_threads',
        default_value='4',
        description='Maximum number of processing threads'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Log level for the node'
    )
    
    # Node configuration
    stereo_camera_node = Node(
        package=stereo_package_name,
        executable='stereo_camera_node',
        name='stereo_camera_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            # Camera parameters
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            
            # Frame IDs
            'left_frame_id': LaunchConfiguration('left_frame_id'),
            'right_frame_id': LaunchConfiguration('right_frame_id'),
            
            # Calibration files
            'left_camera_info_url': LaunchConfiguration('left_camera_info_url'),
            'right_camera_info_url': LaunchConfiguration('right_camera_info_url'),
            
            # Processing parameters
            'use_gpu': LaunchConfiguration('use_gpu'),
            'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
            'num_disparities': LaunchConfiguration('num_disparities'),
            'block_size': LaunchConfiguration('block_size'),
            'min_disparity': 0,
            'max_disparity': LaunchConfiguration('num_disparities'),
            
            # Point cloud filtering
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
            'statistical_filter_k': LaunchConfiguration('statistical_filter_k'),
            'statistical_filter_stddev': LaunchConfiguration('statistical_filter_stddev'),
            'max_processing_threads': LaunchConfiguration('max_processing_threads'),
            
            # ROS parameters
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics if needed
            # ('left/image_raw', 'stereo/left/image_raw'),
            # ('right/image_raw', 'stereo/right/image_raw'),
            # ('left/camera_info', 'stereo/left/camera_info'),
            # ('right/camera_info', 'stereo/right/camera_info'),
            # ('points', 'stereo/points'),
        ]
    )
    
    # Optional: Add a static transform publisher for camera frames
    # This is useful if you need to define the relationship between camera frames
    left_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_camera_tf_publisher',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link',  # parent frame
            LaunchConfiguration('left_frame_id')  # child frame
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_transforms'))
    )
    
    right_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_camera_tf_publisher',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '0.06', '0', '0',  # x, y, z (6cm baseline)
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link',  # parent frame
            LaunchConfiguration('right_frame_id')  # child frame
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_transforms'))
    )
    
    # Argument for enabling camera transforms
    publish_camera_transforms_arg = DeclareLaunchArgument(
        'publish_camera_transforms',
        default_value='false',
        description='Publish static transforms for camera frames'
    )
    
    # Log launch information
    launch_info = LogInfo(
        msg=[
            'Launching Jetson Stereo Camera Node with the following configuration:\n',
            '  Package: ', stereo_package_name, '\n',
            '  Namespace: ', LaunchConfiguration('namespace'), '\n',
            '  Resolution: ', LaunchConfiguration('camera_width'), 'x', LaunchConfiguration('camera_height'), '\n',
            '  FPS: ', LaunchConfiguration('camera_fps'), '\n',
            '  Stereo Algorithm: ', LaunchConfiguration('stereo_algorithm'), '\n',
            '  GPU Acceleration: ', LaunchConfiguration('use_gpu'), '\n',
            '  Log Level: ', LaunchConfiguration('log_level')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        use_gpu_arg,
        stereo_algorithm_arg,
        num_disparities_arg,
        block_size_arg,
        left_frame_id_arg,
        right_frame_id_arg,
        left_camera_info_url_arg,
        right_camera_info_url_arg,
        namespace_arg,
        use_sim_time_arg,
        voxel_leaf_size_arg,
        statistical_filter_k_arg,
        statistical_filter_stddev_arg,
        max_processing_threads_arg,
        log_level_arg,
        publish_camera_transforms_arg,
        
        # Log launch info
        launch_info,
        
        # Nodes
        stereo_camera_node,
        left_camera_tf,
        right_camera_tf,
    ])