from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

stereo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('jetank_perception'),
            'launch',
            'stereo_camera.launch.py'
        ])
    ]),
    # launch_arguments={
    #     'namespace': 'my_robot/stereo',
    #     'camera_width': '1280',  # Override config file value
    #     'log_level': 'debug'
    # }.items()
)