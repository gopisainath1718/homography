from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Use absolute path for SLLIDAR package
    sllidar_pkg_dir = '/home/rainier/ros2_ws/src/sllidar_ros2'  # Absolute path

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'map_name',
            default_value='default_map',
            description='Base name for saved map and pbstream file'
        ),

        # Include Realsense camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            )
        ),

        # Include SLLIDAR S2 LIDAR driver launch with the correct name for the launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_pkg_dir, 'launch', 'sllidar_s2_launch.py')  # Absolute path
            )
        ),
    ])

