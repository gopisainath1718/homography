import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/rainier/ros2_ws/src/littleslam_ros2/launch/lidar_visualizer.py'],  # Full path to your Python script
            name='lidar_visualizer_node',
            output='screen'
        ),
    ])

