import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess  # Use ExecuteProcess to run Python script

def generate_launch_description():
    # Path to the folder where the map will be saved
    map_save_folder = '/home/rainier/ros2_ws/src/littleslam_ros2/maps'
    
    # Ensure the map_save_folder exists
    os.makedirs(map_save_folder, exist_ok=True)

    # Declare the map name argument (you can modify this to dynamically change the file name if needed)
    map_name = LaunchConfiguration('map_name', default='my_map')

    # Define the process to execute the save_map.py script with python3
    save_map_node = ExecuteProcess(
        cmd=['python3', '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/save_map.py'],  # Explicitly run with python3
        output='screen',
        shell=True
    )

    # Return the launch description with all actions
    return LaunchDescription([
        save_map_node
    ])

