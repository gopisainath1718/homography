#!/usr/bin/env python3
"""
littleslam_matplotlib_ros2.py - Combined ROS2 Launch & Visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points
import threading
import matplotlib.animation as animation

class SlamVisualizer(Node):
    def __init__(self):
        super().__init__('slam_visualizer')
        self.lock = threading.Lock()
        
        # Initialize data stores
        self.map_points = None
        self.current_pose = None
        self.path = []
        self.scan = None

        # Setup subscribers
        self.create_subscription(PointCloud2, '/icp_map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_cb, 10)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Initialize matplotlib
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_title("ROS2 SLAM Visualization")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        
        # Start animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, 
            interval=50, blit=False
        )

    def map_cb(self, msg):
        with self.lock:
            self.map_points = np.array([[p[0], p[1]] 
                for p in read_points(msg, field_names=("x", "y"), skip_nans=True)])

    def pose_cb(self, msg):
        with self.lock:
            self.current_pose = msg.pose

    def path_cb(self, msg):
        with self.lock:
            self.path = [(p.pose.position.x, p.pose.position.y) 
                       for p in msg.poses]

    def scan_cb(self, msg):
        with self.lock:
            angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
            ranges = np.array(msg.ranges)
            valid = (ranges > msg.range_min) & (ranges < msg.range_max)
            self.scan = {
                'angles': angles[valid],
                'ranges': ranges[valid]
            }

    def update_plot(self, frame):
        with self.lock:
            self.ax.clear()
            
            # Plot map points
            if self.map_points is not None:
                self.ax.plot(self.map_points[:, 0], self.map_points[:, 1], 
                           'b.', markersize=1, alpha=0.5, label='Map')
            
            # Plot path
            if self.path:
                path_x = [p[0] for p in self.path]
                path_y = [p[1] for p in self.path]
                self.ax.plot(path_x, path_y, 'r-', linewidth=1.5, label='Path')
            
            # Plot current pose and scan
            if self.current_pose:
                x = self.current_pose.position.x
                y = self.current_pose.position.y
                q = self.current_pose.orientation
                yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
                
                # Draw pose arrow
                dx = 0.5 * np.cos(yaw)
                dy = 0.5 * np.sin(yaw)
                self.ax.arrow(x, y, dx, dy, head_width=0.3, 
                            head_length=0.4, fc='green', ec='green')
                
                # Plot scan points
                if self.scan:
                    scan_x = x + self.scan['ranges'] * np.cos(yaw + self.scan['angles'])
                    scan_y = y + self.scan['ranges'] * np.sin(yaw + self.scan['angles'])
                    self.ax.scatter(scan_x, scan_y, c='orange', s=2, alpha=0.6, label='Scan')

            self.ax.legend()
            self.ax.set_aspect('equal')
            
        return self.ax,

def generate_launch_description():
    return LaunchDescription([
        # Launch littleslam_ros2 node
        Node(
            package='littleslam_ros2',
            executable='littleslam_ros2',
            name='slam_node',
            output='screen',
            parameters=[{'use_odom': False}]
        ),
        
        # Launch visualization node
        ExecuteProcess(
            cmd=['ros2', 'run', 'your_package', 'littleslam_matplotlib_ros2'],
            shell=True
        )
    ])

def main(args=None):
    rclpy.init(args=args)
    visualizer = SlamVisualizer()
    try:
        plt.ion()
        plt.show()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

