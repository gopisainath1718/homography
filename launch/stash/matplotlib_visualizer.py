#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points  # Utility for PointCloud2 conversion


class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')

        # Subscribers for map, path, and pose data
        self.create_subscription(PointCloud2, '/icp_map', self.map_callback, 10)
        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)

        # Initialize Matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Mapping and Localization")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")

        self.map_points = None
        self.path_x = []
        self.path_y = []
        self.current_pose = None

    def map_callback(self, msg):
        # Convert PointCloud2 to numpy array of points (x, y)
        points = np.array([[p[0], p[1]] for p in read_points(msg, field_names=("x", "y"), skip_nans=True)])
        self.map_points = points

    def path_callback(self, msg):
        # Extract path points (x, y) from Path message
        self.path_x = [pose.pose.position.x for pose in msg.poses]
        self.path_y = [pose.pose.position.y for pose in msg.poses]

    def pose_callback(self, msg):
        # Extract current pose (x, y)
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def update_plot(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            # Clear and re-plot data
            self.ax.clear()
            if self.map_points is not None:
                self.ax.plot(self.map_points[:, 0], self.map_points[:, 1], 'b.', label="Map Points")
            if len(self.path_x) > 0:
                self.ax.plot(self.path_x, self.path_y, 'r-', label="Path")
            if self.current_pose is not None:
                self.ax.plot(self.current_pose[0], self.current_pose[1], 'go', label="Current Pose")

            # Add legend and redraw plot
            self.ax.legend()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MatplotlibVisualizer()

    try:
        plt.ion()  # Interactive mode on for dynamic updates
        plt.show()
        node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

