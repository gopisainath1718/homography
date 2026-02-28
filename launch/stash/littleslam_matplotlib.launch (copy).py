from launch import LaunchDescription
from launch_ros.actions import Node
import threading
import rclpy
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points


def start_matplotlib_visualization():
    """
    Starts the Matplotlib visualization in the main thread.
    """
    class MatplotlibVisualizer(RclpyNode):
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

            # Data containers
            self.map_points = None
            self.path_x = []
            self.path_y = []
            self.current_pose = None

        def map_callback(self, msg):
            # Convert PointCloud2 to numpy array of points (x, y)
            points = np.array([[p[0], p[1]] for p in read_points(msg, field_names=("x", "y"), skip_nans=True)])
            if points.size > 0:
                self.get_logger().info(f"Received {len(points)} map points.")
                self.map_points = points

        def path_callback(self, msg):
            # Extract path points (x, y) from Path message
            if len(msg.poses) > 0:
                self.path_x = [pose.pose.position.x for pose in msg.poses]
                self.path_y = [pose.pose.position.y for pose in msg.poses]
                self.get_logger().info(f"Received path with {len(msg.poses)} poses.")

        def pose_callback(self, msg):
            # Extract current pose (x, y)
            self.current_pose = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(f"Received current pose: x={self.current_pose[0]}, y={self.current_pose[1]}")

        def update_plot(self):
            while rclpy.ok():
                rclpy.spin_once(self)

                # Clear and re-plot data
                self.ax.clear()
                if self.map_points is not None and len(self.map_points) > 0:
                    self.ax.plot(self.map_points[:, 0], self.map_points[:, 1], 'b.', label="Map Points")
                else:
                    self.get_logger().warn("No map points to display.")

                if len(self.path_x) > 0 and len(self.path_y) > 0:
                    self.ax.plot(self.path_x, self.path_y, 'r-', label="Path")
                else:
                    self.get_logger().warn("No path to display.")

                if self.current_pose is not None:
                    # Plot current pose as a green dot
                    self.ax.plot(self.current_pose[0], self.current_pose[1], 'go', label="Current Pose")
                else:
                    self.get_logger().warn("No current pose to display.")

                # Dynamically adjust plot limits to fit all features
                all_x = []
                all_y = []
                if self.map_points is not None:
                    all_x.extend(self.map_points[:, 0])
                    all_y.extend(self.map_points[:, 1])
                all_x.extend(self.path_x)
                all_y.extend(self.path_y)
                if self.current_pose is not None:
                    all_x.append(self.current_pose[0])
                    all_y.append(self.current_pose[1])

                if len(all_x) > 0 and len(all_y) > 0:
                    margin = 1.0  # Add some margin around the data
                    min_x, max_x = min(all_x) - margin, max(all_x) + margin
                    min_y, max_y = min(all_y) - margin, max(all_y) + margin
                    self.ax.set_xlim(min_x, max_x)
                    self.ax.set_ylim(min_y, max_y)

                # Add legend and redraw plot
                self.ax.legend()
                plt.pause(0.01)

    def run_visualizer():
        rclpy.init()
        visualizer = MatplotlibVisualizer()
        try:
            plt.ion()  # Interactive mode on for dynamic updates
            plt.show(block=True)  # Ensure the plot stays open until manually closed
            visualizer.update_plot()
        except KeyboardInterrupt:
            pass
        finally:
            visualizer.destroy_node()
            rclpy.shutdown()

    # Start the visualization directly on the main thread for compatibility with Matplotlib
    run_visualizer()


def generate_launch_description():
    """
    Launches littleslam_ros2 node and starts Matplotlib-based visualization.
    """
    threading.Thread(target=start_matplotlib_visualization, daemon=True).start()

    return LaunchDescription([
        # Launch littleslam_ros2 node
        Node(
            package='littleslam_ros2',
            executable='littleslam_ros2',  # Correct executable name
            name='littleslam_ros2',
            output='screen',
            parameters=[
                {'use_odom': False}  # Adjust parameter as needed
            ]
        )
    ])

