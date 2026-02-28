import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import struct

# Function to convert PointCloud2 to numpy array
def point_cloud2_to_numpy(cloud_msg):
    # Extract the fields and their offsets from the PointCloud2 message
    fields = cloud_msg.fields
    point_step = cloud_msg.point_step
    offset_x = next((field.offset for field in fields if field.name == 'x'), None)
    offset_y = next((field.offset for field in fields if field.name == 'y'), None)
    offset_z = next((field.offset for field in fields if field.name == 'z'), None)

    if offset_x is None or offset_y is None or offset_z is None:
        raise ValueError("PointCloud2 message does not contain 'x', 'y', and 'z' fields.")
    
    # Calculate the number of points (based on data size and point step)
    num_points = len(cloud_msg.data) // point_step
    
    # Prepare numpy array to hold the x, y, z points
    points = np.zeros((num_points, 3), dtype=np.float32)

    # Unpack the point cloud data
    for i in range(num_points):
        # Unpack the bytes based on the offsets and point_step
        data = cloud_msg.data[i * point_step:(i + 1) * point_step]
        
        # Extract x, y, z from the data
        x = struct.unpack_from('f', data, offset_x)[0]
        y = struct.unpack_from('f', data, offset_y)[0]
        z = struct.unpack_from('f', data, offset_z)[0]
        
        points[i] = [x, y, z]

    return points

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')

        # Create subscribers for each topic
        self.icp_map_sub = self.create_subscription(
            PointCloud2,
            '/icp_map',
            self.icp_map_callback,
            10
        )
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            10
        )
        self.path_sub = self.create_subscription(
            PoseStamped,
            '/path',
            self.path_callback,
            10
        )

        # Initialize the plot with fixed axis
        self.fig, self.ax = plt.subplots()
        self.lidar_path = []
        self.lidar_poses = []
        self.point_cloud_data = []

        # Fixed axis limits for better visualization
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        # Store plot data
        self.scatter_plot = None
        self.path_plot = None

    def icp_map_callback(self, msg: PointCloud2):
        try:
            # Convert point cloud data from PointCloud2 to a numpy array
            self.point_cloud_data = point_cloud2_to_numpy(msg)

            # Filter points based on distance to remove outliers (limiting to 10m range)
            max_distance = 10
            filtered_data = self.point_cloud_data[(np.sqrt(self.point_cloud_data[:, 0]**2 + self.point_cloud_data[:, 1]**2)) < max_distance]

            # Update the point cloud data on the plot
            if self.scatter_plot is None:
                self.scatter_plot = self.ax.scatter(filtered_data[:, 0], filtered_data[:, 1], c='k', s=1)  # Black points
            else:
                self.scatter_plot.set_offsets(np.c_[filtered_data[:, 0], filtered_data[:, 1]])

            # Update the pose and path
            self.plot_current_pose()
            self.plot_lidar_path()

            # Update the plot
            plt.pause(0.01)

        except ValueError as e:
            self.get_logger().error(f"Error processing PointCloud2 message: {e}")

    def current_pose_callback(self, msg: PoseStamped):
        # Extract the current pose and update the lidar_poses list
        position = msg.pose.position
        self.lidar_poses = [(position.x, position.y)]

    def path_callback(self, msg: PoseStamped):
        # Extract the path and update the lidar_path list
        position = msg.pose.position
        self.lidar_path.append((position.x, position.y))

    def plot_lidar_path(self):
        # Plot the lidar's travel path
        path_array = np.array(self.lidar_path)
        if len(path_array) > 1:
            if self.path_plot is None:
                self.path_plot, = self.ax.plot(path_array[:, 0], path_array[:, 1], label="Path", color='g')
            else:
                self.path_plot.set_data(path_array[:, 0], path_array[:, 1])

    def plot_current_pose(self):
        # Plot the current pose of the lidar
        if self.lidar_poses:
            current_pose = np.array(self.lidar_poses)
            self.ax.scatter(current_pose[0][0], current_pose[0][1], color='r', label="Current Pose", zorder=5)

def main(args=None):
    rclpy.init(args=args)

    lidar_visualizer = LidarVisualizer()

    try:
        rclpy.spin(lidar_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

