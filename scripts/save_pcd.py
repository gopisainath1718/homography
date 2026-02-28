#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pcl

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.get_logger().info("Starting PointCloudSaver, waiting for /icp_map messages...")
        self.subscription = self.create_subscription(
            PointCloud2, '/icp_map', self.callback, 10)
        self.count = 0

    def callback(self, msg):
        # Convert ROS PointCloud2 to PCL
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    
        if not points:
            self.get_logger().warn(f"Skipped saving icp_map_{self.count}.pcd because it had no valid points.")
            return

        cloud = pcl.PointCloud()
        cloud.from_list(points)

        # Save to PCD file
        filename = f"icp_map_{self.count}.pcd"
        pcl.save(cloud, filename)
        self.get_logger().info(f"Saved {filename}")
        self.count += 1

def main():
    rclpy.init()
    node = PointCloudSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
