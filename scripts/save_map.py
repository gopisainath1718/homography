import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import os

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')

        # Declare parameters
        self.map_save_folder = self.declare_parameter('map_save_folder', '/home/rainier/ros2_ws/src/littleslam_ros2/maps').value
        self.map_name = self.declare_parameter('map_name', 'my_map').value

        # Subscribe to the /icp_map topic (OccupancyGrid message)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/icp_map',  # Ensure that /icp_map topic is being published
            self.listener_callback,
            10  # QoS (Quality of Service)
        )

    def listener_callback(self, msg):
        # Path to save the .pgm and .yaml files
        pgm_file_path = os.path.join(self.map_save_folder, f"{self.map_name}.pgm")
        yaml_file_path = os.path.join(self.map_save_folder, f"{self.map_name}.yaml")

        # Assuming msg is the OccupancyGrid message
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Save the map using ros2 run map_server map_saver (equivalent of rosrun map_server map_saver)
        self.get_logger().info(f"Saving map as {pgm_file_path}")
        os.system(f"ros2 run map_server map_saver -f {pgm_file_path}")

        # Optionally, save the YAML configuration file for AMCL
        yaml_content = f"""image: {pgm_file_path}
resolution: {resolution}
origin: [{origin.position.x}, {origin.position.y}, {origin.position.z}]
neg: false
occupied_thresh: 0.65
free_thresh: 0.196
"""
        with open(yaml_file_path, 'w') as yaml_file:
            yaml_file.write(yaml_content)
        self.get_logger().info(f"Saved map yaml configuration as {yaml_file_path}")


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and spin the map saver node
    map_saver_node = MapSaverNode()
    rclpy.spin(map_saver_node)

    # Clean up and shut down
    map_saver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

