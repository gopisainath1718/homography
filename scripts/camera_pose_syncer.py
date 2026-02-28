import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from realsense2_camera_msgs.msg import Metadata  # Correct message import for camera data
import csv
from collections import deque

class DataListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        
        # Subscriber for /current_pose topic
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )

        # Subscriber for /camera/color/metadata topic (correct message type)
        self.camera_subscription = self.create_subscription(
            Metadata,
            '/camera/color/metadata',
            self.camera_callback,
            10
        )
        
        # Queue to store pose data in chronological order
        self.pose_data_queue = deque()

        # Open CSV file for writing data (overwrite the file if it exists)
        self.csv_file = open('camera_pose_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp (sec)', 'frame_id', 'x', 'y', 'theta'])
        self.get_logger().info("CSV file opened for writing")

    def pose_callback(self, msg):
        # Store current pose data without any conversion
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Convert to seconds
        pose_data = {
            'timestamp': timestamp,  # Store the timestamp here
            'frame_id': msg.header.frame_id,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': msg.pose.orientation.z  # Assuming theta is represented as a quaternion or angle
        }
        self.pose_data_queue.append(pose_data)
        self.get_logger().info(f"Pose added to queue: {pose_data}")

    def camera_callback(self, msg):
        # Access timestamp from the camera metadata (using header.stamp)
        camera_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame_id = msg.header.frame_id
        
        # Log the received camera timestamp
        self.get_logger().info(f"Received camera data at timestamp {camera_timestamp}")
        
        # Use the camera timestamp and take the latest pose data from the queue
        closest_pose = None
        if self.pose_data_queue:
            # Get the most recent pose from the queue (use the most recent pose available)
            closest_pose = self.pose_data_queue[-1]  # Just grab the latest pose

        if closest_pose:
            # Write the data to CSV with the camera timestamp
            self.csv_writer.writerow([camera_timestamp, frame_id, closest_pose['x'], closest_pose['y'], closest_pose['theta']])
            self.csv_file.flush()  # Ensure data is written immediately
            self.get_logger().info(f"Camera timestamp {camera_timestamp} matched with pose data")

    def close_csv(self):
        # Ensure the CSV file is flushed and closed properly on shutdown
        self.csv_file.flush()  # Flush data to disk
        self.csv_file.close()  # Close the CSV file
        self.get_logger().info("CSV file closed")

def main(args=None):
    rclpy.init(args=args)

    data_listener = DataListener()

    try:
        rclpy.spin(data_listener)
    except KeyboardInterrupt:
        pass
    finally:
        data_listener.close_csv()  # Ensure CSV file is closed properly
        # Only call shutdown once, after the CSV file is properly closed
        rclpy.shutdown()

if __name__ == '__main__':
    main()

