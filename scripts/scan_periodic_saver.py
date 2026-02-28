import csv
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import os
import pandas as pd
from io import StringIO
from rclpy.qos import QoSProfile

def main():
    rclpy.init()

    # Parameters
    pose_name = 'default_pose_scan'
    write_csv = True
    map_dir = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts'
    csv_file_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/transformed_data.csv'

    # Create the directory if it doesn't exist
    os.makedirs(map_dir, exist_ok=True)

    # Load transformed data CSV for frame_id and timestamp
    if not os.path.exists(csv_file_path):
        print(f"CSV file not found at: {csv_file_path}")
        return
    else:
        transformed_data = pd.read_csv(csv_file_path)
        print(f"Loaded transformed data from {csv_file_path}")

    # Setup for the output CSV
    csv_path = os.path.join(map_dir, f'{pose_name}.csv')
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['timestamp', 'tf_full', 'scan_full'])
    print(f"CSV file will be saved as {csv_path}")

    # Initialize rclpy and create a node for subscriptions
    node = rclpy.create_node('scan_pose_saver')

    # Variables to store latest pose and scan data
    latest_pose = None
    latest_scan = None

    # Callback functions to capture pose and scan data
    def pose_callback(msg):
        nonlocal latest_pose
        print("Received PoseStamped data.")
        
        timestamp = node.get_clock().now().to_msg()
        frame_id = transformed_data['frame_id'].iloc[0]  # Assume the frame_id is same for the entire dataset

        # Extract pose data
        pose_position_x = msg.pose.position.x
        pose_position_y = msg.pose.position.y
        pose_position_z = msg.pose.position.z
        pose_orientation_x = msg.pose.orientation.x
        pose_orientation_y = msg.pose.orientation.y
        pose_orientation_z = msg.pose.orientation.z
        pose_orientation_w = msg.pose.orientation.w

        # Store pose data in a dictionary to avoid the issue of list indexing
        latest_pose = {
            'timestamp': timestamp,
            'frame_id': frame_id,
            'pose_position': [pose_position_x, pose_position_y, pose_position_z],
            'pose_orientation': [pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w]
        }

    def scan_callback(msg):
        nonlocal latest_scan
        print("Received LaserScan data.")
        latest_scan = msg

    # Create subscriptions with QoS
    qos_profile = QoSProfile(depth=10)
    pose_subscriber = node.create_subscription(PoseStamped, '/current_pose', pose_callback, qos_profile)
    scan_subscriber = node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile)

    # Format the pose and scan data into a full string format
    def format_pose(pose_data):
        buf = StringIO()
        try:
            buf.write(f'header.stamp: {pose_data["timestamp"].sec}.{pose_data["timestamp"].nanosec}, ')
            buf.write(f'frame_id: {pose_data["frame_id"]}, ')
            buf.write(f'position: ({pose_data["pose_position"][0]}, {pose_data["pose_position"][1]}, {pose_data["pose_position"][2]}), ')
            buf.write(f'orientation: ({pose_data["pose_orientation"][0]}, {pose_data["pose_orientation"][1]}, {pose_data["pose_orientation"][2]}, {pose_data["pose_orientation"][3]})')
        except KeyError:
            buf.write("N/A")
        return buf.getvalue().strip()

    def format_scan(scan_msg):
        buf = StringIO()
        try:
            buf.write(f'header.stamp: {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}, ')
            buf.write(f'frame_id: {scan_msg.header.frame_id}, ')
            buf.write(f'angle_min: {scan_msg.angle_min}, angle_max: {scan_msg.angle_max}, ')
            buf.write(f'angle_increment: {scan_msg.angle_increment}, time_increment: {scan_msg.time_increment}, ')
            buf.write(f'scan_time: {scan_msg.scan_time}, range_min: {scan_msg.range_min}, range_max: {scan_msg.range_max}, ')
            buf.write(f'ranges: [{", ".join(map(str, scan_msg.ranges))}], ')
            buf.write(f'intensities: [{", ".join(map(str, scan_msg.intensities))}]')
        except AttributeError:
            buf.write("N/A")
        return buf.getvalue().strip()

    # Set a timer to periodically write the data to the CSV file
    def write_csv_snapshot():
        nonlocal latest_pose, latest_scan
        if latest_pose is None or latest_scan is None:
            print("Pose or Scan data not received yet. Skipping CSV writing.")
            return

        timestamp = latest_pose['timestamp']
        frame_id = latest_pose['frame_id']
        
        # Format the data
        pose_str = format_pose(latest_pose)
        scan_str = format_scan(latest_scan)

        # Write to CSV
        csv_writer.writerow([f'{timestamp.sec}.{timestamp.nanosec}', pose_str, scan_str])

        print('Pose and Scan data saved to CSV.')

    # Set a timer to write data periodically every 0.5 seconds
    timer = node.create_timer(0.5, write_csv_snapshot)

    try:
        print("Starting data collection... Press Ctrl+C to stop.")
        rclpy.spin(node)  # Wait and process data as long as rosbag is playing
    except KeyboardInterrupt:
        print("Keyboard Interrupt: Stopping.")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        csv_file.close()
        print(f"CSV file saved at {csv_path}")

if __name__ == '__main__':
    main()

