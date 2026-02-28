import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import math

# Define Constants
RW_ORIGIN_X = 0.0  # Real-world origin x in meters
RW_ORIGIN_Y = 0.0  # Real-world origin y in meters
RF_ORIGIN_X_PX = 1375  # Robot frame origin x in pixels
RF_ORIGIN_Y_PX = 1917  # Robot frame origin y in pixels
RESOLUTION = 0.02  # Resolution in meters per pixel

# Function to transform real-world coordinates to image coordinates
def real_to_image_coords(x_rw, y_rw):
    # Convert real-world coordinates (in meters) to image frame coordinates (in pixels)
    x_img = (x_rw - RW_ORIGIN_X) / RESOLUTION + RF_ORIGIN_X_PX
    y_img = (y_rw - RW_ORIGIN_Y) / RESOLUTION + RF_ORIGIN_Y_PX
    return x_img, y_img

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        
        # Open CSV file for writing data
        self.csv_file = open('robot_pose_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x_image', 'y_image', 'theta_image'])

    def pose_callback(self, msg):
        # Extract real-world pose data (x, y, theta)
        x_rw = msg.pose.position.x
        y_rw = msg.pose.position.y
        theta_rw = msg.pose.orientation.z  # Assuming theta is represented as a quaternion or angle
        
        # Convert the pose to image coordinates
        x_img, y_img = real_to_image_coords(x_rw, y_rw)
        
        # Store the pose and orientation in CSV
        self.csv_writer.writerow([x_img, y_img, theta_rw])
        self.get_logger().info(f"Processed pose: Image Coordinates: ({x_img}, {y_img}), Theta: {theta_rw}")

    def close_csv(self):
        # Close the CSV file when done
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)

    pose_listener = PoseListener()

    try:
        rclpy.spin(pose_listener)
    except KeyboardInterrupt:
        pass
    finally:
        pose_listener.close_csv()  # Close the CSV file when the program ends
        rclpy.shutdown()

if __name__ == '__main__':
    main()
