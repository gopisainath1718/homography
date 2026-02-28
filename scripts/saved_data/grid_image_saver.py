import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import csv
import os
from time import time, sleep

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Initialize the CvBridge for converting ROS images to OpenCV images
        self.bridge = CvBridge()
        
        # Subscribe to the /camera/color/image_raw topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Set up CSV reading for transformed data
        self.csv_data = self.load_transformed_data('/home/rainier/ros2_ws/src/littleslam_ros2/scripts/transformed_data.csv')

        # Prepare the save folder for images
        self.save_folder = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/saved_data'
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        # Initialize time tracking
        self.last_save_time = time()

    def load_transformed_data(self, csv_file_path):
        # Load the CSV file with transformed pose data
        data = []
        with open(csv_file_path, mode='r') as infile:
            csv_reader = csv.reader(infile)
            header = next(csv_reader)  # Skip the header
            
            for row in csv_reader:
                # Store timestamp, x_image, and y_image
                timestamp = float(row[0])
                x_image = float(row[2])
                y_image = float(row[3])
                data.append((timestamp, x_image, y_image))
        
        return data

    def image_callback(self, msg):
        # Get current timestamp
        current_time = time()

        # Only save an image every 1 second
        if current_time - self.last_save_time >= 1:
            self.last_save_time = current_time  # Update last save time

            # Get the timestamp from the camera image message
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Find the closest timestamp in the transformed pose data
            closest_data = self.get_closest_pose_data(timestamp)

            if closest_data:
                x_image, y_image = closest_data

                # Round x_image and y_image to the nearest multiples of 20
                x_lower = (x_image // 20) * 20
                x_upper = x_lower + 20
                y_lower = (y_image // 20) * 20
                y_upper = y_lower + 20

                # Format the filename as requested
                filename = f"grid_{int(x_lower)}_{int(x_upper)}_{int(y_lower)}_{int(y_upper)}.png"

                # Convert the ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Save the image with the appropriate filename
                save_path = os.path.join(self.save_folder, filename)
                cv2.imwrite(save_path, cv_image)

                self.get_logger().info(f"Saved image: {filename}")

    def get_closest_pose_data(self, timestamp):
        # Find the closest timestamp in the transformed data
        closest_time_diff = float('inf')
        closest_data = None
        
        for data in self.csv_data:
            time_diff = abs(data[0] - timestamp)
            if time_diff < closest_time_diff:
                closest_time_diff = time_diff
                closest_data = data[1:]  # Return x_image, y_image

        return closest_data

def main(args=None):
    rclpy.init(args=args)

    image_saver = ImageSaver()

    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
