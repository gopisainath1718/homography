import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np
import math

# ========================
# USER CONFIGURABLE PARAMS
# ========================
HOMOGRAPHY_MATRIX = np.array([
    [1.03794133e+00, 2.86485588e-01, -2.23076009e+02],
    [-2.56523391e-01, 9.95132542e-01, 4.91246002e+02],
    [3.89692639e-05, -8.17031905e-08, 1.00000000e+00]
])

ORIGIN_PX_X = 1080  # origin in pixels (X)
ORIGIN_PX_Y = 1802  # origin in pixels (Y)
RESOLUTION = 0.02   # meters per pixel

OUTPUT_CSV = 'transformed_pose_data.csv'


# ================
# TRANSFORM SCRIPT
# ================
class HomographyPoseLogger(Node):
    def __init__(self):
        super().__init__('homography_pose_logger')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )

        self.csv_file = open(OUTPUT_CSV, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x_pixel', 'y_pixel', 'theta_rad'])

    def pose_callback(self, msg):
        x_rw = msg.pose.position.x
        y_rw = msg.pose.position.y
        theta = self.get_yaw_from_quaternion(msg.pose.orientation)

        # Apply homography to real-world pose
        point_rw = np.array([x_rw, y_rw, 1.0])
        transformed = HOMOGRAPHY_MATRIX @ point_rw
        transformed /= transformed[2]

        # Convert to image pixel coordinates
        x_img = transformed[0] / RESOLUTION + ORIGIN_PX_X
        y_img = transformed[1] / RESOLUTION + ORIGIN_PX_Y

        # Write to CSV
        self.csv_writer.writerow([x_img, y_img, theta])
        self.get_logger().info(f'Pose: ({x_img:.2f}, {y_img:.2f}), θ = {theta:.3f} rad')

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def close_csv(self):
        self.csv_file.close()


# ============
# ENTRY POINT
# ============
def main():
    rclpy.init()
    node = HomographyPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
