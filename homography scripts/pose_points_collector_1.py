import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pathlib import Path
import csv

CSV_FILENAME = Path(__file__).parent / "collected_pose_points_glor.csv"

class PoseCollector(Node):
    def __init__(self):
        super().__init__('pose_collector')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        self.pose_data = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.pose_data.append([x, y, z, qx, qy, qz, qw])
        self.get_logger().info(f"Received pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def save_to_csv(self):
        try:
            with open(CSV_FILENAME, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                writer.writerows(self.pose_data)
            print(f"[✅] Saved {len(self.pose_data)} poses to: {CSV_FILENAME}")
        except Exception as e:
            print(f"[❌] Failed to save CSV: {e}")

def main():
    rclpy.init()
    node = PoseCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[⚠️] Keyboard interrupt received. Stopping node...")
        node.save_to_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

