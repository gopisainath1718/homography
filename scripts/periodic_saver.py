import csv
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import SaveMap
from cartographer_ros_msgs.srv import WriteState as SaveState
from io import StringIO
import os

class PeriodicSaver(Node):
    def __init__(self):
        super().__init__('periodic_saver')
        self.declare_parameter('map_name', 'default_map')
        self.declare_parameter('write_csv', False)
        self.declare_parameter('map_dir', './')

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.write_csv = self.get_parameter('write_csv').get_parameter_value().bool_value
        self.map_dir = self.get_parameter('map_dir').get_parameter_value().string_value

        os.makedirs(self.map_dir, exist_ok=True)

        self.save_timer = self.create_timer(10.0, self.save_everything)

        if self.write_csv:
            self.latest_tf = None
            self.latest_scan = None

            self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

            csv_path = os.path.join(self.map_dir, f'{self.map_name}.csv')
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'tf_full', 'scan_full'])
            self.csv_timer = self.create_timer(0.5, self.write_csv_snapshot)

    def tf_callback(self, msg):
        self.latest_tf = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def write_csv_snapshot(self):
        if self.latest_tf is None or self.latest_scan is None:
            return

        timestamp = self.get_clock().now().to_msg()
        tf_str = self.format_tf(self.latest_tf)
        scan_str = self.format_scan(self.latest_scan)

        self.csv_writer.writerow([f'{timestamp.sec}.{timestamp.nanosec}', tf_str, scan_str])
        self.get_logger().info('Saved TF + Scan to CSV')

    def format_tf(self, tf_msg):
        buf = StringIO()
        for t in tf_msg.transforms:
            buf.write(f'header.stamp: {t.header.stamp.sec}.{t.header.stamp.nanosec}, ')
            buf.write(f'frame_id: {t.header.frame_id}, child_frame_id: {t.child_frame_id}, ')
            buf.write(f'translation: ({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}), ')
            buf.write(f'rotation: ({t.transform.rotation.x}, {t.transform.rotation.y}, {t.transform.rotation.z}, {t.transform.rotation.w})\n')
        return buf.getvalue().strip()

    def format_scan(self, scan_msg):
        buf = StringIO()
        buf.write(f'header.stamp: {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}, ')
        buf.write(f'frame_id: {scan_msg.header.frame_id}, ')
        buf.write(f'angle_min: {scan_msg.angle_min}, angle_max: {scan_msg.angle_max}, ')
        buf.write(f'angle_increment: {scan_msg.angle_increment}, time_increment: {scan_msg.time_increment}, ')
        buf.write(f'scan_time: {scan_msg.scan_time}, range_min: {scan_msg.range_min}, range_max: {scan_msg.range_max}, ')
        buf.write(f'ranges: [{", ".join(map(str, scan_msg.ranges))}], ')
        buf.write(f'intensities: [{", ".join(map(str, scan_msg.intensities))}]')
        return buf.getvalue().strip()

    def save_everything(self):
        self.get_logger().info(f'Saving map and state as: {self.map_name}')
        self.save_map()

    def save_map(self):
        self.map_client = self.create_client(SaveMap, '/map_saver/save_map')
        if not self.map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/map_saver/save_map not available')
            return

        request = SaveMap.Request()
        request.map_url = os.path.join(self.map_dir, self.map_name)
        request.image_format = 'pgm'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        future = self.map_client.call_async(request)
        future.add_done_callback(self._map_done_cb)

    def _map_done_cb(self, future):
        try:
            result = future.result()
            if result:
                self.get_logger().info('Map saved successfully')
            else:
                self.get_logger().error('Failed to save map')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')
        self.save_pbstream()

    def save_pbstream(self):
        self.pbstream_client = self.create_client(SaveState, '/write_state')
        if not self.pbstream_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/write_state service not available')
            return

        request = SaveState.Request()
        request.filename = os.path.join(self.map_dir, f'{self.map_name}.pbstream')
        self.get_logger().info(f"Requesting pbstream save to: {request.filename}")
        future = self.pbstream_client.call_async(request)
        future.add_done_callback(self._pbstream_done_cb)

    def _pbstream_done_cb(self, future):
        try:
            result = future.result()
            if result and result.status.code == 0:
                self.get_logger().info(f"pbstream saved successfully: {result.status.message}")
            else:
                self.get_logger().error(f"Failed to save pbstream: {result.status.message if result else 'no response'}")
        except Exception as e:
            self.get_logger().error(f"pbstream save failed: {e}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
                                                  

