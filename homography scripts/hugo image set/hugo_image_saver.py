import csv
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

INPUT_CSV = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_hugo.csv"
OUTPUT_CSV = "path_with_images.csv"
OUTPUT_IMG_DIR = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set"
MAP_IMAGE_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
IMAGE_TOPIC = "/camera/color/image_raw"
SAVE_INTERVAL = 1.0  # seconds

os.makedirs(OUTPUT_IMG_DIR, exist_ok=True)

def rotate_90_clockwise(x, y, image_width, image_height):
    cx, cy = image_width / 2, image_height / 2
    dx, dy = x - cx, y - cy
    new_x = cx + dy
    new_y = cy - dx
    return int(round(new_x)), int(round(new_y))

def load_and_transform_points(input_csv, img_width=1920, img_height=1080):
    points = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        print("[DEBUG] CSV headers:", reader.fieldnames)
        for row in reader:
            print("[DEBUG] Row:", row)
            try:
                x = float(row['x_image'])
                y = float(row['y_image'])
                x_t, y_t = rotate_90_clockwise(x, y, img_width, img_height)
                points.append((x_t, y_t))
            except Exception as e:
                print("[ERROR] Skipping row:", row, "| Error:", e)
                continue
    print(f"[DEBUG] Loaded {len(points)} points after rotation.")
    return points

class ImageSaver(Node):
    def __init__(self, transformed_points):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, 10)
        self.last_saved_ros_time = None
        self.frame_count = 0
        self.transformed_points = transformed_points
        self.saved_data = []

        self.get_logger().info("=== ImageSaver Node started ===")
        self.get_logger().info(f"Waiting for images on topic: {IMAGE_TOPIC}")
        self.get_logger().info(f"Expected total points: {len(transformed_points)}")

    def image_callback(self, msg: Image):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_saved_ros_time is None:
            self.last_saved_ros_time = stamp
            self.save_image(msg)
            return

        if stamp - self.last_saved_ros_time >= SAVE_INTERVAL:
            self.last_saved_ros_time = stamp
            self.save_image(msg)

    def save_image(self, msg: Image):
        if self.frame_count < len(self.transformed_points):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img_filename = f"frame_{self.frame_count:04d}.png"
                img_path = os.path.join(OUTPUT_IMG_DIR, img_filename)
                cv2.imwrite(img_path, cv_image)

                x, y = self.transformed_points[self.frame_count]
                self.saved_data.append({
                    "x": x,
                    "y": y,
                    "image_filename": img_filename
                })

                self.get_logger().info(f"Saved image {img_filename} at ({x},{y}) [Frame {self.frame_count}]")
                self.frame_count += 1
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")
        else:
            self.get_logger().info("✅ All points captured, saving CSV and shutting down...")
            self._save_output_csv()
            rclpy.shutdown()

    def _save_output_csv(self):
        with open(OUTPUT_CSV, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=["x", "y", "image_filename"])
            writer.writeheader()
            for row in self.saved_data:
                writer.writerow(row)
        self.get_logger().info(f"📂 CSV saved at: {OUTPUT_CSV}")

def visualize_path(output_csv, map_img_path, image_folder):
    map_img = mpimg.imread(map_img_path)
    map_img_rotated = cv2.rotate(map_img, cv2.ROTATE_90_CLOCKWISE)

    points = []
    image_files = []
    with open(output_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row['x']), float(row['y'])))
            image_files.append(row['image_filename'])

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(map_img_rotated)
    scatter = ax.scatter([p[0] for p in points], [p[1] for p in points], c='red', s=10)

    annot = AnnotationBbox(OffsetImage(cv2.cvtColor(cv2.imread(os.path.join(image_folder, image_files[0])), cv2.COLOR_BGR2RGB), zoom=0.2), (0,0), frameon=True, pad=0.5)
    annot.set_visible(False)
    ax.add_artist(annot)

    def update_annot(ind):
        idx = ind["ind"][0]
        pos = points[idx]
        img = cv2.cvtColor(cv2.imread(os.path.join(image_folder, image_files[idx])), cv2.COLOR_BGR2RGB)
        annot.xy = pos
        annot.set_child(OffsetImage(img, zoom=0.3))
        annot.set_visible(True)

    def hover(event):
        vis = annot.get_visible()
        if event.inaxes == ax:
            cont, ind = scatter.contains(event)
            if cont:
                update_annot(ind)
                fig.canvas.draw_idle()
            elif vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", hover)
    plt.title("Hover over path points to see images")
    plt.show()

def main():
    rclpy.init()
    points = load_and_transform_points(INPUT_CSV, img_width=1920, img_height=1080)
    node = ImageSaver(points)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt - exiting early")
        if node.frame_count > 0:
            node._save_output_csv()
        rclpy.shutdown()

    visualize_path(OUTPUT_CSV, MAP_IMAGE_PATH, OUTPUT_IMG_DIR)

if __name__ == '__main__':
    main()

