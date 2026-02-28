import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# === CONFIGURATION ===
csv_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/collected_pose_points_glor.csv"
map_image_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_glor_new.png"
output_csv_path = "converted_pose_pixels.csv"
output_image_path = "map_with_path.png"

# Image origin in pixels and resolution
origin_px = (1148, 1657)  # (x, y) in image pixels
meters_per_pixel = 0.02   # 1 pixel = 0.02 meters

# === FUNCTION TO READ REAL-WORLD POSE DATA ===
def read_pose_data(csv_file):
    poses = []
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            try:
                x = float(row['x'])
                y = float(row['y'])
                poses.append((x, y))
            except:
                continue
    return poses

# === FUNCTION TO CONVERT REAL-WORLD TO IMAGE PIXELS ===
def convert_to_pixel_coords(poses, origin_px, meters_per_pixel):
    origin_x, origin_y = origin_px
    pixel_coords = []
    for x_m, y_m in poses:
        x_px = origin_x + int(x_m / meters_per_pixel)
        y_px = origin_y - int(y_m / meters_per_pixel)
        pixel_coords.append((x_px, y_px))
    return pixel_coords

# === FUNCTION TO SAVE PIXEL COORDINATES TO CSV ===
def save_to_csv(pixel_coords, output_file):
    with open(output_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x_image', 'y_image'])
        writer.writerows(pixel_coords)

# === FUNCTION TO PLOT PATH ON MAP IMAGE ===
def plot_path_on_map(image_path, pixel_coords, output_path):
    img = mpimg.imread(image_path)
    x_vals, y_vals = zip(*pixel_coords)

    plt.figure(figsize=(10, 10))
    plt.imshow(img)
    plt.plot(x_vals, y_vals, marker='o', linestyle='-', linewidth=1, markersize=1, color='red')
    plt.title("Robot Path on Map")
    plt.axis('off')
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()

# === MAIN PIPELINE ===
poses = read_pose_data(csv_path)
pixel_coords = convert_to_pixel_coords(poses, origin_px, meters_per_pixel)
save_to_csv(pixel_coords, output_csv_path)
plot_path_on_map(map_image_path, pixel_coords, output_image_path)

print(f"Converted pixel coordinates saved to {output_csv_path}")
print(f"Path plotted and saved as {output_image_path}")

