import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

# === CONFIGURATION ===
csv_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/collected_pose_points_glor.csv"
map_image_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_glor_new.png"
output_image_path = "map_with_path.png"
output_csv_path = "converted_pose_pixels.csv"
image_origin_px = (1148, 1657)  # Change this to your actual image origin in pixels
meters_per_pixel = 0.02  # Adjust this based on your map resolution (e.g., 5cm/pixel)

# === FUNCTION DEFINITIONS ===
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

def mirror_and_transform_to_image_coords(poses, origin_px, m_per_px):
    img_coords = []
    for x, y in poses:
        # Mirror across X and Y axes
        x_mirrored, y_mirrored = -x, -y

        # Convert to image pixels (RViz to image convention)
        x_img = origin_px[0] + int(x_mirrored / m_per_px)
        y_img = origin_px[1] - int(y_mirrored / m_per_px)
        img_coords.append((x_img, y_img))
    return img_coords

def save_pixel_coords_to_csv(img_coords, output_file):
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x_image', 'y_image'])  # header
        for x, y in img_coords:
            writer.writerow([x, y])

def draw_path_on_image(img, path_points, color=(255, 0, 255), radius=1, thickness=1):
    for i in range(1, len(path_points)):
        cv2.line(img, path_points[i-1], path_points[i], color, thickness)
        cv2.circle(img, path_points[i], radius, color, -1)
    return img

# === MAIN EXECUTION ===
# Load image
map_img = cv2.imread(map_image_path)

# Process poses
poses = read_pose_data(csv_path)
pose_pixels = mirror_and_transform_to_image_coords(poses, image_origin_px, meters_per_pixel)

# Save converted pixel coordinates to CSV
save_pixel_coords_to_csv(pose_pixels, output_csv_path)

# Draw and save image
map_with_path = draw_path_on_image(map_img.copy(), pose_pixels)
cv2.imwrite(output_image_path, map_with_path)

print(f"[INFO] Path overlay saved to: {output_image_path}")
print(f"[INFO] Transformed pixel coordinates saved to: {output_csv_path}")
