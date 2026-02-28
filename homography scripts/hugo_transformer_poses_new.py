import csv
import cv2
import numpy as np

# === CONFIGURATION ===
csv_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/collected_pose_points_hugo.csv"
map_image_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_hugo_new.png"
output_image_path = "map_with_path.png"
output_csv_path = "converted_pose_pixels.csv"
image_origin_px = (1340, 1044)  # This is the rotation pivot
meters_per_pixel = 0.02

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

def mirror_transform_and_rotate_ccw_poses(poses, origin_px, m_per_px):
    img_coords = []
    ox, oy = origin_px
    for x, y in poses:
        # Mirror in real-world coordinates
        x_m, y_m = -x, -y

        # Convert to pixel coordinates
        x_img = ox + int(x_m / m_per_px)
        y_img = oy - int(y_m / m_per_px)

        # 90-degree counterclockwise rotation around origin
        dx = x_img - ox
        dy = y_img - oy

        x_rot = ox - dy
        y_rot = oy + dx

        img_coords.append((int(x_rot), int(y_rot)))
    return img_coords

def save_pixel_coords_to_csv(img_coords, output_file):
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x_image', 'y_image'])
        for x, y in img_coords:
            writer.writerow([x, y])

def draw_path_on_image(img, path_points, color=(255, 0, 255), radius=1, thickness=1):
    for i in range(1, len(path_points)):
        cv2.line(img, path_points[i-1], path_points[i], color, thickness)
        cv2.circle(img, path_points[i], radius, color, -1)
    return img

# === MAIN EXECUTION ===
map_img = cv2.imread(map_image_path)
poses = read_pose_data(csv_path)
pose_pixels = mirror_transform_and_rotate_ccw_poses(poses, image_origin_px, meters_per_pixel)

save_pixel_coords_to_csv(pose_pixels, output_csv_path)
map_with_path = draw_path_on_image(map_img.copy(), pose_pixels)
cv2.imwrite(output_image_path, map_with_path)

print(f"[INFO] 90° CCW path overlay saved to: {output_image_path}")
print(f"[INFO] Transformed pixel coordinates saved to: {output_csv_path}")
