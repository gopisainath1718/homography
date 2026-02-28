import csv
import cv2
import numpy as np

# === INPUT PATHS ===
original_map_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"                 # Input 1
original_transformed_pose = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/transformed_data_alpha.csv"        # Input 2
new_transformed_pose = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_hugo.csv"              # Input 3
output_image_path = "comparison_overlay.png"

# === COLORS & DRAW CONFIG ===
COLOR_ORIGINAL = (0, 255, 0)      # Green
COLOR_NEW = (255, 0, 255)         # Purple
POINT_RADIUS = 2
LINE_THICKNESS = 2

# === HOMOGRAPHY MATRIX ===
homography_matrix = np.array([
    [1.06840364e+00, 3.21839577e-01, -2.52140304e+02],
    [-2.77042147e-01, 1.05529454e+00, 4.92894140e+02],
    [4.85998408e-05, 2.10318278e-05, 1.00000000e+00]
])

def read_image_coords(csv_file):
    coords = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row['x_image'])
                y = float(row['y_image'])
                coords.append((x, y))
            except:
                continue
    return coords

def apply_homography(coords, H):
    pts = np.array(coords, dtype=np.float32).reshape(-1, 1, 2)
    warped = cv2.perspectiveTransform(pts, H)
    return [tuple(pt[0]) for pt in warped]

def draw_path(img, points, color):
    for i in range(1, len(points)):
        pt1 = (int(points[i-1][0]), int(points[i-1][1]))
        pt2 = (int(points[i][0]), int(points[i][1]))
        cv2.line(img, pt1, pt2, color, LINE_THICKNESS)
        cv2.circle(img, pt2, POINT_RADIUS, color, -1)

# === MAIN EXECUTION ===
# Load map image
map_img = cv2.imread(original_map_path)

# Load both paths
original_path = read_image_coords(original_transformed_pose)
new_path = read_image_coords(new_transformed_pose)

# Apply homography to the new path
new_path_transformed = apply_homography(new_path, homography_matrix)

# Draw both paths
draw_path(map_img, original_path, COLOR_ORIGINAL)
draw_path(map_img, new_path_transformed, COLOR_NEW)

# Save final image
cv2.imwrite(output_image_path, map_img)
print(f"[INFO] Saved comparison image to: {output_image_path}")
