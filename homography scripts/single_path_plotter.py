import csv
import cv2
import numpy as np

# === INPUT PATHS ===
map_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
csv_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_glor.csv"
output_image_path = "fury_path_overlay_transformed.png"

# === DRAWING CONFIG ===
PATH_COLOR = (255, 0, 0)  # Red
POINT_RADIUS = 2
LINE_THICKNESS = 1

# === HOMOGRAPHY MATRIX ===
homography_matrix = np.array([
    [1.06697603e+00, 2.16688881e-02, 1.61954615e+02],
    [8.12237180e-03, 1.05393604e+00, 2.57667577e+02],
    [4.13900031e-05, -2.33481130e-07, 1.00000000e+00]
])

# === FUNCTION: READ CSV POINTS ===
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

# === FUNCTION: APPLY HOMOGRAPHY ===
def apply_homography(coords, H):
    pts = np.array(coords, dtype=np.float32).reshape(-1, 1, 2)
    warped = cv2.perspectiveTransform(pts, H)
    return [tuple(pt[0]) for pt in warped]

# === FUNCTION: DRAW PATH ===
def draw_path(img, points, color):
    for i in range(1, len(points)):
        pt1 = (int(points[i-1][0]), int(points[i-1][1]))
        pt2 = (int(points[i][0]), int(points[i][1]))
        cv2.line(img, pt1, pt2, color, LINE_THICKNESS)
        cv2.circle(img, pt2, POINT_RADIUS, color, -1)

# === MAIN EXECUTION ===
map_img = cv2.imread(map_path)
if map_img is None:
    raise FileNotFoundError(f"Failed to load map at: {map_path}")

# Read and transform the path
raw_coords = read_image_coords(csv_path)
transformed_coords = apply_homography(raw_coords, homography_matrix)

# Draw the transformed path
draw_path(map_img, transformed_coords, PATH_COLOR)

# Save the result
cv2.imwrite(output_image_path, map_img)
print(f"[INFO] Saved transformed path overlay to: {output_image_path}")

