import csv
import cv2
import numpy as np

# === CONFIG ===
map_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
pose_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_fury.csv"
output_path = "single_path_overlay.png"

# If needed, set homography matrix (or set to None if not used)
USE_HOMOGRAPHY = False
H = np.array([
    [1.06840364e+00, 3.21839577e-01, -2.52140304e+02],
    [-2.77042147e-01, 1.05529454e+00, 4.92894140e+02],
    [4.85998408e-05, 2.10318278e-05, 1.00000000e+00]
])

# === VISUAL PARAMETERS ===
COLOR_PATH = (0, 0, 255)  # Red
POINT_RADIUS = 2
LINE_THICKNESS = 2

# === FUNCTIONS ===
def read_coords(csv_file):
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

def draw_path(img, points, color, thickness=LINE_THICKNESS, draw_circles=True):
    pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [pts], isClosed=False, color=color, thickness=thickness, lineType=cv2.LINE_AA)
    if draw_circles:
        for pt in pts:
            cv2.circle(img, tuple(pt[0]), POINT_RADIUS, color, -1)

# === MAIN ===
map_img = cv2.imread(map_path)
pose_points = read_coords(pose_path)

if USE_HOMOGRAPHY:
    pose_points = apply_homography(pose_points, H)

draw_path(map_img, pose_points, COLOR_PATH)
cv2.imwrite(output_path, map_img)
print(f"[✅] Saved single overlay image to: {output_path}")

