import csv
import cv2
import numpy as np

# === INPUT PATHS ===
map_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
alpha_pose_path = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/transformed_data_alpha.csv"
fury_pose_path  = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_fury.csv"
hugo_pose_path  = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_hugo.csv"
glor_pose_path  = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_glor.csv"
output_path     = "quadruple_path_overlay.png"

# === COLORS ===
COLOR_ALPHA = (0, 255, 0)      # Green
COLOR_FURY  = (0, 0, 180)      # Soft Red
COLOR_HUGO  = (255, 0, 0)      # Blue
COLOR_GLOR  = (211, 0, 255)    # Violet

POINT_RADIUS = 2
LINE_THICKNESS = 2
FURY_LINE_THICKNESS = 1

# === HOMOGRAPHY MATRICES ===
H_FURY = np.array([
    [1.06840364e+00, 3.21839577e-01, -2.52140304e+02],
    [-2.77042147e-01, 1.05529454e+00, 4.92894140e+02],
    [4.85998408e-05, 2.10318278e-05, 1.00000000e+00]
])
H_HUGO = np.array([
    [1.01040424e+00, 1.08608544e-01, -8.50754558e+01],
    [-5.86196700e-02, 1.00630386e+00, 3.76928541e+02],
    [1.69049448e-05, -2.14893424e-06, 1.00000000e+00]
])
H_GLOR = np.array([
    [1.06697603e+00, 2.16688881e-02, 1.61954615e+02],
    [8.12237180e-03, 1.05393604e+00, 2.57667577e+02],
    [4.13900031e-05, -2.33481130e-07, 1.00000000e+00]
])

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

# === MAIN EXECUTION ===
map_img = cv2.imread(map_path)

# Load all raw paths
alpha_path = read_coords(alpha_pose_path)
fury_raw   = read_coords(fury_pose_path)
hugo_raw   = read_coords(hugo_pose_path)
glor_raw   = read_coords(glor_pose_path)

# Apply homographies to map each onto Alpha frame
fury_path = apply_homography(fury_raw, H_FURY)
hugo_path = apply_homography(hugo_raw, H_HUGO)
glor_path = apply_homography(glor_raw, H_GLOR)

# Draw all paths
draw_path(map_img, alpha_path, COLOR_ALPHA)
draw_path(map_img, fury_path, COLOR_FURY, thickness=FURY_LINE_THICKNESS, draw_circles=False)
draw_path(map_img, hugo_path, COLOR_HUGO)
draw_path(map_img, glor_path, COLOR_GLOR)

# Save final output
cv2.imwrite(output_path, map_img)
print(f"[✅] Saved 4-path overlay image to: {output_path}")
