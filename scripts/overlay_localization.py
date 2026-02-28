import json
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import math
import time

# Parameters
MAP_FILE = "/home/rainier/ros2_ws/src/littleslam_ros2/image_3.png"
RESOLUTION = 0.0367  # Meters per pixel (corrected)
ORIGIN_PX = 524  # Pixel x-coordinate of the real-world origin (0, 0) in map frame
ORIGIN_PY = 730  # Pixel y-coordinate of the real-world origin (0, 0) in map frame
JSON_FILE = "/home/rainier/ros2_ws/localization_data.json"
SCALE_FACTOR = 1.0  # Assume the data is already in meters

# Load the map image
map_img = Image.open(MAP_FILE).convert("RGB")
map_width, map_height = map_img.size
print(f"Map dimensions: {map_width}x{map_height}")

# Load the localization data
with open(JSON_FILE, "r") as f:
    data = json.load(f)

path_data = data["path"]
current_pose_data = data["current_pose"]

# Debug: Check the number of points in path_data and current_pose_data
print(f"Number of points in path_data: {len(path_data)}")
print(f"Number of points in current_pose_data: {len(current_pose_data)}")

# Ensure path_data and current_pose_data have the same length
if len(path_data) != len(current_pose_data):
    print("Warning: path_data and current_pose_data have different lengths. Truncating to the shorter length.")
    min_length = min(len(path_data), len(current_pose_data))
    path_data = path_data[:min_length]
    current_pose_data = current_pose_data[:min_length]

# Debug: Print the first few points in path_data
if len(path_data) > 0:
    print("First few points in path_data (before scaling):")
    for i in range(min(5, len(path_data))):
        print(f"Point {i}: x={path_data[i]['x']}, y={path_data[i]['y']}")
else:
    print("path_data is empty!")

# Debug: Print the last few points in path_data
if len(path_data) > 0:
    print("Last few points in path_data (before scaling):")
    for i in range(max(0, len(path_data) - 5), len(path_data)):
        print(f"Point {i}: x={path_data[i]['x']}, y={path_data[i]['y']}")

# Debug: Print the first few points in current_pose_data
if len(current_pose_data) > 0:
    print("First few points in current_pose_data (before scaling):")
    for i in range(min(5, len(current_pose_data))):
        print(f"Point {i}: x={current_pose_data[i]['x']}, y={current_pose_data[i]['y']}")
else:
    print("current_pose_data is empty!")

# Debug: Compare path_data and current_pose_data positions
for i in range(min(5, len(path_data))):
    path_pos = (path_data[i]["x"], path_data[i]["y"])
    pose_pos = (current_pose_data[i]["x"], current_pose_data[i]["y"])
    print(f"Index {i}: Path position = {path_pos}, Current pose position = {pose_pos}")

# Check the initial movement direction
if len(path_data) >= 2:
    dx = path_data[1]["x"] - path_data[0]["x"]
    dy = path_data[1]["y"] - path_data[0]["y"]
    print(f"Initial movement: dx={dx}, dy={dy}")
    if abs(dx) > abs(dy) and dx < 0:
        print("Initial movement is in the negative x-direction in the map frame, which maps to upwards in the image (correct).")
    else:
        print("Warning: Initial movement is not in the negative x-direction in the map frame. Expected negative x to move upwards in the image. Please re-record the rosbag with the correct trajectory.")

# Check the range of the path data (before scaling)
if path_data:
    x_values = [pose["x"] for pose in path_data]
    y_values = [pose["y"] for pose in path_data]
    print(f"x range (before scaling): {min(x_values)} to {max(x_values)}")
    print(f"y range (before scaling): {min(y_values)} to {max(y_values)}")
else:
    print("Cannot compute range: path_data is empty.")

# Scale the coordinates
for pose in path_data:
    pose["x"] /= SCALE_FACTOR
    pose["y"] /= SCALE_FACTOR
for pose in current_pose_data:
    pose["x"] /= SCALE_FACTOR
    pose["y"] /= SCALE_FACTOR

# Check the range after scaling
if path_data:
    x_values = [pose["x"] for pose in path_data]
    y_values = [pose["y"] for pose in path_data]
    print(f"x range (after scaling): {min(x_values)} to {max(x_values)}")
    print(f"y range (after scaling): {min(y_values)} to {max(y_values)}")

# Function to convert metric coordinates to pixel coordinates
# Start with 90-degree clockwise rotation and invert y-directionality
def metric_to_pixel(x, y):
    # 90-degree clockwise rotation: (x, y) -> (y, -x)
    # Invert y-direction: (y, -x) -> (y, x)
    x_image = y
    y_image = x
    # Convert to pixel coordinates
    px = ORIGIN_PX + (x_image / RESOLUTION)
    py = ORIGIN_PY + (y_image / RESOLUTION)
    # Clip the coordinates to the map boundaries
    px = max(0, min(map_width - 1, px))
    py = max(0, min(map_height - 1, py))
    return int(px), int(py)

# Function to convert quaternion to yaw angle (in radians)
def quaternion_to_yaw(qx, qy, qz, qw):
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    # Adjust yaw for the 90-degree clockwise rotation and inverted y-direction
    # 90-degree clockwise rotation: yaw -= pi/2
    # Inverting y-direction after rotation: additional 180-degree adjustment
    yaw -= math.pi / 2  # For 90-degree clockwise rotation
    yaw += math.pi      # For y-inversion
    # Normalize to [-pi, pi]
    if yaw > math.pi:
        yaw -= 2 * math.pi
    elif yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw

# Convert all path points to pixel coordinates
path_pixels = [metric_to_pixel(pose["x"], pose["y"]) for pose in path_data]
print("All pixel coordinates of the path:")
for i, (px, py) in enumerate(path_pixels):
    print(f"Point {i}: ({px}, {py})")

# Verify the starting point
if path_data:
    first_point_metric = (path_data[0]["x"], path_data[0]["y"])
    print(f"First point in metric coordinates: {first_point_metric}")
    if abs(first_point_metric[0]) > 0.1 or abs(first_point_metric[1]) > 0.1:
        print("Warning: First point in metric coordinates is not close to (0, 0). Adjusting origin.")
        # Adjust the origin based on the first point
        px_first, py_first = path_pixels[0]
        ORIGIN_PX -= (first_point_metric[0] / RESOLUTION)
        ORIGIN_PY -= (first_point_metric[1] / RESOLUTION)
        print(f"Adjusted ORIGIN_PX: {ORIGIN_PX}, ORIGIN_PY: {ORIGIN_PY}")
        # Recompute path pixels with the adjusted origin
        path_pixels = [metric_to_pixel(pose["x"], pose["y"]) for pose in path_data]

# Verify the starting point in pixel coordinates
if path_pixels:
    print(f"Starting point in pixel coordinates: {path_pixels[0]}")
    if abs(path_pixels[0][0] - 521) > 5 or abs(path_pixels[0][1] - 719) > 5:
        print("Warning: Starting point is not close to (521, 719). Please check the localization data.")

# Simulate localization by animating the path and current pose
plt.ion()  # Enable interactive mode for animation
fig, ax = plt.subplots()
ax.imshow(np.array(map_img))

# Counter for saving frames
frame_counter = 0

# List to store path points for incremental drawing
path_x = []
path_y = []

# Iterate through the path data and current pose data simultaneously
num_frames = min(len(path_data), len(current_pose_data))

for i in range(num_frames):
    # Clear the plot
    ax.clear()
    ax.imshow(np.array(map_img))

    # Draw the path incrementally up to the current index
    if i > 0:  # Need at least 2 points to draw a line
        print(f"Drawing path up to point {i}...")
        # Add the current point to the path
        px, py = path_pixels[i]
        path_x.append(px)
        path_y.append(py)
        # Draw the path with a thicker green line for better visibility
        ax.plot(path_x, path_y, color='limegreen', linewidth=3)

    # Draw the current pose as a red dot at the latest path point (to eliminate lag)
    px, py = path_pixels[i]  # Use the path position to ensure the red dot is at the end of the green line
    ax.plot(px, py, 'ro', markersize=10)  # Red dot

    # Draw an arrow for the orientation using the current_pose_data orientation
    pose = current_pose_data[i]
    yaw = quaternion_to_yaw(
        pose["orientation"]["x"],
        pose["orientation"]["y"],
        pose["orientation"]["z"],
        pose["orientation"]["w"]
    )
    arrow_length = 20  # Length of the arrow in pixels
    arrow_end_x = px + arrow_length * math.cos(yaw)
    arrow_end_y = py + arrow_length * math.sin(yaw)  # No need to flip y-axis for Matplotlib
    ax.arrow(px, py, arrow_end_x - px, arrow_end_y - py, color='red', head_width=5, head_length=10)

    # Update the plot
    plt.draw()

    # Save every 10th frame
    if frame_counter % 10 == 0:
        plt.savefig(f"frame_{frame_counter}.png")
        print(f"Saved frame_{frame_counter}.png")
    
    frame_counter += 1
    plt.pause(0.05)  # Pause to simulate real-time movement (adjust as needed)

plt.ioff()
plt.show()
