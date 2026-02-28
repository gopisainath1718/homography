import pcl
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, binary_erosion, binary_fill_holes
import os

# Load and merge PCD files
pcd_dir = "/home/rainier/ros2_ws/src/littleslam_ros2/icp data/"
all_points = []

for i in range(891):  # From icp_map_0.pcd to icp_map_905.pcd #changed for second run
    pcd_file = os.path.join(pcd_dir, f"icp_map_{i}.pcd")
    if os.path.exists(pcd_file):
        cloud = pcl.load(pcd_file)
        points = cloud.to_array()
        # Basic filtering: Remove points with z > 1.0 or z < -1.0 (adjust as needed)
        mask = (points[:, 2] > -1.0) & (points[:, 2] < 1.0)
        points = points[mask]
        all_points.append(points)

# Concatenate all points into a single array
if all_points:
    merged_points = np.concatenate(all_points, axis=0)
    merged_cloud = pcl.PointCloud()
    merged_cloud.from_array(merged_points)
else:
    raise ValueError("No PCD files were loaded!")

# Extract the points (XYZ)
points = merged_cloud.to_array()
print("Number of points in merged PCD:", points.shape[0])
print("Sample points:", points[:5])

# Print the min and max values for x and y
min_x, min_y = points[:, 0].min(), points[:, 1].min()
max_x, max_y = points[:, 0].max(), points[:, 1].max()
print("Min X:", min_x, "Max X:", max_x)
print("Min Y:", min_y, "Max Y:", max_y)

# Define resolution (meters per pixel)
resolution = 0.02  # Finer resolution: 2 cm per pixel

# Shift points to positive coordinates
points_shifted = points[:, :2] - [min_x, min_y]
print("Shifted points sample:", points_shifted[:5])

# Convert to grid coordinates
grid = (points_shifted / resolution).astype(int)
print("Grid points sample:", grid[:5])
max_x_grid, max_y_grid = grid.max(axis=0)
print("Grid max X:", max_x_grid, "Grid max Y:", max_y_grid)

# Create occupancy grid (0: free, 100: occupied, -1: unknown)
occupancy_grid = np.ones((max_y_grid + 1, max_x_grid + 1), dtype=np.int8) * -1  # Unknown
print("Occupancy grid shape:", occupancy_grid.shape)

# Populate the grid
for x, y in grid:
    if 0 <= x < occupancy_grid.shape[1] and 0 <= y < occupancy_grid.shape[0]:
        occupancy_grid[y, x] = 100  # Occupied

# Post-process the grid
# Step 1: Dilate to connect nearby occupied cells (reduce gaps)
occupied_mask = (occupancy_grid == 100)
occupied_mask = binary_dilation(occupied_mask, iterations=2)

# Step 2: Fill enclosed areas (assume they are free space)
filled_mask = binary_fill_holes(occupied_mask)
free_mask = filled_mask & (occupancy_grid != 100)
occupancy_grid[free_mask] = 0  # Mark as free

# Step 3: Erode to remove small noise
occupied_mask = binary_erosion(occupied_mask, iterations=1)
occupancy_grid = np.where(occupied_mask, 100, occupancy_grid)

# Step 4: Mark remaining unknown areas
occupancy_grid[occupancy_grid == -1] = -1  # Ensure unknown areas are preserved

# Crop the grid to the bounding box of occupied cells
occupied_indices = np.where(occupancy_grid == 100)
if len(occupied_indices[0]) > 0:
    min_y, max_y = occupied_indices[0].min(), occupied_indices[0].max()
    min_x, max_x = occupied_indices[1].min(), occupied_indices[1].max()
    # Add a small padding to avoid cutting off edges
    padding = 5  # 5 pixels padding
    min_y = max(0, min_y - padding)
    min_x = max(0, min_x - padding)
    max_y = min(occupancy_grid.shape[0], max_y + padding)
    max_x = min(occupancy_grid.shape[1], max_x + padding)
    cropped_grid = occupancy_grid[min_y:max_y+1, min_x:max_x+1]
else:
    cropped_grid = occupancy_grid
    min_x, min_y = 0, 0

# Convert to PNG format (using RGB)
# Same approach as PGM: 0 -> black, 205 -> light gray, -1 -> gray
grid_for_png = np.zeros((cropped_grid.shape[0], cropped_grid.shape[1], 3), dtype=np.uint8)

# Assign colors to the grid (same as in PGM)
grid_for_png[cropped_grid == 100] = [0, 0, 0]       # Occupied -> black
grid_for_png[cropped_grid == -1] = [169, 169, 169]   # Unknown -> gray (darker)
grid_for_png[cropped_grid == 0] = [205, 205, 205]    # Free -> light gray

# Check unique values in the grid (flatten the array)
print("Unique values in grid_for_png:", np.unique(grid_for_png.reshape(-1, 3), axis=0))

# Save the grid as a PNG image
img = Image.fromarray(grid_for_png)
img.save("map.png")

print("Conversion successful. map.png has been generated.")

