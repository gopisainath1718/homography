import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# === CONFIG ===
csv_file_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/transformed_data.csv'
map_image_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/map_alpha_processed.png'  # Replace with correct filename

def read_csv(csv_file_path):
    x_coords = []
    y_coords = []
    theta_vals = []

    with open(csv_file_path, 'r') as f:
        reader = csv.DictReader(f)  # Use DictReader to access by column name
        for row in reader:
            try:
                x = float(row['x_image'])
                y = float(row['y_image'])
                theta = float(row['theta'])
                x_coords.append(x)
                y_coords.append(y)
                theta_vals.append(theta)
            except (ValueError, KeyError):
                continue  # Skip malformed lines or missing keys

    return np.array(x_coords), np.array(y_coords), np.array(theta_vals)

def plot_robot_path():
    x_coords, y_coords, theta_vals = read_csv(csv_file_path)
    if len(x_coords) == 0:
        print("⚠️ No valid data points found in CSV.")
        return

    img = mpimg.imread(map_image_path)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img)

    # Path
    ax.plot(x_coords, y_coords, color='green', linewidth=2, label='Path')

    # Start and End points
    ax.scatter(x_coords[0], y_coords[0], color='blue', label='Start')
    ax.scatter(x_coords[-1], y_coords[-1], color='red', label='End')

    # Draw orientation arrows (quiver)
    step = max(1, len(x_coords) // 100)  # Downsample for clarity if many points
    dx = np.cos(theta_vals[::step]) * 10  # Arrow length = 10 pixels
    dy = np.sin(theta_vals[::step]) * 10
    ax.quiver(
        x_coords[::step], y_coords[::step],
        dx, dy,
        angles='xy', scale_units='xy', scale=1, color='orange', width=0.003
    )

    # Set correct limits
    ax.set_xlim(0, img.shape[1])
    ax.set_ylim(img.shape[0], 0)  # Invert y-axis to match image coordinates
    ax.set_title("Robot Path over Map")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.legend()
    ax.set_aspect('equal')
    plt.grid(False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_robot_path()

