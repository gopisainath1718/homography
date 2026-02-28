import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# === CONFIG ===
original_csv = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/transformed_data.csv'
fury_csv = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/fury_transformed_pose_data.csv'
map_image_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/map_alpha_processed.png'

def read_csv(csv_file_path, theta_field='theta'):
    x_coords, y_coords, theta_vals = [], [], []
    with open(csv_file_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row['x_image'])
                y = float(row['y_image'])
                theta = float(row.get(theta_field, 0))
                x_coords.append(x)
                y_coords.append(y)
                theta_vals.append(theta)
            except (ValueError, KeyError):
                continue
    return np.array(x_coords), np.array(y_coords), np.array(theta_vals)

def plot_robot_paths():
    # Read both paths
    x1, y1, theta1 = read_csv(original_csv, theta_field='theta')
    x2, y2, theta2 = read_csv(fury_csv, theta_field='theta_rad')

    img = mpimg.imread(map_image_path)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img)

    # Plot original path (green)
    ax.plot(x1, y1, color='green', linewidth=2, label='Original Path')
    ax.scatter(x1[0], y1[0], color='blue', label='Start (Original)')
    ax.scatter(x1[-1], y1[-1], color='red', label='End (Original)')
    step1 = max(1, len(x1) // 100)
    ax.quiver(
        x1[::step1], y1[::step1],
        np.cos(theta1[::step1]) * 10, np.sin(theta1[::step1]) * 10,
        angles='xy', scale_units='xy', scale=1, color='orange', width=0.003
    )

    # Plot fury path (purple)
    ax.plot(x2, y2, color='purple', linewidth=2, label='Fury Path')
    ax.scatter(x2[0], y2[0], color='magenta', label='Start (Fury)')
    ax.scatter(x2[-1], y2[-1], color='black', label='End (Fury)')
    step2 = max(1, len(x2) // 100)
    ax.quiver(
        x2[::step2], y2[::step2],
        np.cos(theta2[::step2]) * 10, np.sin(theta2[::step2]) * 10,
        angles='xy', scale_units='xy', scale=1, color='purple', width=0.003
    )

    # Plot formatting
    ax.set_xlim(0, img.shape[1])
    ax.set_ylim(img.shape[0], 0)  # Invert y-axis to match image coordinates
    ax.set_title("Comparison: Original vs Fury Path")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.legend()
    ax.set_aspect('equal')
    plt.grid(False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_robot_paths()
