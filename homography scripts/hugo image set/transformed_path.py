import csv
import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math

# === CONFIG ===
CSV_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_hugo.csv"
MAP_IMAGE_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
IMAGE_FOLDER = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set"
TOTAL_FRAMES = 162
ORIGIN = (1158, 1362)
THETA_DEG = 84  # degrees CCW

def load_and_sample_points(csv_path, total_frames):
    df = pd.read_csv(csv_path)
    all_points = list(zip(df['x_image'], df['y_image']))
    if len(all_points) < total_frames:
        raise ValueError("Not enough points to sample 162")
    step = len(all_points) / total_frames
    sampled = [all_points[int(i * step)] for i in range(total_frames)]
    return sampled

def shift_points_to_origin(points, origin):
    first_x, first_y = points[0]
    dx = origin[0] - first_x
    dy = origin[1] - first_y
    return [(x + dx, y + dy) for (x, y) in points]

def rotate_points(points, origin, theta_deg):
    ox, oy = origin
    theta_rad = math.radians(theta_deg)
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    rotated = []
    for x, y in points:
        dx = x - ox
        dy = y - oy
        rx = ox + dx * cos_theta - dy * sin_theta
        ry = oy + dx * sin_theta + dy * cos_theta
        rotated.append((rx, ry))
    return rotated

def rotate_map_clockwise(image):
    return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

def main():
    sampled_points = load_and_sample_points(CSV_PATH, TOTAL_FRAMES)
    shifted_points = shift_points_to_origin(sampled_points, ORIGIN)
    rotated_points = rotate_points(shifted_points, ORIGIN, theta_deg=THETA_DEG)

    map_img = mpimg.imread(MAP_IMAGE_PATH)
    rotated_map = rotate_map_clockwise(map_img)

    image_filenames = [f"frame_{i:04d}.png" for i in range(len(rotated_points))]
    xs = [p[0] for p in rotated_points]
    ys = [p[1] for p in rotated_points]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(rotated_map)
    ax.scatter(xs, ys, c='red', s=15)

    # Image annotation setup
    init_img = cv2.cvtColor(cv2.imread(os.path.join(IMAGE_FOLDER, image_filenames[0])), cv2.COLOR_BGR2RGB)
    imagebox = OffsetImage(init_img, zoom=0.3)
    annot = AnnotationBbox(imagebox, (xs[0], ys[0]), frameon=True, pad=0.5)
    annot.set_visible(False)
    ax.add_artist(annot)

    def on_click(event):
        if event.inaxes != ax:
            return
        x_click, y_click = event.xdata, event.ydata
        distances = [(x - x_click)**2 + (y - y_click)**2 for x, y in zip(xs, ys)]
        idx = int(np.argmin(distances))
        x, y = xs[idx], ys[idx]
        img_path = os.path.join(IMAGE_FOLDER, image_filenames[idx])
        if os.path.exists(img_path):
            img = cv2.imread(img_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            imagebox.set_data(img)
            annot.xybox = (x, y)
            annot.xy = (x, y)
            annot.set_visible(True)
            fig.canvas.draw_idle()
            print(f"[INFO] Clicked {idx}: {img_path}")
        else:
            print(f"[WARN] Missing image: {img_path}")

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.title("Click a path point to view its corresponding image (rotated 6° CCW)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

