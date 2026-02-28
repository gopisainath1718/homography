import os
import json
import math
import random
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.table import Table

# === CONFIGURATION ===
CSV_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/converted_pose_pixels_hugo.csv"
MAP_IMAGE_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
SENSOR_JSON_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set/particle_sensor_data.json"
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

def generate_mock_sensor_data(num_frames):
    mock_data = []
    for i in range(num_frames):
        pollen_level = random.randint(0, 10)
        mold_level = random.randint(0, 5)
        mites_level = random.randint(0, 5)
        dander_level = random.randint(0, 5)
        allergen_level = random.randint(0, 10)
        mock_data.append({
            "sensor_timestamp": f"2025-03-19 00:{43 + i//60:02d}:{i%60:02d}",
            "data": {
                "levels": {
                    "pollen": {"level": pollen_level, "description": level_description(pollen_level)},
                    "mold": {"level": mold_level, "description": level_description(mold_level)},
                    "mites": {"level": mites_level, "description": level_description(mites_level)},
                    "dander": {"level": dander_level, "description": level_description(dander_level)},
                    "allergen": {"level": allergen_level, "description": level_description(allergen_level)}
                },
                "values": {
                    "pollen": {"value": random.randint(0, 200)},
                    "mold": {"value": random.randint(0, 100)},
                    "mites": {"value": random.randint(0, 20)},
                    "other": {"value": random.randint(0, 200)},
                    "dander": {"value": random.randint(0, 10)}
                }
            }
        })
    return mock_data

def level_description(level):
    if level <= 2:
        return "Low"
    elif 3 <= level <= 5:
        return "Moderate"
    elif 6 <= level <= 7:
        return "High"
    else:
        return "Very High"

def load_sensor_data(json_path, total_frames):
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        keys = list(data.keys())
        sensor_list = [data[k] for k in keys]
        if len(sensor_list) < total_frames:
            print("[WARNING] Not enough sensor entries — generating mock data...")
            extra_needed = total_frames - len(sensor_list)
            mock = generate_mock_sensor_data(extra_needed)
            sensor_list.extend(mock)
        return sensor_list[:total_frames]
    except Exception as e:
        print(f"[ERROR] Failed to load real sensor data, generating full mock: {e}")
        return generate_mock_sensor_data(total_frames)

def get_color_for_pollen_level(level):
    if level <= 2:
        return "green"
    elif 3 <= level <= 5:
        return "yellow"
    elif 6 <= level <= 7:
        return "orange"
    else:
        return "red"

def main():
    sampled_points = load_and_sample_points(CSV_PATH, TOTAL_FRAMES)
    shifted_points = shift_points_to_origin(sampled_points, ORIGIN)
    rotated_points = rotate_points(shifted_points, ORIGIN, theta_deg=THETA_DEG)

    map_img = mpimg.imread(MAP_IMAGE_PATH)
    rotated_map = rotate_map_clockwise(map_img)

    sensor_data_list = load_sensor_data(SENSOR_JSON_PATH, TOTAL_FRAMES)

    xs = [p[0] for p in rotated_points]
    ys = [p[1] for p in rotated_points]
    pollen_levels = [d['data']['levels']['pollen']['level'] for d in sensor_data_list]
    colors = [get_color_for_pollen_level(level) for level in pollen_levels]

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(rotated_map)
    scatter = ax.scatter(xs, ys, c=colors, s=20)

    def on_click(event):
        if event.inaxes != ax:
            return
        x_click, y_click = event.xdata, event.ydata
        distances = [(x - x_click)**2 + (y - y_click)**2 for x, y in zip(xs, ys)]
        idx = int(np.argmin(distances))
        sensor_info = sensor_data_list[idx]
        
        fields = {
            "Timestamp": sensor_info['sensor_timestamp'],
            "Pollen Level": sensor_info['data']['levels']['pollen']['description'],
            "Mold Level": sensor_info['data']['levels']['mold']['description'],
            "Mites Level": sensor_info['data']['levels']['mites']['description'],
            "Dander Level": sensor_info['data']['levels']['dander']['description'],
            "Allergen Level": sensor_info['data']['levels']['allergen']['description'],
            "Pollen Value": sensor_info['data']['values']['pollen']['value'],
            "Mold Value": sensor_info['data']['values']['mold']['value'],
            "Mites Value": sensor_info['data']['values']['mites']['value'],
            "Other Value": sensor_info['data']['values']['other']['value'],
            "Dander Value": sensor_info['data']['values']['dander']['value']
        }

        fig2, ax2 = plt.subplots(figsize=(6, 6))
        ax2.set_axis_off()
        table = Table(ax2, bbox=[0, 0, 1, 1])

        n_rows = len(fields)
        width, height = 1.0, 1.0 / n_rows

        for i, (key, value) in enumerate(fields.items()):
            table.add_cell(i, 0, width/2, height, text=key, loc='center')
            table.add_cell(i, 1, width/2, height, text=str(value), loc='center')

        ax2.add_table(table)
        plt.title(f"Sensor Data for Point #{idx}")
        plt.show()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.title("Click a path point to view sensor data (mock if needed)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

