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
ENV_SENSOR_JSON_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set/environment_sensor_data.json"
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

def generate_mock_env_sensor_data(num_frames):
    mock_data = []
    for i in range(num_frames):
        voc_level = random.randint(0, 10)
        co2_level = random.randint(0, 10)
        humidity_level = random.randint(0, 10)
        temperature_level = random.randint(0, 10)
        pollution_level = random.randint(0, 10)
        mock_data.append({
            "sensor_timestamp": f"2025-03-19 00:{44 + i//60:02d}:{i%60:02d}",
            "data": {
                "date_time": f"2025-03-19 00:{44 + i//60:02d}:{i%60:02d}",
                "values": {
                    "voc": round(random.uniform(0, 600), 2),
                    "co2": round(random.uniform(400, 1000), 2),
                    "humidity": round(random.uniform(10, 90), 2),
                    "temperature": round(random.uniform(18, 30), 2)
                },
                "levels": {
                    "voc": {"level": voc_level, "description": level_description(voc_level)},
                    "co2": {"level": co2_level, "description": level_description(co2_level)},
                    "humidity": {"level": humidity_level, "description": level_description(humidity_level)},
                    "temperature": {"level": temperature_level, "description": level_description(temperature_level)},
                    "pollution": {"level": pollution_level, "description": level_description(pollution_level)}
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

def load_env_sensor_data(json_path, total_frames):
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        keys = list(data.keys())
        sensor_list = [data[k] for k in keys]
        if len(sensor_list) < total_frames:
            print("[WARNING] Not enough environmental sensor entries — generating mock data...")
            extra_needed = total_frames - len(sensor_list)
            mock = generate_mock_env_sensor_data(extra_needed)
            sensor_list.extend(mock)
        return sensor_list[:total_frames]
    except Exception as e:
        print(f"[ERROR] Failed to load real environmental sensor data, generating full mock: {e}")
        return generate_mock_env_sensor_data(total_frames)

def get_color_for_pollution_level(level):
    if level <= 2:
        return "#800080"  # Dark Purple
    elif 3 <= level <= 5:
        return "#FF00FF"  # Magenta
    else:
        return "#8A2BE2"  # Violet

def main():
    sampled_points = load_and_sample_points(CSV_PATH, TOTAL_FRAMES)
    shifted_points = shift_points_to_origin(sampled_points, ORIGIN)
    rotated_points = rotate_points(shifted_points, ORIGIN, theta_deg=THETA_DEG)

    map_img = mpimg.imread(MAP_IMAGE_PATH)
    rotated_map = rotate_map_clockwise(map_img)

    env_sensor_data_list = load_env_sensor_data(ENV_SENSOR_JSON_PATH, TOTAL_FRAMES)

    xs = [p[0] for p in rotated_points]
    ys = [p[1] for p in rotated_points]
    pollution_levels = [d['data']['levels']['pollution']['level'] for d in env_sensor_data_list]
    colors = [get_color_for_pollution_level(level) for level in pollution_levels]

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(rotated_map)
    scatter = ax.scatter(xs, ys, c=colors, s=20)

    def on_click(event):
        if event.inaxes != ax:
            return
        x_click, y_click = event.xdata, event.ydata
        distances = [(x - x_click)**2 + (y - y_click)**2 for x, y in zip(xs, ys)]
        idx = int(np.argmin(distances))
        sensor_info = env_sensor_data_list[idx]

        fields = {
            "Timestamp": sensor_info['sensor_timestamp'],
            "VOC Level": sensor_info['data']['levels']['voc']['description'],
            "CO2 Level": sensor_info['data']['levels']['co2']['description'],
            "Humidity Level": sensor_info['data']['levels']['humidity']['description'],
            "Temperature Level": sensor_info['data']['levels']['temperature']['description'],
            "Pollution Level": sensor_info['data']['levels']['pollution']['description'],
            "VOC Value": sensor_info['data']['values']['voc'],
            "CO2 Value": sensor_info['data']['values']['co2'],
            "Humidity Value": sensor_info['data']['values']['humidity'],
            "Temperature Value": sensor_info['data']['values']['temperature']
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
        plt.title(f"Environmental Sensor Data for Point #{idx}")
        plt.show()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.title("Click a path point to view environmental sensor data (mock if needed)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
