import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pandas as pd
import cv2
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# === Config ===
CSV_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set/path_with_images.csv"
MAP_IMAGE_PATH = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_alpha_processed.png"
IMAGE_FOLDER = "/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/hugo image set"

def load_data(csv_path):
    df = pd.read_csv(csv_path)
    print(f"[INFO] Loaded {len(df)} points from CSV.")
    return df

def rotate_image_clockwise(image):
    return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

def main():
    df = load_data(CSV_PATH)

    # Load and rotate the background map image
    map_img = mpimg.imread(MAP_IMAGE_PATH)
    rotated_map = rotate_image_clockwise(map_img)

    # Plot the rotated image
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(rotated_map)
    scatter = ax.scatter(df["x"], df["y"], c="red", s=15, label="Path Points")

    # Initial annotation box (empty)
    annot_img = cv2.imread(os.path.join(IMAGE_FOLDER, df.iloc[0]["image_filename"]))
    annot_img_rgb = cv2.cvtColor(annot_img, cv2.COLOR_BGR2RGB)
    imagebox = OffsetImage(annot_img_rgb, zoom=0.3)
    annot = AnnotationBbox(imagebox, (df.iloc[0]["x"], df.iloc[0]["y"]),
                           frameon=True, pad=0.5, box_alignment=(1, 0))
    annot.set_visible(False)
    ax.add_artist(annot)

    def on_click(event):
        if event.inaxes != ax:
            return

        x_click, y_click = event.xdata, event.ydata
        distances = ((df["x"] - x_click) ** 2 + (df["y"] - y_click) ** 2).pow(0.5)
        nearest_idx = distances.idxmin()

        x, y = df.iloc[nearest_idx][["x", "y"]]
        img_path = os.path.join(IMAGE_FOLDER, df.iloc[nearest_idx]["image_filename"])

        if os.path.exists(img_path):
            img = cv2.imread(img_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            imagebox.set_data(img)
            annot.xybox = (x, y)
            annot.xy = (x, y)
            annot.set_visible(True)
            fig.canvas.draw_idle()
            print(f"[INFO] Clicked on ({x:.1f}, {y:.1f}) → {img_path}")
        else:
            print(f"[WARN] Image not found: {img_path}")

    fig.canvas.mpl_connect("button_press_event", on_click)

    plt.title("Click on a point to view the corresponding image frame")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
