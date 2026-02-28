import cv2
import numpy as np
import matplotlib.pyplot as plt

# File paths
image1_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/map_alpha_processed.png'
image2_path = '/home/rainier/ros2_ws/src/littleslam_ros2/homography scripts/map_glor_new.png'

# Global list to store clicked points
clicked_points = []
fig = None  # To hold the matplotlib figure instance

def onclick(event):
    """Mouse click callback to store 8 points."""
    global clicked_points, fig
    if event.xdata is not None and event.ydata is not None and len(clicked_points) < 8:
        clicked_points.append((event.xdata, event.ydata))
        plt.plot(event.xdata, event.ydata, 'ro')
        plt.draw()
        if len(clicked_points) == 8:
            print("Collected 8 points, closing figure...")
            plt.close(fig)

def get_points(image_path, title):
    """Display image and collect 8 user-selected points."""
    global clicked_points, fig
    clicked_points = []

    img = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
    fig = plt.figure()
    plt.imshow(img)
    plt.title(title)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    print(f"[{title}] Please click exactly 8 points...")
    plt.show()

    if len(clicked_points) != 8:
        raise RuntimeError("You must click exactly 8 points.")
    
    return np.array(clicked_points, dtype=np.float32)

def main():
    pts1 = get_points(image1_path, "Image 1")
    pts2 = get_points(image2_path, "Image 2")

    H, status = cv2.findHomography(pts2, pts1, method=0)

    print("\nHomography matrix (Image2 to Image1):")
    print(H)

if __name__ == "__main__":
    main()
