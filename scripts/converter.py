import cv2
import numpy as np

# Step 1: Load the image
input_path = '/home/rainier/ros2_ws/src/littleslam_ros2/image2.png'
image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

if image is None:
    print("Error: Could not load the image. Check the file path.")
    exit()

# Step 2: Threshold the image to ensure binary values
# The image already has a white background (255) and black features (0), but there might be
# intermediate grayscale values (e.g., from anti-aliasing). We’ll force it to be strictly binary.
_, binary = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

# Step 3: Clean up the image (optional)
# Apply a light morphological operation to remove any small noise (e.g., stray pixels).
kernel = np.ones((3, 3), np.uint8)
cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

# Step 4: Ensure the map is suitable for AMCL
# AMCL expects 0 (black) for occupied spaces and 255 (white) for free spaces.
# The image is already in this format, but we confirm with a final threshold.
_, final_map = cv2.threshold(cleaned, 128, 255, cv2.THRESH_BINARY)

# Step 5: Save the image as a .pgm file in the same directory as the input
output_path = '/home/rainier/ros2_ws/src/littleslam_ros2/output_map.pgm'
cv2.imwrite(output_path, final_map)

print(f"Map has been saved as '{output_path}'.")

# Optional: Display the result (for debugging)
cv2.imshow('Final Map', final_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
