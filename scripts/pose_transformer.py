import csv

# Constants for the transformation
REAL_WORLD_ORIGIN_X = 1375  # Image frame origin x in pixels
REAL_WORLD_ORIGIN_Y = 1913  # Image frame origin y in pixels
RESOLUTION = 0.02  # Meters to pixels conversion

# Path to input CSV file (camera pose data)
input_file_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/camera_pose_data.csv'

# Output file for transformed pose data
output_file_path = '/home/rainier/ros2_ws/src/littleslam_ros2/scripts/transformed_data.csv'

# Read the CSV data, transform, and write the results
def transform_pose_data():
    with open(input_file_path, mode='r') as infile:
        csv_reader = csv.reader(infile)
        
        # Read header and data
        header = next(csv_reader)  # Skip the header
        rows = list(csv_reader)

    # Prepare the header for the output file
    output_header = ['Timestamp (sec)', 'frame_id', 'x_image', 'y_image', 'theta']
    
    # List to store transformed data
    transformed_data = []
    
    # Transform each row based on the image frame coordinates
    for row in rows:
        timestamp = float(row[0])
        frame_id = row[1]
        x_real = float(row[2])
        y_real = float(row[3])
        theta = float(row[4])
        
        # Apply the transformation to convert to image frame coordinates
        x_image = (x_real / RESOLUTION) + REAL_WORLD_ORIGIN_X
        y_image = (REAL_WORLD_ORIGIN_Y - y_real / RESOLUTION)
        
        # Append the transformed data
        transformed_data.append([timestamp, frame_id, x_image, y_image, theta])

    # Write the transformed data into the new CSV file
    with open(output_file_path, mode='w', newline='') as outfile:
        csv_writer = csv.writer(outfile)
        csv_writer.writerow(output_header)  # Write the header
        csv_writer.writerows(transformed_data)  # Write the transformed rows

    print(f"Transformation complete. Transformed data saved to {output_file_path}")

# Call the function to perform the transformation
transform_pose_data()

