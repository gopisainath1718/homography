import rclpy
from rclpy.node import Node
import rosbag2_py
import json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

# Paths
BAG_PATH = "/home/rainier/ros2_ws/localization_bag"
OUTPUT_JSON = "/home/rainier/ros2_ws/localization_data.json"

# Initialize rclpy
rclpy.init()

# Create a ROS 2 node
node = Node("extract_localization_data")

# Initialize the bag reader
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
reader.open(storage_options, converter_options)

# Get message types
path_msg_type = get_message("nav_msgs/msg/Path")
pose_msg_type = get_message("geometry_msgs/msg/PoseStamped")

# Initialize TF2 buffer and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

# Data storage
path_data = []
current_pose_data = []

# Topics to read
topics = ["/path", "/current_pose"]
topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

# Check available topics in the rosbag
print("Available topics in the rosbag:")
for topic in topic_types:
    print(f"Topic: {topic.name}, Type: {topic.type}")

# Check if /tf or /tf_static topics are available
tf_topics = [topic.name for topic in topic_types if topic.name in ["/tf", "/tf_static"]]
if not tf_topics:
    print("Warning: No /tf or /tf_static topics found in the rosbag. Skipping TF transformations.")
    use_tf = False
else:
    print("Found TF topics:", tf_topics)
    use_tf = True

# Process the bag
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic not in topics:
        continue

    # Deserialize the message
    if topic == "/path":
        msg = deserialize_message(data, path_msg_type)
        # Transform each pose in the path to the target frame (if TF is available)
        transformed_poses = []
        for pose_stamped in msg.poses:
            if use_tf:
                try:
                    # Try transforming to the map frame
                    target_frame = "map"
                    transform = tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, pose_stamped.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                    pose = transformed_pose.pose
                except Exception as e:
                    print(f"Failed to transform pose to map frame: {e}")
                    # Fallback to odom frame
                    try:
                        target_frame = "odom"
                        transform = tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, pose_stamped.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
                        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                        pose = transformed_pose.pose
                    except Exception as e:
                        print(f"Failed to transform pose to odom frame: {e}")
                        # Use the pose as-is
                        pose = pose_stamped.pose
            else:
                # Use the pose as-is if TF is not available
                pose = pose_stamped.pose

            transformed_poses.append({
                "x": pose.position.x,
                "y": pose.position.y,
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w
                }
            })
            # Debug: Print the position of the last pose in the path
            if transformed_poses:
                last_pose = transformed_poses[-1]
                print(f"Path pose at timestamp {t}: x={last_pose['x']}, y={last_pose['y']}")
        if transformed_poses:
            path_data = transformed_poses  # Update path_data with the latest transformed poses

    elif topic == "/current_pose":
        msg = deserialize_message(data, pose_msg_type)
        if use_tf:
            try:
                # Try transforming to the map frame
                target_frame = "map"
                transform = tf_buffer.lookup_transform(target_frame, msg.header.frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
                transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
                pose = transformed_pose.pose
            except Exception as e:
                print(f"Failed to transform current pose to map frame: {e}")
                # Fallback to odom frame
                try:
                    target_frame = "odom"
                    transform = tf_buffer.lookup_transform(target_frame, msg.header.frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
                    pose = transformed_pose.pose
                except Exception as e:
                    print(f"Failed to transform current pose to odom frame: {e}")
                    # Use the pose as-is
                    pose = msg.pose
        else:
            # Use the pose as-is if TF is not available
            pose = msg.pose

        current_pose_data.append({
            "x": pose.position.x,
            "y": pose.position.y,
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        })
        # Debug: Print the position of the current pose
        print(f"Current pose at timestamp {t}: x={pose.position.x}, y={pose.position.y}")

# Save to JSON
data = {
    "path": path_data,
    "current_pose": current_pose_data
}
with open(OUTPUT_JSON, "w") as f:
    json.dump(data, f, indent=4)

print(f"Saved localization data to {OUTPUT_JSON}")
print(f"Number of path points: {len(path_data)}")
print(f"Number of current pose points: {len(current_pose_data)}")

# Cleanup
node.destroy_node()
rclpy.shutdown()
