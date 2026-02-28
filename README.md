# littleslam_ros2
![dashing](https://github.com/rsasaki0109/littleslam_ros2/workflows/CI/badge.svg)

[LittleSLAM](https://github.com/furo-org/LittleSLAM.git) ROS2 wrapper.

## About This Repository (and Why Homography is Needed)

This package contains a lightweight SLAM (Simultaneous Localization and Mapping) implementation designed for ROS 2. It takes 2D laser scan data (`/scan`) and generates point-cloud maps and trajectories in real-time. 

Alongside the core SLAM mapping, this specific fork/repository introduces a powerful **Homography Pipeline**. When you map the same environment in multiple separate SLAM sessions (e.g., driving different paths or using different robots), the starting origin and orientation of each generated map usually differ. Therefore, you cannot simply overlay the path trajectories on top of each other directly. 

The **Homography Scripts** mathematically align (warp/rotate/translate) the coordinate frame of subsequent maps onto a single unified base map. Thus, we can successfully extract the pose data (trajectories) from different sessions and plot them interchangeably on the same master floor plan for accurate comparison!

---

# Install
First, create a ROS 2 workspace and clone the repository with submodules:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/gopisainath1718/homography.git
colcon build --symlink-install
``` 

# Run  
Before running, always be sure to source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run littleslam_ros2 littleslam_ros2  
```
# IO  
sub  
- sensor_msgs::msg::LaserScan ("scan")
- odom (tf2 "base_link"->"odom")(optional,with use_odom:=true)  

pub  
- sensor_msgs::msg::PointCloud2 ("icp_map")
- nav_msgs::msg::Path ("path")
- nav_msgs::msg::PoseStamped ("current_pose")

# Demo
data:[ros.org Introduction to Working With Laser Scanner Data](http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData) (using [rosbags](https://pypi.org/project/rosbags/))

```bash
source ~/ros2_ws/install/setup.bash
ros2 run littleslam_ros2 littleslam_ros2 /scan:=/base_scan
ros2 bag play Mapping1/
rviz2 -d config/demo.rviz
```

![demo](./image.png)  


# Homography and Path Overlay Pipeline

This repository includes custom scripts to collect path data and overlay multiple trajectories onto a single map using homography transformations.

## Workflow

The pipeline for generating overlayed maps is as follows:

1. **Collect Map Data:** Play your rosbag and execute the script below to save the point cloud data (`.pcd`):
   ```bash
   ros2 run littleslam_ros2 littleslam_ros2 /scan:=/base_scan
   ros2 bag play <your_bag_file>/
   python3 scripts/save_pcd.py
   ```

2. **Convert to PGM/YAML:** Run the conversion script to generate the `.pgm` map and `.yaml` config (you can adjust the resolution inside the script if needed):
   ```bash
   python3 scripts/convert_pcd_to_pgm.py
   ```

3. **Convert to PNG:** Run the PNG conversion script to save the generated map as a `.png` image:
   ```bash
   python3 scripts/convert_pcd_to_png.py
   ```

4. **Collect Pose Points:** Move into the `homography scripts/` folder. Run the pose collector script while playing the bag. This script listens to the `/current_pose` topic and records pose points in real-world coordinates `(x, y, theta_z)` to `collected_pose_points_[name].csv`:
   ```bash
   cd "homography scripts"
   python3 pose_points_collector_1.py
   ```

5. **Transform Pose Points to Pixels:** Run the pose transformer script. You will need to specify the origin of your map (in pixels), the resolution (default is 0.02), and the `.png` map image within the script. This process will output a `converted_pose_points_[name].csv`:
   ```bash
   python3 transformer_poses_new.py
   ```

6. **Generate Homography Matrix:** To plot another trajectory on this base map, you need to spatially align the two distinct maps. Open `homography_generator.py`, and supply the paths to your original and new map targets. A UI window will open—select exactly 8 corresponding feature points on both maps to compute the Homography matrix `[H]`:
   ```bash
   python3 homography_generator.py
   ```

7. **Overlay Paths:** Utilize the generated Homography matrix in conjunction with one of the provided overly scripts to plot multiple identically-aligned paths onto your base map:
   ```bash
   # Use dualism.py for 2 paths, trialism for 3, fourway for 4.
   python3 dualism.py
   ```
