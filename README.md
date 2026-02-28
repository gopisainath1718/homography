# littleslam_ros2
![dashing](https://github.com/rsasaki0109/littleslam_ros2/workflows/CI/badge.svg)

[LittleSLAM](https://github.com/furo-org/LittleSLAM.git) ROS2 wrapper.
# Install
First, create a ROS 2 workspace and clone the repository with submodules:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/gopisainath1718/homography.git
colcon build --symlink-install
``` 

# Run  
Before running, always be sure to source the workspace:
```
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

```
source ~/ros2_ws/install/setup.bash
ros2 run littleslam_ros2 littleslam_ros2 /scan:=/base_scan
ros2 bag play Mapping1/
rviz2 -d config/demo.rviz
```

![demo](./image.png)  

