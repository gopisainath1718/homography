Step-1: Collect a Rosbag with /scan topic with your desired path.
Step-2: Execute the save_pcd.py (/littleslam_ros/scripts/save_pcd.py) while playing the rosbag you collected.
Step-3: Now you have all the pcd data related to the map. Run the convert_pcd_to_pgm.py that saves .pgm map and .yaml config file with appropriate resolution (change if you intend to).
Step-4: Run convert_pcd_to_png.py that saves the pcd data to .png image.
Step-5: Now you have the map image, you need the data to be plotted on top of the map. Move to /homography\scripts folder to find pose_points_collector_1.py. Run this script while playing the bag, it will listen to the /current_pose topic and records all the pose points in real world coordinates (x, y, theta z).
Step-6: To plot these points on the image (.png), you need to have the transformed pose points which you can collect by giving the output of step-5 (collected_pose_points_[name].csv), origin where the localization is beginning from (in pixels (x(px),y(px))), resolution (default=0.02) and that respective map image (.png). Run transformer_poses_new.py for that, you'll get converted_pose_points_[name].csv.
Step-7: Now you have the path of your trajectory plotted on your map. To plot another path/trajectory on this very map, you need to run homography transformation first. To do that, find the homography_generator.py in the /homography\scripts folder and edit the inputs (map_original and map_new). 
Step-8: Upon running the script, a matplotlib dialog box opens up allowing you to select upto 8 points on each map (make sure the real world features of these 8 point remain the same on both the maps to get the exact [H]).
Step-9: Now, you have the homography matrix [H] of the new map with respect to the original map. Use this along with the original map image, converted pose points of original map and converted pose points of new map in dualism.py to plot both the paths on the original map image.
Step-10: Repeat the process with trialism.py and fourway.py to plot three or four paths together respectively.

origin of alpha: (1375, 1913)
reolution of alpha: 0.02

origin of fury:(1083, 1801)
resolution of fury: 0.02

origin of hugo: (1340, 1044) 
resolution of fury: 0.02

origin of glo: (1148, 1657) 
resolution of fury: 0.02
