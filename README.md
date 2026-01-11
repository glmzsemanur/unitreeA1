This repository is built for a project within the scope of Istanbul Technical University, KON414E - Principles of Robot Autonomy course. 

The aim is to generate elevation map of the environemnt. Use the elevation map to obtain occupancy maps with user defined resolution parameter. Use this map to show the wellness of environmental representation and to obtain cost maps for navigation. Navigate the robot to see if the resolution is sufficient for navigation or not.

A demonstration video is available at https://drive.google.com/file/d/1OTMi_gg8kTEEB-TXAcgTcGTPRKue7dY4/view

The report is available at https://www.overleaf.com/read/bfwtjhpvvhvk#e5b750

#### To run the Gazebo and Rviz with world and robot spawned in it:
```bash
roslaunch unitree_gazebo robot_simulation_modified.launch
```
Relaunch (unfortunately up to 10 times) in case: 

[ERROR] The setTau function meets NaN.
Robot not standing up initially (sliding, laying down).
Velodyne points in Rviz is behaves like a disco.

If none above occured, congrats, you have succesfully launched it.

#### To run the elevation mapping with resolution parameter:
```bash
roslaunch elevation_mapping_demos elevation_map.launch elevation_r:=0.05 
```
I suggest minimum 0.05 for stability. Check if the elevation map has reached to Rviz before going any further. If not its probably because, its slow and laggy, increase the resolution for faster rendering.

#### To run the octomap and costmaps:
```bash
roslaunch elevation_mapping_demos octomap.launch occupancy_r:=0.05
```
The resolution shall be no smaller than the elevation resolution. Otherwise it will create gaps in the octomap.
Check if the octomap has reached to Rviz by toggling the octomap from the topics list (its quite shy and does not come up unless you call it this way)
Known issue: When the base is further away than odom, the octomap only renders at spesific viewing angles. 
Try changing the viewpoint in Rviz (zoom in/out, move around etc) or increase resolution.

#### To run the move_base:
```bash
roslaunch elevation_mapping_demos move_base_org.launch
```
The robot is initially at odom, with unkown cost. Move the robot around (using WASD JL in the first terminal) a little to exit the unkown region. In high resolutions, the bridge may become completly lethal, in this case you can lower the inflation_radius in src/elevation_mapping/elevation_mapping_demos/config/param/costmap_common_params.yaml.
Then, quickly press 2 then 5 (in the first terminal) to allow the move_base control of the robot. Try a couple times until it succeeds.
To go back to the keyboard control mode, quickly press 2 and 4.

Use Rviz to set a goal and observe the autonomous navigation of the robot.



