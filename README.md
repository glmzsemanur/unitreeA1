#### To run the Gazebo and Rviz with world and robot spawned in it:
```bash
roslaunch unitree_gazebo robot_simulation_modified.launch
```
Relaunch (up to 10 times) in case: [ERROR] The setTau function meets NaN.
Robot not standing up initially (sliding, laying down).
Velodyne points in Rviz is a disco.
If none above occured, congrats, youve succesfully launched it.

#### To run the elevation mapping with resolution parameter:
```bash
roslaunch elevation_mapping_demos elevation_map.launch elevation_r:=0.05 
```
I suggest minimum 0.05 for stability, if its slow and laggy, increase the resolution.
Chck if the elevation map has reached to Rviz before going further.

#### To run the octomap and costmaps:
```bash
roslaunch elevation_mapping_demos octomap.launch occupancy_r:=0.05
```
The resolution shall no smaller than the elevation resolution. 
Check if the octomap has reached to Rviz by toggling the octomap topic (its quite shy and does not come up unless you call it)
Known issue: When the base is further away than odom, the octomap only renders at spesific viewing angles. 
Try changing the viewpoint in Rviz (zoom in/out, move around etc).

#### To run the move_base:
```bash
roslaunch elevation_mapping_demos move_base_org.launch
```
The robot is initially at odom, with unkown cost. Move the robot around (using WASD JL in the first terminal) a little to exit the unkown region.
Then, quickly press 2 then 5 (in the first terminal) to allow the move_base control of the robot. Try a couple times until it succeeds.
To go back to the keyboard control mode, quickly press 2 and 4.





