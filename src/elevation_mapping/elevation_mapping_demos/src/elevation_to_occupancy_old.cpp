#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>

class GridMapToOccupancy {
public:
  GridMapToOccupancy() {
    nh_ = ros::NodeHandle("~");
    // Change topic to your specific elevation map raw topic
    sub_ = nh_.subscribe("/elevation_mapping/elevation_map_raw", 1, &GridMapToOccupancy::callback, this);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  }

  void callback(const grid_map_msgs::GridMap& message) {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(message, map);

    const std::string layer = "elevation_smooth";
    
    if (!map.exists(layer)) return;

    nav_msgs::OccupancyGrid occupancyGrid;
    
    // This is the "Magic" function that handles the flip, the origin, and the buffer logic correctly
    grid_map::GridMapRosConverter::toOccupancyGrid(map, layer, 0, 100.0, occupancyGrid);

    // Explicitly set the -100 logic
    // The converter maps the range [-101, -99] to 100 (Occupied)
    // and everything else to 0 (Free)
    
    pub_.publish(occupancyGrid);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_to_occ_node");
  GridMapToOccupancy node;
  ros::spin();
  return 0;
}