#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/OccupancyGrid.h>

#include <cmath>
#include <memory>

class OctomapToOccupancySurface
{
public:
  OctomapToOccupancySurface()
  {
    ros::NodeHandle nh, pnh("~");

    pnh.param("height_threshold", height_threshold_, 0.15);
    pnh.param("max_step_height", max_step_height_, 0.2);
    pnh.param("unknown_is_free", unknown_is_free_, true);

    octomap_sub_ = nh.subscribe("/octomap", 1,
        &OctomapToOccupancySurface::callback, this);

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    ROS_INFO("Octomap → column-based surface OccupancyGrid node started");
  }

private:
  ros::Subscriber octomap_sub_;
  ros::Publisher map_pub_;

  double height_threshold_;
  double max_step_height_;
  bool unknown_is_free_;

  void callback(const octomap_msgs::OctomapConstPtr& msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_abs(
        octomap_msgs::fullMsgToMap(*msg));

    if (!tree_abs)
      return;

    auto* tree = dynamic_cast<octomap::OcTree*>(tree_abs.get());
    if (!tree)
      return;

    const double res = tree->getResolution();

    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    int width  = std::ceil((max_x - min_x) / res);
    int height = std::ceil((max_y - min_y) / res);

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = res;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(width * height, unknown_is_free_ ? 0 : -1);

    // ----- COLUMN-BASED RAY CASTING -----
    for (int iy = 0; iy < height; ++iy)
    {
      for (int ix = 0; ix < width; ++ix)
      {
        double x = min_x + (ix + 0.5) * res;
        double y = min_y + (iy + 0.5) * res;

        bool found_ground = false;
        double ground_z = 0.0;
        double highest_z = -1e9;

        for (double z = min_z; z <= max_z; z += res * 0.5)
        {
          auto* node = tree->search(x, y, z);
          if (!node || !tree->isNodeOccupied(node))
            continue;

          if (!found_ground)
          {
            ground_z = z;
            found_ground = true;
          }

          highest_z = z;
        }

        if (!found_ground)
          continue;

        double column_height = highest_z - ground_z;

        bool occupied =
            (ground_z > height_threshold_) ||
            (column_height > max_step_height_);

        int idx = iy * width + ix;
        grid.data[idx] = occupied ? 100 : 0;
      }
    }

    map_pub_.publish(grid);
    ROS_INFO_THROTTLE(2.0, "Published column-based occupancy grid");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_to_occupancy_surface");
  OctomapToOccupancySurface node;
  ros::spin();
  return 0;
}
